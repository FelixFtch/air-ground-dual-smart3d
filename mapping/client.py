"""Serialized JSONL client. One owner process can multiplex both robot sensors.

This module starts only the map executable; it never starts CARLA or ROS.
Point coordinates and ray origin must already be in the same capture-time map
frame. MapCache contains a read-only consumer copy, never a second mapper.
Explicit free_rays may represent a real LiDAR hit beyond the integration range:
the bridge crops the measured ray to its clearing range and labels its source as
lidar_measured_beyond_range. free_rays.include_endpoint=False excludes a real
far hit surface from its own ray's free evidence. Complete LiDAR frames also
send protected_hit_points plus mark_range_m: these are original, in-bounds real
returns between marking and clearing range, not cropped endpoints. The mapper
excludes their voxels from *every* free ray in that frame. Real far returns apply
normal hit evidence only to cells previously marked by an accepted in-range hit,
even if later free evidence cleared that cell. They cannot create new obstacles
in never-marked far space. This marking provenance is distinct from observed
free-space metadata, is saved/restored, and is removed by explicit forget.
Later measured free rays still clear known surfaces normally; no lock or TTL is
introduced. The declared mark_range_m also bounds new points in that request.
Missing returns remain unknown, never free.
"""
from __future__ import annotations

from sim_io import json_codec as json
import math
import base64
import binascii
import struct
import os
from pathlib import Path
import selectors
import subprocess
import threading
import time


FREE_RAY_SOURCES = frozenset({
    "registered_depth_measured_beyond_range",
    "carla_depth_far_plane",
    "lidar_measured_beyond_range",
})


class MapError(RuntimeError):
    def __init__(self, response):
        self.response = response
        self.code = response.get("error", "map_error")
        super().__init__(f"{self.code}: {response.get('message', '')}")


class OctomapClient:
    def __init__(self, executable, *, timeout=20.0, profile_transport=False):
        self.timeout = timeout
        self.profile_transport = bool(profile_transport)
        self._lock = threading.Lock()
        self._request_id = 0
        self._buffer = bytearray()
        self.process = subprocess.Popen(
            [str(Path(executable).resolve())], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=None, bufsize=0,
        )

    def request(self, op, **fields):
        started = time.monotonic() if self.profile_transport else None
        with self._lock:
            acquired = time.monotonic() if self.profile_transport else None
            if self.process.poll() is not None:
                raise RuntimeError("OctoMap service has exited")
            self._request_id += 1
            payload = dict(fields, op=op, request_id=self._request_id)
            encoded = (json.dumps(payload, allow_nan=False, separators=(",", ":")) + "\n").encode()
            encoded_at = time.monotonic() if self.profile_transport else None
            # pipe writes may be partial (especially point clouds).
            remaining = memoryview(encoded)
            while remaining:
                count = self.process.stdin.write(remaining)
                if not count:
                    raise RuntimeError("OctoMap stdin closed")
                remaining = remaining[count:]
            written_at = time.monotonic() if self.profile_transport else None
            deadline = time.monotonic() + self.timeout
            with selectors.DefaultSelector() as selector:
                selector.register(self.process.stdout, selectors.EVENT_READ)
                while b"\n" not in self._buffer:
                    duration = deadline - time.monotonic()
                    if duration <= 0 or not selector.select(duration):
                        # Do not send more work after timeout: its late response
                        # would otherwise be mistaken for the next request.
                        self.process.terminate()
                        self.process.wait(timeout=5)
                        raise TimeoutError("OctoMap response timeout; service terminated")
                    chunk = os.read(self.process.stdout.fileno(), 65536)
                    if not chunk:
                        raise RuntimeError("OctoMap stdout closed")
                    self._buffer.extend(chunk)
            line, _, rest = self._buffer.partition(b"\n")
            self._buffer = bytearray(rest)
            received_at = time.monotonic() if self.profile_transport else None
            response = json.loads(line)
            if self.profile_transport:
                decoded_at = time.monotonic()
                response["_transport_timing"] = {
                    "wait_lock_s": acquired - started,
                    "encode_s": encoded_at - acquired,
                    "write_s": written_at - encoded_at,
                    # Includes server execution, response transfer, and pipe
                    # buffering; it is not a standalone C++ compute timer.
                    "await_s": received_at - written_at,
                    "decode_s": decoded_at - received_at,
                    "response_bytes": len(line),
                    "request_bytes": len(encoded),
                }
            if response.get("request_id") != self._request_id:
                raise RuntimeError("OctoMap response identity mismatch")
            if not response.get("ok"):
                raise MapError(response)
            return response

    def close(self):
        if self.process.stdin:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=5)
        if self.process.stdout:
            self.process.stdout.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class MapCache:
    """Consumer sparse snapshot with strict map identity and revision checks."""
    IDENTITY = ("scene_id", "scene_epoch", "content_version", "frame_id", "resolution")

    def __init__(self, snapshot):
        self.identity = {key: snapshot[key] for key in self.IDENTITY}
        self.revision = snapshot["revision"]
        self.cells = self._copy_cells(snapshot["cells"])

    @staticmethod
    def _copy_cells(cells):
        # Canonical immutable coordinates avoid retaining a tracked list in
        # every long-lived voxel record. CPython can then untrack these scalar
        # dictionaries after collection, rather than scanning millions of
        # containers during each generation-2 GC pause. JSON serialization is
        # unchanged: tuples encode as coordinate arrays.
        result = {}
        for cell in cells:
            key = tuple(cell["key"])
            result[key] = dict(cell, key=key)
        return result

    def apply(self, delta):
        if delta.get("refresh_encoding", "keys") not in ("keys", "i16le_base64"):
            raise ValueError("invalid_refresh_encoding")
        if delta.get("refresh_semantics", "full") != "full":
            raise ValueError("execution_cache_requires_full_freshness")
        if any(delta[key] != value for key, value in self.identity.items()):
            raise ValueError("map_identity_mismatch")
        if delta["base_revision"] != self.revision:
            raise ValueError("snapshot_required")
        if delta["revision"] < self.revision:
            raise ValueError("revision_regression")
        if any(cell["state"] not in ("unknown", "occupied", "free")
               or len(cell["key"]) != 3 for cell in delta["changed_cells"]):
            raise ValueError("invalid_cell_state")
        # Stage replacement cells and validate each refresh before any mutation.
        # Keep references for the commit pass instead of hashing/converting every
        # voxel key a second time (refresh-only deltas dominate live mapping).
        changes = self._copy_cells(delta["changed_cells"])
        refreshes = []
        current_cells = self.cells
        expanded_binary = 0
        for group in delta.get("refreshes", []):
            stamp = group["capture_sim_time"]
            if not isinstance(stamp, (float, int)) or not math.isfinite(stamp):
                raise ValueError("invalid_refresh_time")
            cells = []
            append = cells.append
            if "keys_i16le_b64" in group:
                count, encoded = group.get("key_count"), group["keys_i16le_b64"]
                if ("keys" in group or "key_runs" in group or type(count) is not int or
                        count < 0 or count > 2_000_000 or not isinstance(encoded, str) or len(encoded) != count * 8):
                    raise ValueError("invalid_binary_refresh_keys")
                expanded_binary += count
                if expanded_binary > 2_000_000:
                    raise ValueError("refresh_key_expansion_limit")
                try:
                    raw = base64.b64decode(encoded, validate=True)
                except (ValueError, binascii.Error):
                    raise ValueError("invalid_binary_refresh_keys") from None
                if len(raw) != count * 6:
                    raise ValueError("invalid_binary_refresh_keys")
                keys = struct.iter_unpack("<hhh", raw)
            else:
                keys = group["keys"]
            try:
                if changes:
                    for key in keys:
                        key = tuple(key)
                        current = changes[key] if key in changes else current_cells[key]
                        if current["state"] == "unknown":
                            raise ValueError("snapshot_required")
                        append(current)
                else:
                    for key in keys:
                        current = current_cells[tuple(key)]
                        if current["state"] == "unknown":
                            raise ValueError("snapshot_required")
                        append(current)
            except KeyError:
                raise ValueError("snapshot_required") from None
            refreshes.append((stamp, cells))
        for key, cell in changes.items():
            if cell["state"] == "unknown":
                self.cells.pop(key, None)
            else:
                self.cells[key] = cell
        for stamp, cells in refreshes:
            for cell in cells:
                previous = cell.get("last_observed_sim_time")
                if previous is None or stamp > previous:
                    cell["last_observed_sim_time"] = stamp
        self.revision = delta["revision"]

    def snapshot(self):
        # Materialize a detached wire-format snapshot only on explicit request.
        # The persistent consumer cache keeps immutable coordinate keys.
        return dict(self.identity, revision=self.revision,
                    cells=[dict(cell, key=list(key)) for key, cell in self.cells.items()])
