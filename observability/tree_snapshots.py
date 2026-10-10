"""Immutable display-only native tree snapshots, separate from fast state/paths.

The runtime's planner replies are immutable after publication. Retaining that
object lets the writer skip geometry hashing on the faster state timer. None of
these records authorizes motion; the original path and execution checks remain
in the runtime.
"""
from __future__ import annotations

from collections import OrderedDict
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import tempfile
import time

SCHEMA = "air_ground.tree_snapshot/v1"
_ID = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")
_SHA = re.compile(r"[0-9a-f]{64}\Z")
_IDENTITY = ("schema", "epoch", "robot_id", "session_id", "tree_instance_id")


def _encode(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def geometry_digest(planner):
    return hashlib.sha256(_encode({"tree_nodes": planner.get("tree_nodes") or [],
                                  "tree_edges": planner.get("tree_edges") or []})).hexdigest()


def reference_signature(ref):
    if not isinstance(ref, dict):
        return None
    # Identity matters even when geometry happens to match after a recovery.
    return tuple(ref.get(k) for k in (*_IDENTITY, "geometry_sha256", "sha256", "path"))


class TreeSnapshotWriter:
    """Write before state.json references a complete immutable version.

    Superseded files live for a six-second reader grace period (reader health
    limit is three seconds), plus the newest four versions. At the runtime's
    bounded 5 Hz publication rate this is a small rolling window, not an audit
    archive. Only the current planner object per robot is retained in memory.
    """
    def __init__(self, run_root, epoch, *, retain_versions=4, grace_s=6.,
                 max_cached_robots=32, clock=time.time):
        if retain_versions < 1 or grace_s < 3 or max_cached_robots < 1:
            raise ValueError("invalid tree snapshot retention")
        self.root = Path(run_root).resolve()
        self.epoch = epoch
        self.folder = self.root / "shared/tree_snapshots"
        self.retain_versions = retain_versions
        self.grace_s = grace_s
        self.max_cached_robots = max_cached_robots
        self.clock = clock
        self.entries = OrderedDict()
        self._last_cleanup = -math.inf

    def compact(self, robot_id, planner):
        try:
            return self._compact(robot_id, planner)
        except (OSError, ValueError, TypeError) as error:
            # Visualization I/O must not stop control. Explicitly publish no
            # reference rather than retain an apparently healthy old tree.
            self.entries.pop(robot_id, None)
            source=planner if isinstance(planner,dict) else {}
            result={k:v for k,v in source.items() if k not in ("tree_nodes","tree_edges","tree_ref")}
            result.update(tree_nodes_count=0,tree_edges_count=0,tree_ref=None,
                          tree_snapshot_error=type(error).__name__+": "+str(error))
            return result

    def _compact(self, robot_id, planner):
        if not isinstance(robot_id, str) or not _ID.fullmatch(robot_id):
            raise ValueError("invalid tree robot ID")
        source = planner if isinstance(planner, dict) else {}
        compact = {k: v for k, v in source.items() if k not in ("tree_nodes", "tree_edges", "tree_ref")}
        compact.update(tree_nodes_count=0, tree_edges_count=0, tree_ref=None)
        nodes, edges = source.get("tree_nodes"), source.get("tree_edges")
        if (not source or source.get("ok") is False or source.get("tree_exported") is False
                or not isinstance(nodes, (list, tuple)) or not nodes
                or not isinstance(edges, (list, tuple))):
            self.entries.pop(robot_id, None)
            self._cleanup()
            return compact
        session, instance = source.get("session_id"), source.get("tree_instance_id")
        if (not isinstance(session, str) or not session or isinstance(instance, bool)
                or not isinstance(instance, (str, int)) or source.get("robot_id", robot_id) != robot_id):
            compact["tree_snapshot_error"] = "invalid native tree identity"
            self.entries.pop(robot_id, None)
            return compact
        previous = self.entries.get(robot_id)
        identity = (session, instance)
        if previous is not None and previous[0] is source and previous[1] == identity:
            ref = previous[2]
        else:
            geometry = _encode({"tree_nodes": nodes, "tree_edges": edges})
            geometry_sha = hashlib.sha256(geometry).hexdigest()
            header = dict(schema=SCHEMA, epoch=self.epoch, robot_id=robot_id,
                          session_id=session, tree_instance_id=instance, geometry_sha256=geometry_sha)
            if (previous is not None and previous[1] == identity
                    and previous[2]["geometry_sha256"] == geometry_sha):
                ref = previous[2]
            else:
                # Assemble the envelope without encoding large arrays twice.
                payload = _encode(header)[:-1] + b"," + geometry[1:]
                digest = hashlib.sha256(payload).hexdigest()
                relative = f"shared/tree_snapshots/{robot_id}/tree-{digest}.json"
                path = self.root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                if not path.exists():
                    fd, temporary = tempfile.mkstemp(prefix=".tree-", suffix=".tmp", dir=path.parent)
                    try:
                        with os.fdopen(fd, "wb") as stream:
                            stream.write(payload)
                            stream.flush()
                        os.replace(temporary, path)
                    finally:
                        if os.path.exists(temporary):
                            os.unlink(temporary)
                ref = dict(header, sha256=digest, path=relative)
            self.entries[robot_id] = (source, identity, ref)
        self.entries.move_to_end(robot_id)
        while len(self.entries) > self.max_cached_robots:
            self.entries.popitem(last=False)
        compact.update(tree_nodes_count=len(nodes), tree_edges_count=len(edges), tree_ref=dict(ref))
        self._cleanup()
        return compact

    def _cleanup(self):
        now = self.clock()
        if now - self._last_cleanup < 1.:
            return
        self._last_cleanup = now
        protected = {entry[2]["path"] for entry in self.entries.values()}
        if not self.folder.is_dir():
            return
        # File times reflect publication. Reused current geometry is protected
        # independently of age; only superseded immutable files are removed.
        for folder in self.folder.iterdir():
            if not folder.is_dir() or folder.is_symlink():
                continue
            versions = sorted(folder.glob("tree-*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
            for path in versions[self.retain_versions:]:
                try:
                    if (path.relative_to(self.root).as_posix() not in protected
                            and now - path.stat().st_mtime >= self.grace_s):
                        path.unlink()
                except FileNotFoundError:
                    pass


class TreeSnapshotReader:
    """Tree-timer-only hydration and bounded verified version cache.

    A missing file can retain the last *whole* version for at most three seconds
    in the same epoch/robot/session/tree instance. Hash/identity violations,
    explicit planner failure and stale runtime state are hidden immediately.
    Invalid versions are never cached, and each tree tick retries the reference.
    """
    def __init__(self, *, health_s=3., max_cached_robots=32, max_bytes=64 * 1024 * 1024):
        self.health_s = health_s
        self.max_cached_robots = max_cached_robots
        self.max_bytes = max_bytes
        self.entries = OrderedDict()
        self._pending = {}
        self._run_identity = None

    def _load(self, root, ref, expected):
        if not isinstance(ref, dict) or any(ref.get(k) != v for k, v in expected.items()):
            raise ValueError("tree snapshot reference identity mismatch")
        for field in ("sha256", "geometry_sha256"):
            if not isinstance(ref.get(field), str) or not _SHA.fullmatch(ref[field]):
                raise ValueError("invalid tree snapshot digest")
        relative = ref.get("path")
        if not isinstance(relative, str):
            raise ValueError("invalid tree snapshot path")
        pure = PurePosixPath(relative)
        prefix = ("shared", "tree_snapshots", expected["robot_id"])
        if (pure.is_absolute() or ".." in pure.parts or pure.parts[:3] != prefix
                or len(pure.parts) != 4 or pure.name != f"tree-{ref['sha256']}.json"):
            raise ValueError("tree snapshot path must be scoped to the run and robot")
        path = (root / relative).resolve()
        if not path.is_relative_to(root.resolve()):
            raise ValueError("tree snapshot path escapes run")
        signature = reference_signature(ref)
        entry = self.entries.get(expected["robot_id"])
        if entry is not None and entry[0] == signature:
            self.entries.move_to_end(expected["robot_id"])
            return entry[1]
        with path.open("rb") as stream:
            if os.fstat(stream.fileno()).st_size > self.max_bytes:
                raise ValueError("tree snapshot exceeds size budget")
            payload = stream.read(self.max_bytes + 1)
        if len(payload) > self.max_bytes or hashlib.sha256(payload).hexdigest() != ref["sha256"]:
            raise ValueError("tree snapshot sha256 mismatch")
        document = json.loads(payload)
        if not isinstance(document, dict) or any(document.get(k) != v for k, v in expected.items()):
            raise ValueError("tree snapshot document identity mismatch")
        if (not isinstance(document.get("tree_nodes"), list)
                or not isinstance(document.get("tree_edges"), list)
                or document.get("geometry_sha256") != ref["geometry_sha256"]
                or geometry_digest(document) != ref["geometry_sha256"]):
            raise ValueError("tree snapshot geometry mismatch")
        self.entries[expected["robot_id"]] = (signature, document, dict(ref))
        self.entries.move_to_end(expected["robot_id"])
        while len(self.entries) > self.max_cached_robots:
            removed, _ = self.entries.popitem(last=False)
            self._pending.pop(removed, None)
        return document

    def hydrate(self, state, *, now_wall=None, now_monotonic=None):
        if state is None:
            self.entries.clear(); self._pending.clear(); self._run_identity = None
            return None, []
        wall = time.time() if now_wall is None else now_wall
        mono = time.monotonic() if now_monotonic is None else now_monotonic
        root = Path(state.get("_view_run_root", "."))
        run_identity = (str(root), state.get("epoch"))
        if run_identity != self._run_identity:
            self.entries.clear(); self._pending.clear(); self._run_identity = run_identity
        result = dict(state, robots=[])
        issues = []
        active = set()
        age = max(state.get("_view_state_age_s", 0.), max(0., wall-state.get("wall_time", wall)))
        healthy = age <= self.health_s and not state.get("fatal") and state.get("running") is not False
        for robot in state.get("robots", []):
            item = dict(robot)
            planner = dict(robot.get("planner") or {})
            rid = robot.get("robot_id")
            if not isinstance(rid, str) or not _ID.fullmatch(rid):
                item["planner"] = {}; result["robots"].append(item); continue
            active.add(rid)
            valid = (healthy and not robot.get("planner_error") and planner.get("ok") is not False
                     and planner.get("tree_exported") is not False
                     and wall - planner.get("received_wall", wall) <= self.health_s)
            if not valid:
                self.entries.pop(rid, None); self._pending.pop(rid, None)
                planner.update(tree_nodes=[], tree_edges=[], tree_ref=None, _tree_display_identity=None)
            elif "tree_ref" in planner:
                ref = planner.get("tree_ref")
                planner.update(tree_nodes=[], tree_edges=[], _tree_display_identity=None)
                if ref is None:
                    self.entries.pop(rid, None); self._pending.pop(rid, None)
                else:
                    expected = dict(schema=SCHEMA, epoch=state.get("epoch"), robot_id=rid,
                                    session_id=planner.get("session_id"), tree_instance_id=planner.get("tree_instance_id"))
                    try:
                        document = self._load(root, ref, expected)
                        self._pending.pop(rid, None)
                    except OSError:
                        # A race must not combine the new nodes with old edges.
                        # Retain an entire verified version only in this identity.
                        first = self._pending.setdefault(rid, mono)
                        previous = self.entries.get(rid)
                        if (previous is not None and mono-first <= self.health_s
                                and all(previous[1].get(k) == v for k, v in expected.items())):
                            document, ref = previous[1], previous[2]
                        else:
                            document = None
                        issues.append(f"{rid}: tree file unavailable; retrying")
                    except (ValueError, TypeError, KeyError):
                        self.entries.pop(rid, None); self._pending.pop(rid, None)
                        document = None
                        issues.append(f"{rid}: invalid tree snapshot hidden")
                    if document is not None:
                        planner.update(tree_nodes=document["tree_nodes"], tree_edges=document["tree_edges"],
                                       tree_ref=ref, _tree_display_identity=reference_signature(ref))
                    else:
                        planner["tree_ref"] = None
            item["planner"] = planner
            result["robots"].append(item)
        for rid in (set(self.entries) | set(self._pending)) - active:
            self.entries.pop(rid, None); self._pending.pop(rid, None)
        while len(self._pending) > self.max_cached_robots:
            self._pending.pop(next(iter(self._pending)))
        return result, issues
