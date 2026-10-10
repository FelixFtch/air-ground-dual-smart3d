"""Bounded immutable, same-acquisition RGB-D transport (no simulator dependency).

All pixels are retained. XYZ uses optical right/down/forward axes and metres;
the two capture transforms use proper right-handed map rotations. Readers take
one bounded byte snapshot before decoding, so ring collection cannot change an
already opened bundle. A manifest lost to ring collection is safe to retry.
"""
from __future__ import annotations

import ast
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import time
import uuid
import zipfile

import numpy as np

RGBD_SCHEMA = "air_ground.registered_rgbd/v1"
MANIFEST_SCHEMA = "air_ground.registered_rgbd.latest/v1"
WIDTH, HEIGHT = 640, 360
MAX_META_BYTES = 16384
MAX_BYTES = 4 * 1024 * 1024
MIN_DEPTH, MAX_DEPTH = .15, 30.
MAX_RETAIN = 8
_ID = re.compile(r"[A-Za-z0-9_.:-]{1,160}\Z")
_HASH = re.compile(r"[0-9a-f]{64}\Z")
_FILE = re.compile(r"[0-9]+-[0-9a-f]{64}\.npz\Z")


def _json(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")).encode("utf-8")


def _sha(value):
    return hashlib.sha256(value).hexdigest()


def _number(value, label, minimum=0.):
    if type(value) not in (int, float) or not math.isfinite(value) or value < minimum:
        raise ValueError("invalid_" + label)


def validate_transform(value, label):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-5, rtol=0)):
        raise ValueError("invalid_" + label)
    return matrix


def validate_cooperative_bodies(meta):
    """Validate optional capture-time known robot geometry; no human truth input."""
    if "cooperative_bodies" not in meta:
        return []
    bodies = meta["cooperative_bodies"]
    if not isinstance(bodies, list) or not 1 <= len(bodies) <= 16:
        raise ValueError("invalid_cooperative_bodies")
    frame, stamp = meta.get("frame_seq"), meta.get("capture_sim_time")
    if type(frame) is not int or not 0 <= frame < 2**63:
        raise ValueError("invalid_cooperative_capture_frame")
    _number(stamp, "cooperative_capture_time")
    seen = set()
    for body in bodies:
        if not isinstance(body, dict):
            raise ValueError("invalid_cooperative_body")
        rid = body.get("robot_id")
        if not isinstance(rid, str) or not _ID.fullmatch(rid) or rid in seen:
            raise ValueError("invalid_cooperative_robot_id")
        seen.add(rid)
        if (type(body.get("frame_seq")) is not int or body["frame_seq"] != frame
                or type(body.get("capture_sim_time")) not in (int, float)
                or not math.isfinite(body["capture_sim_time"])
                or body["capture_sim_time"] != stamp):
            raise ValueError("cooperative_capture_source_mismatch")
        transform = validate_transform(body.get("body_to_map"), "cooperative_body_to_map")
        for name, positive, limit in (("half_extents_xyz", True, 5.), ("body_offset_xyz", False, 10.)):
            vector = body.get(name)
            if (not isinstance(vector, (list, tuple)) or len(vector) != 3
                    or any(type(v) not in (int, float) or not math.isfinite(v)
                           or abs(v) > limit or (positive and v <= 0) for v in vector)):
                raise ValueError("invalid_cooperative_" + name)
        if rid == meta.get("robot_id"):
            own = validate_transform(meta.get("body_to_map"), "body_to_map")
            if not np.allclose(transform, own, atol=1e-5, rtol=0):
                raise ValueError("cooperative_capture_body_pose_mismatch")
    if meta.get("robot_id") not in seen:
        raise ValueError("cooperative_source_robot_missing")
    return bodies


def validate_metadata(meta, expected=None):
    if not isinstance(meta, dict) or meta.get("schema") != RGBD_SCHEMA:
        raise ValueError("invalid_capture_schema")
    for field in ("scene_id", "scene_epoch", "robot_id", "camera_id", "calibration_id"):
        if not isinstance(meta.get(field), str) or not _ID.fullmatch(meta[field]):
            raise ValueError("invalid_" + field)
    if meta["camera_id"] != "front_rgbd" or (meta.get("width"), meta.get("height")) != (WIDTH, HEIGHT):
        raise ValueError("unexpected_registered_camera_layout")
    if type(meta.get("frame_seq")) is not int or not 0 <= meta["frame_seq"] < 2**63:
        raise ValueError("invalid_frame_seq")
    for field in ("capture_sim_time", "receive_wall_time"):
        _number(meta.get(field), field)
    if meta.get("pixel_center_offset") != .5 or meta.get("depth_range_m") != [MIN_DEPTH, MAX_DEPTH]:
        raise ValueError("invalid_pixel_or_depth_convention")
    if meta.get("color_encoding") != "RGB8" or meta.get("xyz_axes") != "right_down_forward":
        raise ValueError("invalid_pixel_encoding")
    k = np.asarray(meta.get("K"), dtype=float)
    if (k.shape != (3, 3) or not np.isfinite(k).all() or not .1 < k[0, 0] < 100000
            or not np.allclose(k, [[k[0, 0], 0, WIDTH/2], [0, k[0, 0], HEIGHT/2], [0, 0, 1]], atol=1e-8, rtol=0)):
        raise ValueError("invalid_camera_intrinsics")
    for field in ("camera_to_map", "body_to_map"):
        validate_transform(meta.get(field), field)
    validate_cooperative_bodies(meta)
    # Explicit source stamps prevent a producer from accidentally labelling a
    # nearest-frame RGB/depth pair as a registered acquisition.
    for kind in ("rgb", "depth"):
        if (meta.get(kind + "_frame_seq") != meta["frame_seq"]
                or meta.get(kind + "_capture_sim_time") != meta["capture_sim_time"]):
            raise ValueError("capture_source_frame_mismatch")
    for field in ("frame_token", "rgb_sha256", "xyz_sha256"):
        if not isinstance(meta.get(field), str) or not _HASH.fullmatch(meta[field]):
            raise ValueError("invalid_" + field)
    if expected:
        for key, value in expected.items():
            if meta.get(key) != value:
                raise ValueError("capture_identity_mismatch:" + key)


def _validate_arrays(rgb, xyz, meta):
    if (rgb.shape != (HEIGHT, WIDTH, 3) or rgb.dtype != np.uint8
            or xyz.shape != (HEIGHT, WIDTH, 3) or xyz.dtype != np.dtype("<f4")):
        raise ValueError("invalid_registered_rgbd_layout")
    good = np.isfinite(xyz).all(axis=2)
    invalid = np.isnan(xyz).all(axis=2)
    if not (good | invalid).all():
        raise ValueError("invalid_depth_components")
    z = xyz[..., 2]
    if np.any(good & ((z < MIN_DEPTH) | (z > MAX_DEPTH))):
        raise ValueError("depth_outside_valid_range")
    # A shape check alone would accept a transposed or half-pixel-shifted depth
    # raster. Check the organized optical ray geometry as well.
    f = float(meta["K"][0][0])
    u = (np.arange(WIDTH, dtype=np.float64) + .5 - WIDTH/2) / f
    v = (np.arange(HEIGHT, dtype=np.float64) + .5 - HEIGHT/2) / f
    if (not np.allclose(xyz[..., 0][good], (z * u[None, :])[good], atol=1e-5, rtol=2e-6)
            or not np.allclose(xyz[..., 1][good], (z * v[:, None])[good], atol=1e-5, rtol=2e-6)):
        raise ValueError("unregistered_optical_raster")


def encode_capture(rgb, xyz_optical, meta):
    """Create a transport payload; metadata hashes/token are computed here."""
    rgb = np.ascontiguousarray(rgb)
    xyz = np.ascontiguousarray(xyz_optical)
    meta = dict(meta)
    meta.update(schema=RGBD_SCHEMA, color_encoding="RGB8", xyz_axes="right_down_forward",
                pixel_center_offset=.5, depth_range_m=[MIN_DEPTH, MAX_DEPTH],
                rgb_sha256=_sha(rgb.tobytes()), xyz_sha256=_sha(xyz.tobytes()))
    meta.pop("frame_token", None)
    meta["frame_token"] = _sha(_json(meta))
    validate_metadata(meta)
    _validate_arrays(rgb, xyz, meta)
    encoded = _json(meta)
    if len(encoded) > MAX_META_BYTES:
        raise ValueError("capture_metadata_too_large")
    stream = io.BytesIO()
    np.savez(stream, rgb=rgb, xyz=xyz, metadata=np.frombuffer(encoded, dtype=np.uint8))
    payload = stream.getvalue()
    if len(payload) > MAX_BYTES:
        raise ValueError("capture_payload_too_large")
    return payload


def _validate_archive(payload):
    limits = {"rgb.npy": HEIGHT*WIDTH*3 + 512, "xyz.npy": HEIGHT*WIDTH*3*4 + 512,
              "metadata.npy": MAX_META_BYTES + 512}
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        entries = archive.infolist()
        if len(entries) != 3 or {e.filename for e in entries} != set(limits):
            raise ValueError("unexpected_capture_members")
        for entry in entries:
            if (entry.compress_type != zipfile.ZIP_STORED or entry.flag_bits & 1
                    or entry.file_size > limits[entry.filename] or entry.compress_size != entry.file_size):
                raise ValueError("invalid_capture_member_size")
            with archive.open(entry) as member:
                version = np.lib.format.read_magic(member)
                if version not in ((1, 0), (2, 0)):
                    raise ValueError("unsupported_capture_array_version")
                header_size = int.from_bytes(member.read(2 if version == (1, 0) else 4), "little")
                if not 1 <= header_size <= 256:
                    raise ValueError("capture_array_header_too_large")
                header = ast.literal_eval(member.read(header_size).decode("latin1"))
                if not isinstance(header, dict) or set(header) != {"shape", "descr", "fortran_order"}:
                    raise ValueError("invalid_capture_array_header")
                shape, dtype = header["shape"], np.dtype(header["descr"])
                if entry.filename == "rgb.npy":
                    valid = shape == (HEIGHT, WIDTH, 3) and dtype == np.dtype("u1")
                elif entry.filename == "xyz.npy":
                    valid = shape == (HEIGHT, WIDTH, 3) and dtype == np.dtype("<f4")
                else:
                    valid = (isinstance(shape, tuple) and len(shape) == 1 and type(shape[0]) is int
                             and 0 < shape[0] <= MAX_META_BYTES and dtype == np.dtype("u1"))
                if (not valid or header["fortran_order"] is not False or dtype.hasobject
                        or math.prod(shape)*dtype.itemsize + member.tell() != entry.file_size):
                    raise ValueError("invalid_registered_rgbd_layout")


def decode_capture(payload, *, expected=None):
    """Validate a bounded byte snapshot, identity, hashes and registered geometry."""
    if not isinstance(payload, bytes) or not 1 <= len(payload) <= MAX_BYTES:
        raise ValueError("capture_payload_too_large")
    try:
        _validate_archive(payload)
        with np.load(io.BytesIO(payload), allow_pickle=False) as arrays:
            rgb, xyz = arrays["rgb"], arrays["xyz"]
            meta = json.loads(arrays["metadata"].tobytes())
        validate_metadata(meta, expected)
        token_fields = {key: value for key, value in meta.items() if key != "frame_token"}
        if _sha(_json(token_fields)) != meta["frame_token"]:
            raise ValueError("capture_frame_token_mismatch")
        if _sha(rgb.tobytes()) != meta["rgb_sha256"] or _sha(xyz.tobytes()) != meta["xyz_sha256"]:
            raise ValueError("capture_array_hash_mismatch")
        _validate_arrays(rgb, xyz, meta)
    except (zipfile.BadZipFile, KeyError, EOFError, UnicodeDecodeError, SyntaxError, TypeError) as exc:
        raise ValueError("invalid_capture_payload") from exc
    return {"rgb": rgb, "xyz_optical": xyz, "meta": meta}


def _read_bytes(path, limit):
    # Open exactly once and read fully before ZIP/NumPy sees a path. POSIX unlink
    # of this descriptor by ring collection remains safe until the read ends.
    with Path(path).open("rb") as source:
        if os.fstat(source.fileno()).st_size > limit:
            raise ValueError("capture_file_too_large")
        payload = source.read(limit + 1)
    if len(payload) > limit:
        raise ValueError("capture_file_too_large")
    return payload


def load_capture(path, *, expected=None):
    """Read an immutable NPZ or latest.json; enforce caller identity if supplied."""
    path = Path(path)
    manifest = None
    if path.suffix == ".json":
        manifest = json.loads(_read_bytes(path, MAX_META_BYTES))
        if not isinstance(manifest, dict):
            raise ValueError("invalid_capture_manifest")
        filename = manifest.get("file")
        if (manifest.get("schema") != MANIFEST_SCHEMA or not isinstance(filename, str)
                or not _FILE.fullmatch(filename) or manifest.get("path") != filename):
            raise ValueError("invalid_capture_manifest")
        path = path.parent / filename
    payload = _read_bytes(path, MAX_BYTES)
    if manifest and _sha(payload) != manifest.get("payload_sha256"):
        raise ValueError("capture_payload_hash_mismatch")
    result = decode_capture(payload, expected=expected)
    meta = result["meta"]
    if path.name != f'{meta["frame_seq"]}-{meta["frame_token"]}.npz':
        raise ValueError("capture_filename_identity_mismatch")
    if manifest:
        for field in ("scene_id", "scene_epoch", "robot_id", "camera_id", "calibration_id",
                      "frame_seq", "frame_token", "capture_sim_time"):
            if manifest.get(field) != meta[field]:
                raise ValueError("capture_manifest_identity_mismatch:" + field)
    # Reader-added provenance is outside the sealed metadata/token. Consumers
    # retain this exact immutable source path and digest with derived outputs.
    meta["bundle_path"] = str(path.resolve())
    meta["bundle_sha256"] = _sha(payload)
    meta["payload_sha256"] = meta["bundle_sha256"]
    return result


def load_latest(folder, *, expected=None):
    return load_capture(Path(folder) / "latest.json", expected=expected)


def _atomic_write(path, payload):
    path = Path(path)
    temporary = path.with_name("." + path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("xb") as output:
            output.write(payload)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def publish_capture(folder, rgb, xyz_optical, meta, *, retain=MAX_RETAIN):
    """Publish one validated bundle, then its manifest; retain at most eight."""
    if type(retain) is not int or not 1 <= retain <= MAX_RETAIN:
        raise ValueError("invalid_capture_retention")
    payload = encode_capture(rgb, xyz_optical, meta)
    # Decode only metadata after our encoder has validated both full arrays.
    with np.load(io.BytesIO(payload), allow_pickle=False) as arrays:
        metadata = json.loads(arrays["metadata"].tobytes())
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    filename = f'{metadata["frame_seq"]}-{metadata["frame_token"]}.npz'
    target = folder / filename
    if target.exists():
        if _sha(_read_bytes(target, MAX_BYTES)) != _sha(payload):
            raise ValueError("immutable_capture_collision")
    else:
        _atomic_write(target, payload)
    manifest = {key: metadata[key] for key in ("scene_id", "scene_epoch", "robot_id", "camera_id",
                "calibration_id", "frame_seq", "frame_token", "capture_sim_time", "receive_wall_time")}
    manifest.update(schema=MANIFEST_SCHEMA, file=filename, path=filename,
                    payload_sha256=_sha(payload), payload_bytes=len(payload), published_wall_time=time.time())
    _atomic_write(folder / "latest.json", _json(manifest))
    previous = sorted((p for p in folder.glob("*.npz") if p != target and _FILE.fullmatch(p.name)),
                      key=lambda p: (p.stat().st_mtime_ns, p.name), reverse=True)
    for obsolete in previous[retain-1:]:
        obsolete.unlink(missing_ok=True)
    return manifest
