"""Independent read-only subscriber for existing front RGB/depth sensor actors.

Run with the CARLA-compatible Python: ``python -m perception.capture``. This
process never creates actors, ticks a world, requests latest robot poses, or
changes mapper subscriptions. It owns only its own client subscriptions.
"""
from __future__ import annotations

import argparse
import copy
from collections import OrderedDict
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import numpy as np

from .registered_rgbd import (HEIGHT, WIDTH, MAX_DEPTH, MIN_DEPTH, RGBD_SCHEMA,
                              _atomic_write, publish_capture, validate_transform, validate_cooperative_bodies)

ROOT = Path(__file__).resolve().parents[1]
MAP_BASIS = np.diag([1., -1., 1.])
OPTICAL_TO_SENSOR = np.array([[0., 0., 1.], [1., 0., 0.], [0., -1., 0.]])
PAIR_HISTORY = 20


def camera_intrinsics(width, height, fov):
    if (width, height) != (WIDTH, HEIGHT) or not 1 < float(fov) < 179:
        raise ValueError("unsupported_front_rgbd_calibration")
    focal = width / (2 * math.tan(math.radians(float(fov))/2))
    return np.array([[focal, 0., width/2], [0., focal, height/2], [0., 0., 1.]])


def rigid_camera_extrinsic(camera):
    """CARLA's local sensor-to-body matrix, without importing/connecting CARLA."""
    pitch, yaw, roll = [math.radians(float(camera.get(key + "_degrees", 0.)))
                        for key in ("pitch", "yaw", "roll")]
    cp, sp, cy, sy, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    transform = np.eye(4)
    transform[:3, :3] = [[cp*cy, cy*sp*sr-sy*cr, -cy*sp*cr-sy*sr],
                         [cp*sy, sy*sp*sr+cy*cr, -sy*sp*cr+cy*sr],
                         [sp, -cp*sr, cp*cr]]
    transform[:3, 3] = camera["mount_m"]
    return validate_transform(transform, "camera_extrinsic")


def capture_transforms(sensor_to_world, sensor_to_body, origin):
    """Derive both poses from the same capture measurement, never latest pose."""
    sensor = validate_transform(sensor_to_world, "measurement_transform")
    local = validate_transform(sensor_to_body, "camera_extrinsic")
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("invalid_world_origin")
    body = sensor @ np.linalg.inv(local)
    camera_to_map, body_to_map = np.eye(4), np.eye(4)
    camera_to_map[:3, :3] = MAP_BASIS @ sensor[:3, :3] @ OPTICAL_TO_SENSOR
    camera_to_map[:3, 3] = MAP_BASIS @ (sensor[:3, 3] - origin)
    # CARLA body is forward/right/up; reported body is forward/left/up.
    # Changing both bases removes the reflection, preserving proper rotation.
    body_to_map[:3, :3] = MAP_BASIS @ body[:3, :3] @ MAP_BASIS
    body_to_map[:3, 3] = MAP_BASIS @ (body[:3, 3] - origin)
    validate_transform(camera_to_map, "camera_to_map")
    validate_transform(body_to_map, "body_to_map")
    return camera_to_map, body_to_map


def registered_pixels(rgb, depth, intrinsics):
    for measurement in (rgb, depth):
        if (measurement.width, measurement.height) != (WIDTH, HEIGHT):
            raise ValueError("unexpected_front_camera_dimensions")
        if len(measurement.raw_data) != HEIGHT*WIDTH*4:
            raise ValueError("invalid_camera_byte_count")
    rgba = np.frombuffer(rgb.raw_data, np.uint8).reshape(HEIGHT, WIDTH, 4)
    color = rgba[..., 2::-1].copy()
    encoded = np.frombuffer(depth.raw_data, np.uint8).reshape(HEIGHT, WIDTH, 4).astype(np.float64)
    z = (encoded[..., 2] + encoded[..., 1]*256 + encoded[..., 0]*65536) / 16777215 * 1000
    u, v = np.meshgrid(np.arange(WIDTH) + .5, np.arange(HEIGHT) + .5)
    xyz = np.stack(((u-intrinsics[0, 2])*z/intrinsics[0, 0],
                    (v-intrinsics[1, 2])*z/intrinsics[1, 1], z), axis=-1).astype(np.float32)
    xyz[~np.isfinite(z) | (z < MIN_DEPTH) | (z > MAX_DEPTH)] = np.nan
    return color, xyz


def calibration_id(sensor_profile, scene):
    camera = sensor_profile["camera"]
    fields = {"profile_id": sensor_profile["id"], "width": camera["width"], "height": camera["height"],
              "K": camera_intrinsics(camera["width"], camera["height"], camera["fov"]).tolist(),
              "camera_to_body_carla": rigid_camera_extrinsic(camera).tolist(),
              "origin": scene["carla_world_origin_m"], "pixel_center_offset": .5,
              "optical_axes": "right_down_forward", "body_axes": "forward_left_up"}
    digest = hashlib.sha256(json.dumps(fields, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]
    return sensor_profile["id"] + "-front_rgbd-" + digest


def build_capture(rgb, depth, *, scene, scene_epoch, robot, receive_wall_time, cooperative_bodies=None):
    """Decode one exact pair. Reject any frame, timestamp or pose mismatch."""
    if int(rgb.frame) != int(depth.frame) or float(rgb.timestamp) != float(depth.timestamp):
        raise ValueError("rgb_depth_capture_mismatch")
    rgb_pose = np.asarray(rgb.transform.get_matrix(), dtype=float)
    depth_pose = np.asarray(depth.transform.get_matrix(), dtype=float)
    if not np.allclose(rgb_pose, depth_pose, atol=1e-5, rtol=0):
        raise ValueError("rgb_depth_capture_pose_mismatch")
    profile = robot["sensors_config"]
    camera = profile["camera"]
    k = camera_intrinsics(camera["width"], camera["height"], camera["fov"])
    camera_to_map, body_to_map = capture_transforms(depth_pose, rigid_camera_extrinsic(camera), scene["carla_world_origin_m"])
    rgb_pixels, xyz = registered_pixels(rgb, depth, k)
    meta = dict(schema=RGBD_SCHEMA, scene_id=scene["scene_id"], scene_epoch=scene_epoch,
                content_version=str(scene["content_version"]), frame_id="map", robot_id=robot["robot_id"],
                camera_id="front_rgbd", calibration_id=calibration_id(profile, scene),
                frame_seq=int(depth.frame), capture_sim_time=float(depth.timestamp),
                rgb_frame_seq=int(rgb.frame), depth_frame_seq=int(depth.frame),
                rgb_capture_sim_time=float(rgb.timestamp), depth_capture_sim_time=float(depth.timestamp),
                receive_wall_time=float(receive_wall_time), width=WIDTH, height=HEIGHT, K=k.tolist(),
                camera_to_map=camera_to_map.tolist(), body_to_map=body_to_map.tolist(),
                body_axes="forward_left_up", map_axes="x=carla.x,y=-carla.y,z=carla.z-.22",
                pose_basis="measurement_transform_and_rigid_camera_extrinsic")
    if cooperative_bodies is not None:
        meta["cooperative_bodies"] = copy.deepcopy(cooperative_bodies)
        validate_cooperative_bodies(meta)
    return {"rgb": rgb_pixels, "xyz_optical": xyz, "meta": meta}


class ExactPairBuffer:
    """Bounded per-source history and latest complete pair, independent of mapper."""
    def __init__(self, limit=PAIR_HISTORY):
        if not 1 <= limit <= PAIR_HISTORY:
            raise ValueError("invalid_pair_history_limit")
        self.limit = limit
        self.rgb, self.depth = OrderedDict(), OrderedDict()
        self.last_taken = -1
        self.lock = threading.Lock()

    def add(self, kind, measurement, wall=None):
        if kind not in ("rgb", "depth"):
            raise ValueError("unexpected_pair_sensor")
        with self.lock:
            if int(measurement.frame) <= self.last_taken:
                return
            history = getattr(self, kind)
            history[int(measurement.frame)] = (measurement, time.time() if wall is None else wall)
            while len(history) > self.limit:
                history.popitem(last=False)

    def take_latest(self, allowed_frames=None):
        with self.lock:
            common = self.rgb.keys() & self.depth.keys()
            if allowed_frames is not None:
                common &= allowed_frames
            if not common:
                return None
            frame = max(common)
            rgb, rgb_wall = self.rgb[frame]
            depth, depth_wall = self.depth[frame]
            self.last_taken = frame
            for history in (self.rgb, self.depth):
                for old in [key for key in history if key <= frame]:
                    del history[old]
            return rgb, depth, max(rgb_wall, depth_wall)


def active_run(root, now=None):
    """Read one consistent current-run identity; fail closed on stale/stopped run."""
    home = Path(os.environ["AIR_GROUND_RUNTIME"]) if os.environ.get("AIR_GROUND_RUNTIME") else Path(root) / "runtime"
    pointer = json.loads((home / "latest_run.json").read_text())
    run = Path(pointer["run_root"]).resolve()
    if run.parent != (home / "runs").resolve() or run.name != pointer["epoch"]:
        raise ValueError("run_pointer_outside_project")
    state = json.loads((run / "state.json").read_text())
    now = time.time() if now is None else now
    if (state.get("running") is not True or state.get("epoch") != pointer["epoch"]
            or not -.5 <= now-float(state["wall_time"]) <= 4.):
        raise ValueError("run_stopped_or_stale")
    profile = json.loads((run / "resolved_profiles.json").read_text())
    scene = profile["scene_config"]
    expected_port = int(os.environ["AIR_GROUND_CARLA_PORT"]) if os.environ.get("AIR_GROUND_CARLA_PORT") else 2400
    if (scene["carla_port"] != expected_port or scene["carla_host"] != "127.0.0.1"
            or scene["scene_id"] != state["scene_id"] or scene["frame_id"] != "map"
            or scene["carla_world_origin_m"] != [0, 0, .22]):
        raise ValueError("unexpected_independent_scene")
    return run, pointer["epoch"], profile, state


class CaptureService:
    def __init__(self, root=ROOT, max_hz=3., client=None):
        if not 0 < max_hz <= 3:
            raise ValueError("capture_rate_must_be_at_most_3_hz")
        self.root, self.period = Path(root), 1./max_hz
        self.stop = threading.Event()
        self.client = client
        self.subscriptions, self.buffers, self.last_publish = {}, {}, {}
        self.identity, self.run = None, None
        self.profile = None
        self.generation = 0
        self.discovered = 0.
        self.last_sim_time = None
        self.last_status = 0.
        self.last_clock_publish = 0.
        self.stats = {}
        self.refused_epochs = OrderedDict()
        self.body_history = OrderedDict()
        self.body_history_lock = threading.Lock()

    def detach(self):
        with self.body_history_lock:
            self.generation += 1
            self.body_history.clear()
        for sensor in self.subscriptions.values():
            try:
                sensor.stop()
            except RuntimeError:
                pass
        self.subscriptions.clear()
        self.buffers.clear()
        self.last_publish.clear()
        self.identity = None
        self.last_sim_time = None

    def _cooperative_profiles(self):
        robots = self.profile["robots"] if self.profile else []
        if not any("robot_config" in robot for robot in robots):
            return {}  # Legacy transport-only mocks/profiles have no body geometry.
        if not all("robot_config" in robot for robot in robots):
            raise ValueError("incomplete_cooperative_robot_profiles")
        return {robot["robot_id"]: robot for robot in robots}

    def _cooperative_frames(self):
        required = set(self._cooperative_profiles())
        if not required:
            return None
        with self.body_history_lock:
            return {frame for frame, bodies in self.body_history.items()
                    if set(bodies) == required
                    and len({body["capture_sim_time"] for body in bodies.values()}) == 1}

    def _cooperative_bodies(self, frame, stamp):
        required = set(self._cooperative_profiles())
        if not required:
            return None
        with self.body_history_lock:
            bodies = self.body_history.get(frame, {})
            if set(bodies) != required or any(body["capture_sim_time"] != stamp for body in bodies.values()):
                raise ValueError("cooperative_capture_bodies_unavailable")
            return copy.deepcopy([bodies[rid] for rid in sorted(bodies)])

    def receive(self, generation, robot_id, kind, measurement):
        buffer = self.buffers.get(robot_id)
        if generation != self.generation or self.stop.is_set() or buffer is None:
            return
        if kind == "depth":
            robots = self._cooperative_profiles()
            if robots:
                robot = robots[robot_id]
                camera = robot["sensors_config"]["camera"]
                _, body = capture_transforms(measurement.transform.get_matrix(),
                    rigid_camera_extrinsic(camera), self.profile["scene_config"]["carla_world_origin_m"])
                cfg = robot["robot_config"]
                record = dict(robot_id=robot_id, frame_seq=int(measurement.frame),
                    capture_sim_time=float(measurement.timestamp), body_to_map=body.tolist(),
                    half_extents_xyz=list(cfg["body_half_extents_m"]),
                    body_offset_xyz=list(cfg["body_offset_m"]))
                validate_cooperative_bodies(dict(robot_id=robot_id, frame_seq=record["frame_seq"],
                    capture_sim_time=record["capture_sim_time"], body_to_map=record["body_to_map"],
                    cooperative_bodies=[record]))
                with self.body_history_lock:
                    if generation != self.generation:
                        return
                    self.body_history.setdefault(record["frame_seq"], {})[robot_id] = record
                    while len(self.body_history) > PAIR_HISTORY:
                        self.body_history.popitem(last=False)
        if generation == self.generation and not self.stop.is_set():
            buffer.add(kind, measurement)

    def discover(self, world, state):
        robots = {r["robot_id"]: r for r in self.profile["robots"]}
        actor_ids = {r["robot_id"]: r["actor_id"] for r in state["robots"]}
        candidates = {}
        for sensor in world.get_actors().filter("sensor.camera.*"):
            if sensor.type_id not in ("sensor.camera.rgb", "sensor.camera.depth"):
                continue
            parent = sensor.parent
            if parent is None:
                continue
            rid = parent.attributes.get("role_name", "").removeprefix("ag_")
            if rid not in robots or parent.id != actor_ids.get(rid):
                continue
            camera = robots[rid]["sensors_config"]["camera"]
            attrs = sensor.attributes
            if ((int(attrs.get("image_size_x", 0)), int(attrs.get("image_size_y", 0))) != (WIDTH, HEIGHT)
                    or abs(float(attrs.get("fov", 0))-float(camera["fov"])) > 1e-5):
                continue
            key = (rid, sensor.type_id.rsplit(".", 1)[1])
            candidates.setdefault(key, []).append(sensor)
        # Ambiguity fails closed; do not silently choose among duplicated cameras.
        eligible = {key: values[0] for key, values in candidates.items() if len(values) == 1}
        selected = {key: sensor for key, sensor in eligible.items()
                    if (key[0], "rgb") in eligible and (key[0], "depth") in eligible}
        desired_ids = {sensor.id for sensor in selected.values()}
        if any(sensor_id not in desired_ids for sensor_id in self.subscriptions):
            # Replacing one front actor invalidates all queued evidence. The run
            # identity remains constant but callback generation changes.
            identity, last_sim_time = self.identity, self.last_sim_time
            self.detach()
            self.identity, self.last_sim_time = identity, last_sim_time
        for (rid, kind), sensor in selected.items():
            if sensor.id in self.subscriptions:
                continue
            self.buffers.setdefault(rid, ExactPairBuffer())
            generation = self.generation
            sensor.listen(lambda m, gen=generation, robot_id=rid, stream=kind: self.receive(gen, robot_id, stream, m))
            self.subscriptions[sensor.id] = sensor

    def _status(self, running, error=None):
        if self.run is None:
            return
        value = {"schema": "air_ground.rgbd_capture_status/v1", "running": running,
                 "wall_time": time.time(), "scene_epoch": self.run.name, "max_hz": 1/self.period,
                 "subscribed_sensor_ids": sorted(self.subscriptions), "sources": self.stats, "error": error}
        _atomic_write(self.run / "perception_capture_status.json", json.dumps(value, allow_nan=False).encode())

    def step(self):
        run, epoch, profile, state = active_run(self.root)
        if epoch in self.refused_epochs:
            raise ValueError("capture_clock_rollback_requires_new_epoch")
        calibrations = tuple((r["robot_id"], calibration_id(r["sensors_config"], profile["scene_config"]))
                             for r in profile["robots"])
        identity = (str(run), epoch, state["world_id"], calibrations)
        if identity != self.identity:
            self.detach()
            self.identity, self.run, self.profile = identity, run, profile
            self.stats = {}
            self.discovered = 0.
        world = self.client.get_world()
        if world.id != state["world_id"]:
            raise ValueError("run_world_identity_mismatch")
        # Runtime telemetry can trail asynchronous image acquisition while its
        # mapper/planners are busy. Use the actual world clock for freshness,
        # never query latest actor poses to complete an older image.
        snapshot = world.get_snapshot()
        sim_time = float(snapshot.timestamp.elapsed_seconds)
        if self.last_sim_time is not None and sim_time < self.last_sim_time:
            # A clock reset without a fresh scene epoch must not continue the
            # same video session. Refuse this run until owner updates epoch.
            self.refused_epochs[epoch] = True
            while len(self.refused_epochs) > 8:
                self.refused_epochs.popitem(last=False)
            raise ValueError("capture_clock_rollback_requires_new_epoch")
        self.last_sim_time = sim_time
        now = time.monotonic()
        if now-self.last_clock_publish >= .1:
            clock = dict(schema="air_ground.sensor_clock/v1", scene_id=profile["scene_config"]["scene_id"],
                         scene_epoch=epoch, sim_time=sim_time, frame=int(snapshot.frame), wall_time=time.time())
            clock_folder = run / "shared"
            clock_folder.mkdir(parents=True, exist_ok=True)
            _atomic_write(clock_folder / "sensor_clock.json", json.dumps(clock, allow_nan=False).encode())
            self.last_clock_publish = now
        if now-self.discovered >= 1.:
            self.discover(world, state)
            self.discovered = now
        scene = self.profile["scene_config"]
        for robot in self.profile["robots"]:
            rid = robot["robot_id"]
            if now-self.last_publish.get(rid, -math.inf) < self.period or rid not in self.buffers:
                continue
            pair = self.buffers[rid].take_latest(allowed_frames=self._cooperative_frames())
            if pair is None:
                continue
            rgb, depth, wall = pair
            try:
                # Snapshot is sampled separately and can slightly trail a callback.
                if not -.5 <= sim_time-float(depth.timestamp) <= 1.5 or time.time()-wall > 1.5:
                    raise ValueError("stale_capture_pair")
                bodies = self._cooperative_bodies(int(depth.frame), float(depth.timestamp))
                capture = build_capture(rgb, depth, scene=scene, scene_epoch=epoch, robot=robot, receive_wall_time=wall,
                                        cooperative_bodies=bodies)
                manifest = publish_capture(run / rid / "rgbd", capture["rgb"], capture["xyz_optical"], capture["meta"])
                self.last_publish[rid] = time.monotonic()
                stats = self.stats.setdefault(rid, {"published": 0, "rejected": 0})
                stats.update(frame_seq=manifest["frame_seq"], capture_sim_time=manifest["capture_sim_time"],
                             frame_token=manifest["frame_token"], receive_wall_time=wall)
                stats["published"] += 1
            except ValueError as exc:
                stats = self.stats.setdefault(rid, {"published": 0, "rejected": 0})
                stats["rejected"] += 1
                stats["last_rejection"] = str(exc)
        if now-self.last_status > 1.:
            self._status(True)
            self.last_status = now

    def serve(self):
        if self.client is None:
            import carla
            port = int(os.environ["AIR_GROUND_CARLA_PORT"]) if os.environ.get("AIR_GROUND_CARLA_PORT") else 2400
            self.client = carla.Client("127.0.0.1", port)
            self.client.set_timeout(3.)
        previous_error = None
        try:
            while not self.stop.is_set():
                try:
                    self.step()
                    previous_error = None
                except (RuntimeError, OSError, KeyError, ValueError) as exc:
                    message = str(exc)
                    self.detach()
                    self._status(False, message)
                    if message != previous_error:
                        print("RGB-D capture waiting:", message, flush=True)
                        previous_error = message
                    self.stop.wait(.4)
                self.stop.wait(.02)
        finally:
            self.detach()
            self._status(False, "subscriber_stopped")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--max-hz", type=float, default=3.)
    args = parser.parse_args(argv)
    service = CaptureService(args.root, args.max_hz)
    home = Path(os.environ["AIR_GROUND_RUNTIME"]) if os.environ.get("AIR_GROUND_RUNTIME") else args.root / "runtime"
    lock_path = home / "perception_capture.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("An RGB-D capture subscriber already owns this project lock")
        lock.seek(0)
        lock.truncate()
        lock.write(str(os.getpid()))
        lock.flush()
        for signum in (signal.SIGTERM, signal.SIGINT):
            signal.signal(signum, lambda *_: service.stop.set())
        service.serve()


if __name__ == "__main__":
    main()
