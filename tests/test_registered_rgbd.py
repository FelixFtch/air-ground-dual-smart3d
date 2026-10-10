"""CPU transport and capture-coordinate acceptance tests; no live simulator."""
import copy
import io
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest

from perception.capture import (ExactPairBuffer, build_capture, camera_intrinsics,
                                capture_transforms, rigid_camera_extrinsic)
from perception.registered_rgbd import (HEIGHT, WIDTH, MAX_BYTES, decode_capture,
                                      encode_capture, load_capture, load_latest,
                                      publish_capture)


def measurements(frame=40, timestamp=10., matrix=None, distance=2.):
    matrix = np.eye(4) if matrix is None else matrix
    transform = SimpleNamespace(get_matrix=lambda: matrix)
    value = round(distance/1000*16777215)
    common = dict(width=WIDTH, height=HEIGHT, frame=frame, timestamp=timestamp, transform=transform)
    rgb = SimpleNamespace(**common, raw_data=bytes([11, 22, 33, 255])*(WIDTH*HEIGHT))
    depth = SimpleNamespace(**common, raw_data=bytes([(value >> 16)&255, (value >> 8)&255, value&255, 255])*(WIDTH*HEIGHT))
    return rgb, depth


def example(frame=40, robot_id="go2_01", epoch="epoch-a", matrix=None, distance=2.):
    camera = dict(width=WIDTH, height=HEIGHT, fov=100, mount_m=[.5, 0, .4], pitch_degrees=0)
    robot = dict(robot_id=robot_id, sensors_config=dict(id="test-camera", camera=camera))
    scene = dict(scene_id="air_ground_lab_v1", content_version="test-v1", carla_world_origin_m=[0, 0, .22])
    rgb, depth = measurements(frame, 10.+frame/30, matrix, distance)
    return build_capture(rgb, depth, scene=scene, scene_epoch=epoch, robot=robot, receive_wall_time=12345.)


def payload(capture):
    return encode_capture(capture["rgb"], capture["xyz_optical"], capture["meta"])


def rewrite_archive(data, *, rgb=None, xyz=None, meta=None, compress=False):
    with np.load(io.BytesIO(data), allow_pickle=False) as source:
        fields = {key: source[key] for key in source.files}
    if rgb is not None:
        fields["rgb"] = rgb
    if xyz is not None:
        fields["xyz"] = xyz
    if meta is not None:
        fields["metadata"] = np.frombuffer(json.dumps(meta).encode(), dtype=np.uint8)
    output = io.BytesIO()
    (np.savez_compressed if compress else np.savez)(output, **fields)
    return output.getvalue()


def test_roundtrip_full_registered_raster_and_rgb_order():
    capture = example()
    result = decode_capture(payload(capture), expected={"robot_id": "go2_01", "scene_epoch": "epoch-a"})
    assert result["rgb"].shape == (360, 640, 3)
    assert result["rgb"].dtype == np.uint8
    assert result["rgb"][0, 0].tolist() == [33, 22, 11]
    assert result["xyz_optical"].dtype == np.float32
    np.testing.assert_array_equal(result["xyz_optical"], capture["xyz_optical"])
    assert result["meta"]["pixel_center_offset"] == .5
    assert result["meta"]["body_to_map"][0][3] == -.5


@pytest.mark.parametrize("key,value", [("scene_epoch", "wrong"), ("robot_id", "uav_01"),
                                      ("calibration_id", "wrong")])
def test_identity_rejects_cross_source_epoch_or_calibration(key, value):
    with pytest.raises(ValueError, match="identity_mismatch"):
        decode_capture(payload(example()), expected={key: value})


def test_tokens_include_frame_robot_and_capture_pose():
    base = decode_capture(payload(example()))["meta"]["frame_token"]
    moved = np.eye(4)
    moved[0, 3] = 1.
    tokens = [decode_capture(payload(item))["meta"]["frame_token"] for item in
              (example(frame=41), example(robot_id="uav_01"), example(epoch="epoch-b"), example(matrix=moved))]
    assert len(set(tokens + [base])) == 5


@pytest.mark.parametrize("field", ["rgb_frame_seq", "depth_frame_seq", "rgb_capture_sim_time", "depth_capture_sim_time"])
def test_source_frame_mixing_is_rejected(field):
    capture = example()
    capture["meta"][field] += 1
    with pytest.raises(ValueError, match="source_frame_mismatch"):
        payload(capture)


def test_rgb_depth_pose_and_time_mismatch_rejected_before_publication():
    camera = dict(width=640, height=360, fov=100, mount_m=[.5, 0, .4])
    kwargs = dict(scene=dict(scene_id="lab", content_version="v1", carla_world_origin_m=[0, 0, .22]),
                  scene_epoch="test", robot=dict(robot_id="go2", sensors_config=dict(id="cam", camera=camera)),
                  receive_wall_time=1.)
    rgb, depth = measurements()
    depth.timestamp += .01
    with pytest.raises(ValueError, match="capture_mismatch"):
        build_capture(rgb, depth, **kwargs)
    rgb, depth = measurements()
    pose = np.eye(4)
    pose[0, 3] = .01
    depth.transform = SimpleNamespace(get_matrix=lambda: pose)
    with pytest.raises(ValueError, match="pose_mismatch"):
        build_capture(rgb, depth, **kwargs)


@pytest.mark.parametrize("distance", [0., .1, 31., 999.])
def test_out_of_range_depth_is_nan(distance):
    result = decode_capture(payload(example(distance=distance)))
    assert np.isnan(result["xyz_optical"]).all()


def test_pixel_center_validation_and_known_corner_ray():
    capture = example()
    xyz, k = capture["xyz_optical"], np.asarray(capture["meta"]["K"])
    z = xyz[0, 0, 2]
    np.testing.assert_allclose(xyz[0, 0], [(0.5-320)*z/k[0, 0], (0.5-180)*z/k[1, 1], z], atol=1e-6)
    capture["xyz_optical"][..., 0] += .5*z/k[0, 0]
    with pytest.raises(ValueError, match="unregistered_optical_raster"):
        payload(capture)


def test_pitched_uav_and_body_turn_produce_capture_proper_rotations():
    body = rigid_camera_extrinsic(dict(mount_m=[5., 4., 3.], yaw_degrees=67, pitch_degrees=12, roll_degrees=8))
    extrinsic = rigid_camera_extrinsic(dict(mount_m=[.6, .1, -.2], yaw_degrees=-7, pitch_degrees=-25))
    sensor = body @ extrinsic
    camera_map, body_map = capture_transforms(sensor, extrinsic, [0, 0, .22])
    optical_point = np.array([1., 2., 5., 1.])
    sensor_point = np.array([5., 1., -2., 1.])
    world_point = sensor @ sensor_point
    expected = [world_point[0], -world_point[1], world_point[2]-.22, 1.]
    np.testing.assert_allclose(camera_map @ optical_point, expected, atol=1e-12)
    np.testing.assert_allclose(body_map[:3, 3], [5., -4., 2.78], atol=1e-12)
    assert np.linalg.det(camera_map[:3, :3]) == pytest.approx(1.)
    assert np.linalg.det(body_map[:3, :3]) == pytest.approx(1.)
    # A static world point stays static when the capture-time UAV pose changes.
    static_point = np.array([10., -2., 1., 1.])
    estimates = []
    for yaw in (-90., 0., 75.):
        moving = rigid_camera_extrinsic(dict(mount_m=[5., 4., 3.], yaw_degrees=yaw, pitch_degrees=15))
        camera_map, _ = capture_transforms(moving @ extrinsic, extrinsic, [0, 0, .22])
        optical = np.linalg.inv(camera_map) @ static_point
        estimates.append(camera_map @ optical)
    np.testing.assert_allclose(estimates, np.tile(static_point, (3, 1)), atol=1e-12)


def test_arrays_and_metadata_tampering_detected():
    data = payload(example())
    result = decode_capture(data)
    rgb = result["rgb"].copy()
    rgb[0, 0, 0] ^= 1
    with pytest.raises(ValueError, match="array_hash_mismatch"):
        decode_capture(rewrite_archive(data, rgb=rgb))
    result["meta"]["scene_epoch"] = "forged"
    with pytest.raises(ValueError, match="frame_token_mismatch"):
        decode_capture(rewrite_archive(data, meta=result["meta"]))


def test_reflection_transform_invalid_component_and_wrong_dtype_rejected():
    capture = example()
    capture["meta"]["body_to_map"][1][1] = -1
    with pytest.raises(ValueError, match="body_to_map"):
        payload(capture)
    capture = example()
    capture["xyz_optical"][0, 0, 0] = np.inf
    with pytest.raises(ValueError, match="depth_components"):
        payload(capture)
    capture = example()
    capture["xyz_optical"] = capture["xyz_optical"].astype(np.float64)
    with pytest.raises(ValueError, match="layout"):
        payload(capture)


def test_zip_amplification_members_and_array_header_rejected(monkeypatch):
    data = payload(example())
    with pytest.raises(ValueError, match="member_size"):
        decode_capture(rewrite_archive(data, compress=True))
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, "w") as archive:
        archive.writestr("rgb.npy", b"no numpy header")
    with pytest.raises(ValueError, match="members"):
        decode_capture(stream.getvalue())
    stream = io.BytesIO()
    with zipfile.ZipFile(io.BytesIO(data)) as source, zipfile.ZipFile(stream, "w") as target:
        for name in source.namelist():
            member = source.read(name)
            if name == "xyz.npy":
                malicious = io.BytesIO()
                np.lib.format.write_array_header_1_0(malicious, dict(shape=(999999999, 3), descr="<f4", fortran_order=False))
                member = malicious.getvalue()
            target.writestr(name, member)
    def forbidden_load(*args, **kwargs):
        raise AssertionError("NumPy allocator reached before header validation")
    monkeypatch.setattr(np, "load", forbidden_load)
    with pytest.raises(ValueError, match="layout"):
        decode_capture(stream.getvalue())
    with pytest.raises(ValueError, match="payload_too_large"):
        decode_capture(b"x"*(MAX_BYTES+1))


def test_publication_is_immutable_and_ring_is_bounded(tmp_path):
    capture = example()
    initial = publish_capture(tmp_path, capture["rgb"], capture["xyz_optical"], capture["meta"])
    first_path = tmp_path / initial["file"]
    # Hold a descriptor like load_capture's reader while the writer collects it.
    with first_path.open("rb") as already_open:
        for frame in range(41, 50):
            capture = example(frame=frame)
            manifest = publish_capture(tmp_path, capture["rgb"], capture["xyz_optical"], capture["meta"])
        assert not first_path.exists()
        assert decode_capture(already_open.read())["meta"]["frame_seq"] == 40
    assert len(list(tmp_path.glob("*.npz"))) == 8
    assert not list(tmp_path.glob("*.tmp"))
    latest = load_latest(tmp_path, expected={"scene_epoch": "epoch-a"})
    assert latest["meta"]["frame_seq"] == 49
    assert manifest["payload_bytes"] < MAX_BYTES
    target = tmp_path / manifest["file"]
    before = target.stat().st_mtime_ns
    publish_capture(tmp_path, capture["rgb"], capture["xyz_optical"], capture["meta"])
    assert target.stat().st_mtime_ns == before


def test_manifest_integrity_and_path_traversal_rejected(tmp_path):
    capture = example()
    manifest = publish_capture(tmp_path, capture["rgb"], capture["xyz_optical"], capture["meta"])
    manifest["robot_id"] = "wrong"
    (tmp_path / "latest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="manifest_identity"):
        load_latest(tmp_path)
    manifest["file"] = "../" + manifest["file"]
    (tmp_path / "latest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="manifest"):
        load_latest(tmp_path)


def test_pair_buffer_is_bounded_and_selects_only_latest_exact_frame():
    buffer = ExactPairBuffer(limit=4)
    for frame in range(20):
        rgb, depth = measurements(frame=frame)
        buffer.add("rgb", rgb, wall=frame)
    assert len(buffer.rgb) == 4
    assert buffer.take_latest() is None
    for frame in (16, 18):
        _, depth = measurements(frame=frame)
        buffer.add("depth", depth, wall=frame+.1)
    rgb, depth, wall = buffer.take_latest()
    assert rgb.frame == depth.frame == 18
    assert wall == 18.1
    buffer.add("depth", measurements(frame=17)[1])
    assert len(buffer.depth) == 0
    assert list(buffer.rgb) == [19]
