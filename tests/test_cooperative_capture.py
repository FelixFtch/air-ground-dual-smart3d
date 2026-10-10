"""Capture-time cooperative body exclusions; no simulator or model connections."""
import copy
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

from perception.capture import PAIR_HISTORY, capture_transforms, rigid_camera_extrinsic
from perception.registered_rgbd import decode_capture, load_latest, validate_cooperative_bodies
from test_capture_service import fixture_service
from test_registered_rgbd import example, measurements, payload, rewrite_archive


def configured_service(tmp_path):
    service, sensors, run, state = fixture_service(tmp_path)
    path = run / 'resolved_profiles.json'
    profile = json.loads(path.read_text())
    for robot in profile['robots']:
        robot['robot_config'] = dict(body_half_extents_m=[.43, .43, .28], body_offset_m=[0, 0, 0])
    path.write_text(json.dumps(profile))
    service.step()
    return service, sensors, run, state


def send(sensors, rid, frame=40, stamp=10., matrix=None, kinds=('rgb', 'depth')):
    rgb, depth = measurements(frame=frame, timestamp=stamp, matrix=matrix)
    for sensor in sensors[:4]:
        if sensor.parent.attributes['role_name'] == 'ag_'+rid:
            kind = sensor.type_id.rsplit('.', 1)[1]
            if kind in kinds:
                sensor.callback(rgb if kind == 'rgb' else depth)


def capture_with_bodies():
    c = example()
    m = c['meta']
    own = dict(robot_id=m['robot_id'], frame_seq=m['frame_seq'], capture_sim_time=m['capture_sim_time'],
               body_to_map=copy.deepcopy(m['body_to_map']), half_extents_xyz=[.43, .43, .28], body_offset_xyz=[0, 0, 0])
    partner = copy.deepcopy(own)
    partner['robot_id'] = 'uav_01'
    partner['body_to_map'][0][3] += 5
    m['cooperative_bodies'] = [own, partner]
    return c


def test_both_body_poses_come_from_exact_depth_frame_and_are_sealed(tmp_path):
    service, sensors, run, state = configured_service(tmp_path)
    first = rigid_camera_extrinsic(dict(mount_m=[2, 3, 1], yaw_degrees=65))
    second = rigid_camera_extrinsic(dict(mount_m=[-4, 1, 3], yaw_degrees=-20, pitch_degrees=8))
    send(sensors, 'go2_01', matrix=first)
    send(sensors, 'uav_01', matrix=second)
    service.step()
    for rid in ('go2_01', 'uav_01'):
        meta = load_latest(run/rid/'rgbd')['meta']
        bodies = {b['robot_id']: b for b in validate_cooperative_bodies(meta)}
        assert set(bodies) == {'go2_01', 'uav_01'}
        for target, sensor_pose in [('go2_01', first), ('uav_01', second)]:
            robot = next(r for r in service.profile['robots'] if r['robot_id'] == target)
            expected = capture_transforms(sensor_pose, rigid_camera_extrinsic(robot['sensors_config']['camera']), [0, 0, .22])[1]
            np.testing.assert_allclose(bodies[target]['body_to_map'], expected, atol=1e-10)
            assert bodies[target]['frame_seq'] == meta['frame_seq'] == 40
            assert bodies[target]['capture_sim_time'] == meta['capture_sim_time'] == 10.


def test_missing_partner_frame_defers_and_then_publishes_without_losing_pair(tmp_path):
    service, sensors, run, _ = configured_service(tmp_path)
    send(sensors, 'go2_01')
    service.step()
    assert not (run/'go2_01/rgbd/latest.json').exists()
    assert service.buffers['go2_01'].last_taken == -1
    send(sensors, 'uav_01')
    service.step()
    assert load_latest(run/'go2_01/rgbd')['meta']['frame_seq'] == 40


def test_wrong_partner_timestamp_is_never_attached_and_next_exact_frame_recovers(tmp_path):
    service, sensors, run, _ = configured_service(tmp_path)
    send(sensors, 'go2_01', stamp=10.)
    send(sensors, 'uav_01', stamp=10.001)
    service.step()
    assert not (run/'go2_01/rgbd/latest.json').exists()
    send(sensors, 'go2_01', frame=41, stamp=10.01)
    send(sensors, 'uav_01', frame=41, stamp=10.01)
    service.step()
    assert load_latest(run/'go2_01/rgbd')['meta']['frame_seq'] == 41


def test_complete_shared_frame_is_used_even_when_one_camera_has_a_newer_frame(tmp_path):
    service, sensors, run, _ = configured_service(tmp_path)
    send(sensors, 'go2_01', frame=40)
    send(sensors, 'uav_01', frame=40)
    send(sensors, 'go2_01', frame=41, stamp=10.01)
    service.step()
    assert load_latest(run/'go2_01/rgbd')['meta']['frame_seq'] == 40
    assert 41 in service.buffers['go2_01'].rgb


def test_history_is_bounded_and_old_callbacks_cannot_restore_it_after_detach(tmp_path):
    service, sensors, _, _ = configured_service(tmp_path)
    callback = sensors[1].callback
    transform = SimpleNamespace(get_matrix=lambda: np.eye(4))
    for frame in range(40, 40+PAIR_HISTORY+5):
        callback(SimpleNamespace(frame=frame, timestamp=10.+frame/30, transform=transform))
    assert len(service.body_history) == PAIR_HISTORY
    service.detach()
    callback(SimpleNamespace(frame=99, timestamp=11., transform=transform))
    assert not service.body_history


def test_legacy_profiles_without_body_geometry_keep_transport_compatible(tmp_path):
    service, sensors, run, _ = fixture_service(tmp_path)
    service.step()
    send(sensors, 'go2_01')
    service.step()
    assert 'cooperative_bodies' not in load_latest(run/'go2_01/rgbd')['meta']


def test_partial_body_profiles_do_not_silently_disable_cooperation(tmp_path):
    service, sensors, run, _ = configured_service(tmp_path)
    del service.profile['robots'][1]['robot_config']
    with pytest.raises(ValueError, match='incomplete_cooperative'):
        service._cooperative_frames()


def test_optional_validator_is_dimension_independent_and_missing_means_empty():
    assert validate_cooperative_bodies({}) == []
    meta = capture_with_bodies()['meta']
    meta['width'], meta['height'] = 40, 40
    assert len(validate_cooperative_bodies(meta)) == 2


@pytest.mark.parametrize('field,value', [('frame_seq', 41), ('frame_seq', True),
    ('capture_sim_time', 10.), ('capture_sim_time', float('nan')),
    ('half_extents_xyz', [0, 1, 1]), ('half_extents_xyz', [True, 1, 1]),
    ('half_extents_xyz', [1, 1, float('inf')]), ('body_offset_xyz', [0, 0]),
    ('body_offset_xyz', [0, 0, float('nan')]), ('body_to_map', np.zeros((4, 4)).tolist())])
def test_invalid_or_cross_frame_partner_metadata_is_rejected(field, value):
    c = capture_with_bodies()
    c['meta']['cooperative_bodies'][1][field] = value
    with pytest.raises(ValueError):
        validate_cooperative_bodies(c['meta'])


@pytest.mark.parametrize('mutation', ['duplicate', 'missing_own', 'wrong_own_pose', 'empty'])
def test_body_identity_and_source_pose_are_checked(mutation):
    meta = capture_with_bodies()['meta']
    if mutation == 'duplicate':
        meta['cooperative_bodies'][1]['robot_id'] = meta['robot_id']
    elif mutation == 'missing_own':
        meta['cooperative_bodies'] = meta['cooperative_bodies'][1:]
    elif mutation == 'wrong_own_pose':
        meta['cooperative_bodies'][0]['body_to_map'][0][3] += .1
    else:
        meta['cooperative_bodies'] = []
    with pytest.raises(ValueError):
        validate_cooperative_bodies(meta)


def test_body_geometry_is_hashed_into_frame_token_and_tamper_rejected():
    c = capture_with_bodies()
    original = payload(c)
    decoded = decode_capture(original)
    assert decoded['meta']['cooperative_bodies'] == c['meta']['cooperative_bodies']
    changed = copy.deepcopy(c)
    changed['meta']['cooperative_bodies'][1]['body_to_map'][0][3] += .1
    assert decode_capture(payload(changed))['meta']['frame_token'] != decoded['meta']['frame_token']
    tampered = copy.deepcopy(decoded['meta'])
    tampered['cooperative_bodies'][1]['body_to_map'][0][3] += .1
    with pytest.raises(ValueError, match='frame_token_mismatch'):
        decode_capture(rewrite_archive(original, meta=tampered))
