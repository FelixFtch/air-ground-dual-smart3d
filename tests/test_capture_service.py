"""Lifecycle isolation tests with mock sensors, no simulator connections."""
import json
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

import perception.capture as capture_module
from perception.capture import CaptureService, active_run
from test_registered_rgbd import measurements


class Sensor:
    def __init__(self, sid, rid, parent_id, kind, width=640, height=360):
        self.id = sid
        self.type_id = 'sensor.camera.' + kind
        self.attributes = dict(image_size_x=str(width), image_size_y=str(height), fov='100')
        self.parent = SimpleNamespace(id=parent_id, attributes={'role_name': 'ag_' + rid})
        self.callback = None
        self.stop_count = 0

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.stop_count += 1
        self.callback = None


class Actors(list):
    def filter(self, pattern):
        return self


def fixture_run(tmp_path, epoch='epoch-a'):
    root = tmp_path
    run = root / 'runtime/runs' / epoch
    run.mkdir(parents=True, exist_ok=True)
    scene = dict(scene_id='lab', content_version='v1', carla_world_origin_m=[0, 0, .22],
                 carla_host='127.0.0.1', carla_port=2400, frame_id='map')
    robots = [dict(robot_id=rid, sensors_config=dict(id=rid+'-cam', camera=dict(
                    width=640, height=360, fov=100, mount_m=[.5, 0, .4]))) for rid in ('go2_01', 'uav_01')]
    profile = dict(scene_config=scene, robots=robots)
    state = dict(running=True, epoch=epoch, wall_time=time.time(), scene_id='lab', sim_time=10.,
                 world_id=1, robots=[dict(robot_id=rid, actor_id=i+1) for i, rid in enumerate(('go2_01', 'uav_01'))])
    (run / 'state.json').write_text(json.dumps(state))
    (run / 'resolved_profiles.json').write_text(json.dumps(profile))
    (root / 'runtime/latest_run.json').write_text(json.dumps(dict(epoch=epoch, run_root=str(run))))
    return root, run, state, profile


def fixture_service(tmp_path):
    root, run, state, profile = fixture_run(tmp_path)
    sensors = Actors([Sensor(1, 'go2_01', 1, 'rgb'), Sensor(2, 'go2_01', 1, 'depth'),
                      Sensor(3, 'uav_01', 2, 'rgb'), Sensor(4, 'uav_01', 2, 'depth'),
                      Sensor(5, 'uav_01', 2, 'depth', width=96, height=96),
                      Sensor(6, 'go2_01', 777, 'rgb')])
    world = SimpleNamespace(id=1, get_actors=lambda: sensors,
                            get_snapshot=lambda: SimpleNamespace(frame=100, timestamp=SimpleNamespace(elapsed_seconds=state['sim_time'])))
    client = SimpleNamespace(get_world=lambda: world)
    return CaptureService(root, client=client), sensors, run, state


def test_subscribes_existing_unique_pair_only_and_never_auxiliary_or_wrong_actor(tmp_path):
    service, sensors, _, _ = fixture_service(tmp_path)
    service.step()
    assert set(service.subscriptions) == {1, 2, 3, 4}
    assert sensors[4].callback is None and sensors[5].callback is None
    service.detach()
    assert all(sensor.stop_count == 1 for sensor in sensors[:4])
    assert all(sensor.stop_count == 0 for sensor in sensors[4:])


def test_duplicate_front_camera_fails_closed(tmp_path):
    service, sensors, _, _ = fixture_service(tmp_path)
    sensors.append(Sensor(7, 'go2_01', 1, 'rgb'))
    service.step()
    assert set(service.subscriptions) == {3, 4}


def test_both_sources_publish_independently_and_rate_is_bounded(tmp_path, monkeypatch):
    service, sensors, run, _ = fixture_service(tmp_path)
    service.step()
    for sensor in sensors[:4]:
        rgb, depth = measurements(frame=40, timestamp=10.)
        sensor.callback(rgb if sensor.type_id.endswith('rgb') else depth)
    service.step()
    for rid in ('go2_01', 'uav_01'):
        assert (run / rid / 'rgbd/latest.json').exists()
        assert service.stats[rid]['published'] == 1
    for sensor in sensors[:4]:
        rgb, depth = measurements(frame=41, timestamp=10.01)
        sensor.callback(rgb if sensor.type_id.endswith('rgb') else depth)
    # Publication duration varies; force a precise recent prior publish time.
    service.last_publish = {rid: time.monotonic() for rid in ('go2_01', 'uav_01')}
    service.step()
    assert all(service.stats[rid]['published'] == 1 for rid in service.stats)
    service.last_publish = {rid: time.monotonic()-1. for rid in ('go2_01', 'uav_01')}
    service.step()
    assert all(service.stats[rid]['published'] == 2 for rid in service.stats)
    with pytest.raises(ValueError):
        CaptureService(tmp_path, max_hz=3.1)


def test_restart_discards_old_callbacks_and_old_pairs(tmp_path):
    service, sensors, run, _ = fixture_service(tmp_path)
    service.step()
    old_callback = sensors[0].callback
    old_rgb, old_depth = measurements(frame=50)
    old_callback(old_rgb)
    fixture_run(tmp_path, epoch='epoch-b')
    service.step()
    old_callback(old_rgb)
    assert len(service.buffers['go2_01'].rgb) == 0
    assert service.run.name == 'epoch-b'
    assert all(sensor.stop_count == 1 for sensor in sensors[:4])


def test_clock_rollback_latches_until_new_epoch_even_after_detach(tmp_path):
    service, sensors, run, state = fixture_service(tmp_path)
    service.step()
    state['sim_time'] = 1.
    (run / 'state.json').write_text(json.dumps(state))
    with pytest.raises(ValueError, match='clock_rollback'):
        service.step()
    service.detach()
    with pytest.raises(ValueError, match='clock_rollback'):
        service.step()
    fixture_run(tmp_path, epoch='epoch-b')
    service.step()
    assert service.run.name == 'epoch-b'


def test_actual_snapshot_clock_accepts_capture_ahead_of_lagged_runtime_state(tmp_path):
    service, sensors, run, state = fixture_service(tmp_path)
    state['sim_time'] = 8.
    (run / 'state.json').write_text(json.dumps(state))
    world = service.client.get_world()
    world.get_snapshot = lambda: SimpleNamespace(frame=1000, timestamp=SimpleNamespace(elapsed_seconds=10.))
    service.step()
    for sensor in sensors[:2]:
        rgb, depth = measurements(frame=40, timestamp=10.)
        sensor.callback(rgb if sensor.type_id.endswith('rgb') else depth)
    service.step()
    assert service.stats['go2_01']['published'] == 1
    clock = json.loads((run / 'shared/sensor_clock.json').read_text())
    assert clock['sim_time'] == 10.
    assert clock['scene_epoch'] == 'epoch-a' and clock['scene_id'] == 'lab'
    assert clock['frame'] == 1000


def test_stopped_stale_wrong_epoch_and_cross_project_run_rejected(tmp_path):
    root, run, state, profile = fixture_run(tmp_path)
    state['running'] = False
    (run / 'state.json').write_text(json.dumps(state))
    with pytest.raises(ValueError, match='stopped_or_stale'):
        active_run(root)
    state['running'] = True
    state['wall_time'] = time.time()-10
    (run / 'state.json').write_text(json.dumps(state))
    with pytest.raises(ValueError, match='stopped_or_stale'):
        active_run(root)
    state['wall_time'] = time.time()
    state['epoch'] = 'different'
    (run / 'state.json').write_text(json.dumps(state))
    with pytest.raises(ValueError, match='stopped_or_stale'):
        active_run(root)
    (root / 'runtime/latest_run.json').write_text(json.dumps(dict(epoch='no', run_root='/tmp/no')))
    with pytest.raises(ValueError, match='outside_project'):
        active_run(root)
