#!/usr/bin/env python3
"""Bounded CARLA live tests for the isolated AirGroundLab UAV actor.

Creates only temporary actors with a per-run role. Does not tick/reload the world,
change settings, control existing robots or edit maps. Requires an empty 4m cube.
"""
from __future__ import annotations
import argparse
import json
import math
import time
import uuid
import struct
from collections import deque
from pathlib import Path

import carla


def xyz(p):
    return [float(p.x), float(p.y), float(p.z)]


def distance(a, b):
    return math.sqrt(sum((u - v) ** 2 for u, v in zip(xyz(a), xyz(b))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2400)
    parser.add_argument('--center', nargs=3, type=float, default=[-7, -7, 2.5])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    map_name = world.get_map().name
    if 'AirGroundLab' not in map_name:
        raise RuntimeError(f'wrong_scene: {map_name}')
    if world.get_settings().synchronous_mode:
        raise RuntimeError('requires externally running asynchronous world; never ticks the world')
    blueprints = world.get_blueprint_library()
    role = 'ag_uav_smoke_' + uuid.uuid4().hex[:10]
    report = {'schema': 'air_ground.uav_smoke/v1', 'map': map_name,
              'started_unix': time.time(), 'role': role, 'center_world_m': args.center,
              'tests': [], 'owned_actor_ids': [], 'all_passed': False}
    owned = []

    def tick():
        snapshot = world.wait_for_tick(10.0)
        if snapshot is None:
            raise RuntimeError('world_tick_unavailable')
        return snapshot

    def spawn_uav(overrides=None):
        bp = blueprints.find('robot.uav.fighter_drone')
        bp.set_attribute('role_name', role)
        # Keep the body-space geofence comfortably beyond this bounded test.
        for axis, value in zip('xyz', args.center):
            bp.set_attribute(f'flight_min_{axis}_m', str(value - 3.0))
            bp.set_attribute(f'flight_max_{axis}_m', str(value + 3.0))
        for key, value in (overrides or {}).items():
            bp.set_attribute(key, str(value))
        uav = world.spawn_actor(bp, carla.Transform(carla.Location(*args.center)))
        owned.append(uav)
        report['owned_actor_ids'].append(uav.id)
        tick()
        return uav

    def destroy(actor):
        if actor in owned:
            actor.destroy()
            owned.remove(actor)
            tick()

    def request(actor, delta=(0, 0, 0), yaw_degrees=0):
        before_tick = tick()
        initial = actor.get_transform()
        desired = carla.Transform(
            carla.Location(initial.location.x + delta[0], initial.location.y + delta[1], initial.location.z + delta[2]),
            carla.Rotation(yaw=initial.rotation.yaw + yaw_degrees))
        responses = client.apply_batch_sync([carla.command.ApplyTransform(actor.id, desired)], False)
        after_tick = tick()
        actual = actor.get_transform()
        return {'frame_before': before_tick.frame, 'frame_after': after_tick.frame,
                'error': responses[0].error or None, 'before': xyz(initial.location),
                'after': xyz(actual.location), 'yaw_before': initial.rotation.yaw,
                'yaw_after': actual.rotation.yaw,
                'moved_m': distance(actual.location, initial.location),
                'reported_speed_mps': actor.get_velocity().length()}

    def case(name, fn):
        ids_before = {a.id for a in owned}
        try:
            result = fn()
            report['tests'].append({'name': name, 'passed': True, **result})
        except Exception as exc:
            report['tests'].append({'name': name, 'passed': False, 'error': str(exc)})
        finally:
            for actor in list(reversed(owned)):
                if actor.id not in ids_before:
                    try:
                        destroy(actor)
                    except Exception as exc:
                        report['tests'].append({'name': name + ':cleanup', 'passed': False, 'error': str(exc)})
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')

    def free_motion():
        uav = spawn_uav()
        extent = xyz(uav.bounding_box.extent)
        assert max(abs(x - y) for x, y in zip(extent, [.48, .48, .34])) < .002, extent
        sensor = world.spawn_actor(blueprints.find('sensor.other.imu'),
                                  carla.Transform(carla.Location(x=.1, z=.2)), attach_to=uav,
                                  attachment_type=carla.AttachmentType.Rigid)
        owned.append(sensor)
        report['owned_actor_ids'].append(sensor.id)
        events = []
        for axis in range(3):
            for sign in (1, -1):
                start = uav.get_location()
                for _ in range(100):
                    delta = [0.0] * 3
                    delta[axis] = .075 * sign
                    event = request(uav, delta)
                    events.append(event)
                    assert not event['error'], event
                    assert event['moved_m'] <= .151, event
                    assert event['reported_speed_mps'] <= 1.51, event
                    if (xyz(uav.get_location())[axis] - xyz(start)[axis]) * sign > .20:
                        break
                assert (xyz(uav.get_location())[axis] - xyz(start)[axis]) * sign > .15, events[-1]
                request(uav)  # explicit zero displacement stops before reversing
        original_yaw = uav.get_transform().rotation.yaw
        for _ in range(25):
            event = request(uav, yaw_degrees=4.0)
            events.append(event)
            assert not event['error'], event
        assert abs(uav.get_transform().rotation.yaw - original_yaw) > 5.0
        expected_sensor = uav.get_transform().transform(carla.Location(x=.1, z=.2))
        assert distance(sensor.get_location(), expected_sensor) < .02
        request(uav)
        stationary = uav.get_location()
        time.sleep(.5)
        tick()
        assert distance(uav.get_location(), stationary) < .001, 'drift_after_commands_stop'
        assert uav.get_velocity().length() < .001, 'stale_velocity'
        return {'extent_m': extent, 'events': events, 'rigid_sensor_follow': True, 'no_drift': True}

    def lidar_parent_exclusion():
        uav = spawn_uav()
        bp = blueprints.find('sensor.lidar.ray_cast')
        settings = {'channels': '96', 'points_per_second': '192000',
                    'rotation_frequency': '10', 'sensor_tick': '.1', 'range': '12',
                    'upper_fov': '75', 'lower_fov': '-85', 'noise_stddev': '0',
                    'dropoff_general_rate': '0', 'dropoff_zero_intensity': '0'}
        for key, value in settings.items():
            bp.set_attribute(key, value)
        lidar = world.spawn_actor(bp, carla.Transform(carla.Location(z=.4)), attach_to=uav,
                                  attachment_type=carla.AttachmentType.Rigid)
        owned.append(lidar)
        report['owned_actor_ids'].append(lidar.id)
        frames = deque(maxlen=3)
        lidar.listen(frames.append)
        cube = blueprints.find('static.prop.mesh')
        cube.set_attribute('mesh_path', '/Engine/BasicShapes/Cube.Cube')
        cube.set_attribute('scale', '.3')
        at = list(args.center)
        at[0] += 1.0
        obstacle = world.spawn_actor(cube, carla.Transform(carla.Location(*at)))
        owned.append(obstacle)
        report['owned_actor_ids'].append(obstacle.id)
        deadline = time.monotonic() + 10
        while len(frames) < 3 and time.monotonic() < deadline:
            tick()
        lidar.stop()
        assert frames, 'no_lidar_frame'
        points = [p for frame in frames for p in struct.iter_unpack('ffff', frame.raw_data)]
        ego = [p for p in points if abs(p[0]) < .481 and abs(p[1]) < .481 and abs(p[2] + .4) < .341]
        down = [p for p in points if p[2] < -.6]
        external_cube = [p for p in points if .83 < p[0] < 1.17 and abs(p[1]) < .17 and -.57 < p[2] < -.23]
        assert not ego, f'own_body_lidar_returns: {len(ego)}'
        assert len(down) > 100, f'no_downward_space: {len(down)}'
        assert external_cube, 'other_obstacle_not_visible'
        return {'points': len(points), 'own_body_returns': len(ego),
                'downward_returns': len(down), 'other_obstacle_returns': len(external_cube)}

    def invalid_request():
        uav = spawn_uav()
        event = request(uav, (1.0, 0, 0))
        assert 'request_exceeds_bounded_move' in (event['error'] or ''), event
        assert event['moved_m'] < .001, event
        return {'event': event}

    def collision(axis, sign):
        uav = spawn_uav()
        bp = blueprints.find('static.prop.mesh')
        bp.set_attribute('mesh_path', '/Engine/BasicShapes/Cube.Cube')
        bp.set_attribute('scale', '.3')
        obstacle_at = list(args.center)
        obstacle_at[axis] += sign * 1.0
        obstacle = world.spawn_actor(bp, carla.Transform(carla.Location(*obstacle_at)))
        owned.append(obstacle)
        report['owned_actor_ids'].append(obstacle.id)
        tick()
        events = []
        blocked = False
        for _ in range(100):
            delta = [0.0] * 3
            delta[axis] = sign * .075
            event = request(uav, delta)
            events.append(event)
            if event['error']:
                assert any(k in event['error'] for k in ('swept_body_blocked', 'root_sweep_clamped')), event
                blocked = True
                break
        assert blocked, 'did_not_block'
        signed_travel = (xyz(uav.get_location())[axis] - args.center[axis]) * sign
        body_half = .34 if axis == 2 else .48
        assert signed_travel <= 1.0 - .15 - body_half + .006, signed_travel
        destroy(obstacle)
        before = uav.get_location()
        for _ in range(15):
            event = request(uav, delta)
            events.append(event)
            assert not event['error'], event
        assert (xyz(uav.get_location())[axis] - xyz(before)[axis]) * sign > .05
        return {'axis': axis, 'sign': sign, 'blocked_travel_m': signed_travel,
                'resumed_after_removal': True, 'events': events}

    def geofence():
        uav = spawn_uav({'flight_max_x_m': args.center[0] + .70})
        events = []
        blocked = False
        for _ in range(100):
            event = request(uav, (.075, 0, 0))
            events.append(event)
            if event['error']:
                assert 'flight_bounds' in event['error'], event
                blocked = True
                break
        assert blocked, 'flight_bound_not_enforced'
        assert uav.get_location().x <= args.center[0] + .22 + .002
        return {'events': events}

    def yaw_collision():
        uav = spawn_uav()
        bp = blueprints.find('static.prop.mesh')
        bp.set_attribute('mesh_path', '/Engine/BasicShapes/Cube.Cube')
        bp.set_attribute('scale', '.08')
        obstacle_at = list(args.center)
        obstacle_at[0] += .63
        obstacle = world.spawn_actor(bp, carla.Transform(carla.Location(*obstacle_at)))
        owned.append(obstacle)
        report['owned_actor_ids'].append(obstacle.id)
        tick()
        events = []
        blocked = False
        for _ in range(160):
            event = request(uav, yaw_degrees=4)
            events.append(event)
            if event['error']:
                assert any(k in event['error'] for k in ('swept_body_blocked', 'root_sweep_clamped')), event
                blocked = True
                break
        assert blocked, 'yaw_sweep_did_not_block'
        assert 1 < uav.get_transform().rotation.yaw < 46, events[-1]
        destroy(obstacle)
        before = uav.get_transform().rotation.yaw
        for _ in range(25):
            event = request(uav, yaw_degrees=4)
            events.append(event)
            assert not event['error'], event
        assert abs(uav.get_transform().rotation.yaw - before) > 5
        return {'events': events, 'resumed_after_removal': True}

    try:
        case('free_xyz_yaw_sensor_stop', free_motion)
        case('lidar_parent_exclusion_other_obstacle_visible', lidar_parent_exclusion)
        case('oversize_request_rejected', invalid_request)
        case('geofence_full_body', geofence)
        case('yaw_sweep_collision', yaw_collision)
        for axis, axis_name in enumerate('xyz'):
            for sign in (1, -1):
                case(f'sweep_{axis_name}_{sign:+d}', lambda axis=axis, sign=sign: collision(axis, sign))
    finally:
        for actor in list(reversed(owned)):
            try:
                actor.destroy()
            except Exception:
                pass
        report['finished_unix'] = time.time()
        report['all_passed'] = bool(report['tests']) and all(t['passed'] for t in report['tests'])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'all_passed': report['all_passed'],
                      'tests': [{k: v for k, v in t.items() if k in ('name', 'passed', 'error')} for t in report['tests']],
                      'report': str(args.output)}, indent=2))
    raise SystemExit(0 if report['all_passed'] else 1)


if __name__ == '__main__':
    main()
