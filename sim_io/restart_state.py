"""Validate a stopped runtime checkpoint before restoring pose and patrol progress.

This module never connects to CARLA, changes a profile, or restores a path/tree.
The caller must independently restore the matching map and revalidate new plans.
"""
from pathlib import Path
import json
import math


def _number(value, field, *, minimum=None, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{field} must be a finite number')
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f'{field} is outside its valid range')
    if integer and (not isinstance(value, int) or value != int(value)):
        raise ValueError(f'{field} must be an integer')
    return int(value) if integer else float(value)


def _xyz(value, field):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{field} must contain three coordinates')
    return [_number(v, field) for v in value]


def _robot_index(robots, field):
    if not isinstance(robots, list) or not robots:
        raise ValueError(f'{field} must contain robot identities')
    result = {}
    for robot in robots:
        if not isinstance(robot, dict):
            raise ValueError(f'{field} has an invalid robot identity')
        rid = robot.get('robot_id')
        if not isinstance(rid, str) or not rid or rid in result:
            raise ValueError(f'{field} has a duplicate or invalid robot identity')
        result[rid] = robot
    return result


def load_restart_state(team, state_path):
    """Return only validated per-robot pose/counters from a stopped checkpoint.

    The adjacent ``resolved_profiles.json`` must describe the same scene and
    robot profiles. Goals must uniquely identify an unchanged route point: never
    infer a route index from a nearest point or silently restart the patrol.
    Missing files and invalid JSON raise their normal I/O/JSON exceptions;
    invalid checkpoint content raises ValueError. The function is all-or-nothing.
    """
    state_path = Path(state_path)
    state = json.loads(state_path.read_text())
    profiles = json.loads((state_path.parent / 'resolved_profiles.json').read_text())
    if not isinstance(state, dict) or state.get('schema') != 'air_ground.state/v1':
        raise ValueError('unsupported restart state schema')
    if state.get('running') is not False:
        raise ValueError('restart state must be a stopped checkpoint (running=false)')
    if not isinstance(team, dict) or not isinstance(profiles, dict):
        raise ValueError('invalid resolved team profiles')
    profile_hash = team.get('profile_hash')
    if not isinstance(profile_hash, str) or not profile_hash or profiles.get('profile_hash') != profile_hash:
        raise ValueError('restart profile_hash mismatch')
    scene = team.get('scene_config', {})
    old_scene = profiles.get('scene_config', {})
    scene_id = scene.get('scene_id')
    if not scene_id or state.get('scene_id') != scene_id or old_scene.get('scene_id') != scene_id:
        raise ValueError('restart scene_id mismatch')
    current = _robot_index(team.get('robots'), 'current profiles')
    previous = _robot_index(profiles.get('robots'), 'saved profiles')
    saved = _robot_index(state.get('robots'), 'restart state')
    if current.keys() != previous.keys() or current.keys() != saved.keys():
        raise ValueError('restart robot identities mismatch')

    restored = {}
    for rid, instance in current.items():
        old = previous[rid]
        record = saved[rid]
        cfg = instance.get('robot_config', {})
        old_cfg = old.get('robot_config', {})
        dimension = cfg.get('dimension')
        if (dimension not in (2, 3)
                or not instance.get('profile')
                or instance.get('profile') != old.get('profile')
                or not cfg.get('profile_id')
                or cfg.get('profile_id') != old_cfg.get('profile_id')
                or dimension != old_cfg.get('dimension')
                or cfg.get('mobility') != old_cfg.get('mobility')
                or record.get('dimension') != dimension
                or record.get('kind') != cfg.get('mobility')):
            raise ValueError(f'{rid}: restart robot profile identity mismatch')
        position = _xyz(record.get('position_xyz'), f'{rid}.position')
        yaw = _number(record.get('yaw'), f'{rid}.yaw')
        bounds = scene.get('flight_bounds' if dimension == 3 else 'bounds')
        if not isinstance(bounds, (list, tuple)) or len(bounds) != 3:
            raise ValueError(f'{rid}: missing position bounds')
        for axis, interval in enumerate(bounds):
            if not isinstance(interval, (list, tuple)) or len(interval) != 2:
                raise ValueError(f'{rid}: invalid position bounds')
            low, high = (_number(v, f'{rid}.bounds') for v in interval)
            if low > high or not low <= position[axis] <= high:
                raise ValueError(f'{rid}: position outside permitted bounds')
        if dimension == 2:
            planar_z = _number(cfg.get('planar_z'), f'{rid}.planar_z')
            if abs(position[2] - planar_z) > 1e-4:
                raise ValueError(f'{rid}: position differs from planar_z')
        route = instance.get('route')
        if not isinstance(route, list) or not route or old.get('route') != route:
            raise ValueError(f'{rid}: missing or changed route')
        goal = _xyz(record.get('goal_xyz'), f'{rid}.goal_xyz')
        matches = [index for index, point in enumerate(route)
                   if all(abs(a - b) <= 1e-4 for a, b in zip(_xyz(point, f'{rid}.route'), goal))]
        if len(matches) != 1:
            raise ValueError(f'{rid}: goal_xyz must match exactly one route point')
        distance = _number(record.get('distance_m'), f'{rid}.distance_m', minimum=0)
        vertical = _number(record.get('vertical_distance_m'), f'{rid}.vertical_distance_m', minimum=0)
        if vertical > distance + 1e-6:
            raise ValueError(f'{rid}: vertical distance exceeds total distance')
        restored[rid] = dict(position=position, yaw=yaw, route_index=matches[0],
                            goal_revision=_number(record.get('goal_revision'), f'{rid}.goal_revision', minimum=1, integer=True),
                            distance_m=distance, vertical_distance_m=vertical,
                            completed_goals=_number(record.get('completed_goals'), f'{rid}.completed_goals', minimum=0, integer=True))
    return restored
