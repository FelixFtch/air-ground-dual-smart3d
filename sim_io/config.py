"""Resolve immutable robot/appearance/sensor profiles into isolated instances."""
from pathlib import Path
import hashlib
import json
import re
import yaml

ROOT = Path(__file__).resolve().parents[1]


def runtime_home():
    """Per-stack runtime directory. A second UE sets AIR_GROUND_RUNTIME so the two stacks do not share latest_run.json."""
    import os
    override = os.environ.get('AIR_GROUND_RUNTIME')
    return Path(override) if override else ROOT / 'runtime'


def carla_port(default=2400):
    import os
    raw = os.environ.get('AIR_GROUND_CARLA_PORT')
    return int(raw) if raw else default


def load_team(name='go2_fighter', root=ROOT):
    root = Path(root).resolve()
    def read(kind, key):
        if not re.fullmatch(r'[a-z0-9_]+', key):
            raise ValueError('invalid profile identifier')
        p = root / 'profiles' / kind / (key + '.yaml')
        return yaml.safe_load(p.read_text())
    team = read('teams', name)
    team['scene_config'] = read('scenes', team['scene'])
    ids = set()
    for instance in team['robots']:
        rid = instance['robot_id']
        if not re.fullmatch(r'[a-z][a-z0-9_]{1,31}', rid) or rid in ids:
            raise ValueError('duplicate or invalid robot_id: ' + rid)
        ids.add(rid)
        robot = read('robots', instance['profile'])
        if (robot['dimension'], robot['mobility']) not in ((2, 'ground_xy_yaw'), (3, 'xyz_yaw')):
            raise ValueError('mobility and planner dimension mismatch')
        if len(robot['body_half_extents_m']) != 3 or min(robot['body_half_extents_m']) <= 0:
            raise ValueError('invalid collision envelope')
        for point in [instance['spawn_map_xyz'], *instance['route']]:
            if len(point) != 3:
                raise ValueError('all task coordinates must contain XYZ')
            if robot['dimension'] == 2 and abs(point[2] - robot['planar_z']) > 1e-6:
                raise ValueError('ground route changes altitude')
        instance.update(robot_config=robot, appearance_config=read('appearances', robot['appearance']),
                        sensors_config=read('sensors', robot['sensors']), role_name='ag_' + rid,
                        base_frame=rid + '/base_link', namespace='/' + rid)
    if team['scene_config']['carla_port'] in (2000, 2100, 2200, 2300):
        raise ValueError('reserved stable-scene port')
    team['profile_hash'] = hashlib.sha256(json.dumps(team, sort_keys=True).encode()).hexdigest()
    return team
