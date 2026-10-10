"""Restart admission rejects incompatible or unsafe pose/progress snapshots."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from sim_io.config import load_team
from sim_io.restart_state import load_restart_state


class RestartState(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / 'state.json'
        self.team = load_team()
        self.profiles = copy.deepcopy(self.team)
        self.state = dict(schema='air_ground.state/v1', running=False,
                          scene_id=self.team['scene_config']['scene_id'], robots=[])
        for instance in self.team['robots']:
            cfg = instance['robot_config']
            self.state['robots'].append(dict(
                robot_id=instance['robot_id'], dimension=cfg['dimension'], kind=cfg['mobility'],
                position_xyz=list(instance['spawn_map_xyz']), yaw=.73,
                goal_xyz=list(instance['route'][1]), goal_revision=23,
                distance_m=102.5, vertical_distance_m=3.25 if cfg['dimension'] == 3 else 0.,
                completed_goals=22))

    def load(self):
        self.path.write_text(json.dumps(self.state))
        self.path.with_name('resolved_profiles.json').write_text(json.dumps(self.profiles))
        return load_restart_state(self.team, self.path)

    def test_restore_both_robots_without_merging_pose_or_progress(self):
        before = copy.deepcopy(self.team)
        restored = self.load()
        self.assertEqual(self.team, before)
        self.assertEqual(set(restored), {'go2_01', 'uav_01'})
        for robot in self.state['robots']:
            value = restored[robot['robot_id']]
            self.assertEqual(value['position'], robot['position_xyz'])
            self.assertEqual(value['yaw'], .73)
            self.assertEqual(value['route_index'], 1)
            self.assertEqual(value['goal_revision'], 23)
            self.assertEqual(value['completed_goals'], 22)
            self.assertEqual(value['distance_m'], 102.5)
            self.assertEqual(set(value), {'position', 'yaw', 'route_index', 'goal_revision',
                                          'distance_m', 'vertical_distance_m', 'completed_goals'})
        restored['go2_01']['position'][0] = 999
        self.assertNotEqual(restored['uav_01']['position'][0], 999)
        self.assertEqual(self.team, before)

    def test_requires_stopped_checkpoint(self):
        for invalid in (True, 0, None, 'false'):
            with self.subTest(invalid=invalid):
                self.state['running'] = invalid
                with self.assertRaisesRegex(ValueError, 'stopped checkpoint'): self.load()

    def test_rejects_cross_scene_or_profile(self):
        mutations = [lambda: self.profiles.update(profile_hash='different'),
                     lambda: self.profiles['scene_config'].update(scene_id='factory'),
                     lambda: self.state.update(scene_id='town10')]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.profiles = copy.deepcopy(self.team)
                self.state['scene_id'] = self.team['scene_config']['scene_id']
                mutate()
                with self.assertRaisesRegex(ValueError, 'mismatch'): self.load()

    def test_rejects_missing_duplicate_or_cross_robot_identity(self):
        original = copy.deepcopy(self.state['robots'])
        invalid_lists = [original[:1], [original[0], original[0]],
                         [dict(original[0], robot_id='go2_02'), original[1]],
                         [dict(original[0], dimension=3), original[1]]]
        for invalid in invalid_lists:
            with self.subTest(invalid=invalid):
                self.state['robots'] = invalid
                with self.assertRaises(ValueError): self.load()
        self.state['robots'] = original
        self.profiles['robots'][0]['profile'] = 'different_robot'
        with self.assertRaisesRegex(ValueError, 'profile identity'): self.load()

    def test_rejects_invalid_pose_or_outside_flight_bounds(self):
        for field, invalid in [('position_xyz', [0, 0]), ('position_xyz', [float('nan'), 0, .33]),
                               ('position_xyz', [31, 0, .33]), ('position_xyz', [0, 0, .331]),
                               ('yaw', float('inf')), ('yaw', False)]:
            with self.subTest(field=field, invalid=invalid):
                old = self.state['robots'][0][field]
                self.state['robots'][0][field] = invalid
                with self.assertRaises(ValueError): self.load()
                self.state['robots'][0][field] = old
        self.state['robots'][1]['position_xyz'] = [25, 0, 2]
        with self.assertRaisesRegex(ValueError, 'outside permitted bounds'): self.load()

    def test_rejects_missing_or_ambiguous_route_point(self):
        self.state['robots'][0]['goal_xyz'] = [0, 0, .33]
        with self.assertRaisesRegex(ValueError, 'exactly one route point'): self.load()
        route = self.team['robots'][0]['route']
        self.state['robots'][0]['goal_xyz'] = route[1]
        route.append(list(route[1]))
        self.profiles['robots'][0]['route'] = copy.deepcopy(route)
        with self.assertRaisesRegex(ValueError, 'exactly one route point'): self.load()

    def test_rejects_changed_route_even_with_stale_hash(self):
        self.profiles['robots'][0]['route'][0][0] += 1
        with self.assertRaisesRegex(ValueError, 'changed route'): self.load()

    def test_rejects_invalid_counters(self):
        for field, invalid in [('goal_revision', 0), ('goal_revision', 1.5),
                               ('completed_goals', -1), ('completed_goals', True),
                               ('distance_m', -1), ('distance_m', float('inf')),
                               ('vertical_distance_m', 200)]:
            with self.subTest(field=field, invalid=invalid):
                old = self.state['robots'][0][field]
                self.state['robots'][0][field] = invalid
                with self.assertRaises(ValueError): self.load()
                self.state['robots'][0][field] = old


if __name__ == '__main__': unittest.main()
