import copy
import tempfile
import unittest
from pathlib import Path
import sys
import yaml
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_io.config import load_team, ROOT
from sim_io.geometry import to_map_xyz, to_carla_xyz, transform_points, depth_points


class ProfilesGeometry(unittest.TestCase):
    def test_distinct_instances(self):
        t = load_team()
        self.assertEqual([r['robot_config']['dimension'] for r in t['robots']], [2, 3])
        self.assertEqual(len({r['base_frame'] for r in t['robots']}), 2)

    def test_axis_and_capture_transform(self):
        origin = [0, 0, .201]
        a = np.array([[1., 2., 3.], [-4., -5., 6.]])
        np.testing.assert_allclose(to_carla_xyz(to_map_xyz(a, origin), origin), a)
        m = np.eye(4); m[:3, 3] = [4, 8, 1]
        np.testing.assert_allclose(transform_points([[1, 0, 0]], m, origin), [[5, -8, .799]])

    def test_no_depth_is_not_free(self):
        class Image: width=24; height=24; raw_data=bytes(24*24*4)
        hit, free, far = depth_points(Image(), 90)
        self.assertEqual(len(hit)+len(free), 0)

    def test_depth_far_plane_explicit(self):
        class Image: width=24; height=24; raw_data=bytes([255,255,255,255])*24*24
        hit, free, far = depth_points(Image(), 90)
        self.assertEqual(len(hit), 0)
        self.assertTrue(far.all())
        np.testing.assert_allclose(np.linalg.norm(free, axis=1), 12)

    def test_reject_duplicate_and_dimension(self):
        import shutil
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);shutil.copytree(ROOT/'profiles',root/'profiles')
            p=root/'profiles/teams/go2_fighter.yaml';t=yaml.safe_load(p.read_text())
            t['robots'][1]['robot_id']=t['robots'][0]['robot_id'];p.write_text(yaml.safe_dump(t))
            with self.assertRaisesRegex(ValueError,'duplicate'):load_team(root=root)


if __name__ == '__main__': unittest.main()
