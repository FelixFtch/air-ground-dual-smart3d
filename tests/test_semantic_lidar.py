"""Town10 semantic selection: people and sky stay out of the static voxel map."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("semantic_lidar", ROOT / "sim_io/semantic_lidar.py")
semantic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(semantic)


def cloud(*rows):
    record = np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("cos", "<f4"), ("idx", "<u4"), ("tag", "<u4"),
    ])
    return np.array(list(rows), dtype=record).tobytes()


class SemanticLidarTests(unittest.TestCase):
    def test_people_and_sky_are_dropped_and_untagged_walls_stay(self):
        raw = cloud(
            (1, 0, 1, 1, 0, 4),    # Walls
            (2, 0, 1, 1, 0, 12),   # Pedestrians
            (3, 0, 0, 1, 0, 0),    # None, this lab's untagged cubes
            (0, 0, 8, 1, 0, 11),   # Sky
            (4, 1, 0, 1, 0, 1),    # Roads
        )
        points, dropped = semantic.select_static_points(raw)
        self.assertEqual(dropped, 2)
        self.assertEqual(points.tolist(), [[1, 0, 1], [3, 0, 0], [4, 1, 0]])

    def test_empty_scan(self):
        points, dropped = semantic.select_static_points(b"")
        self.assertEqual((points.shape, dropped), ((0, 3), 0))

    def test_truncated_buffer_is_rejected(self):
        with self.assertRaises(ValueError):
            semantic.select_static_points(b"\x00" * 10)


if __name__ == "__main__":
    unittest.main()
