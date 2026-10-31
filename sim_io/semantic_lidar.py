"""Town10 semantic-lidar selection for the shared voxel map.

CARLA labels match ``carla::rpc::CityObjectLabel`` in this UE 5.8 build.
Pedestrians and vehicles are omitted, the same set Town10 never voxelized.
Roads, walls, buildings and the other static classes accumulate.
Label 0 (None) is kept as well: this lab's walls and props are engine cubes,
so the tagger has no folder name and reports None. Dropping that label would
erase the scene. Sky is not a surface.
"""
from __future__ import annotations

import numpy as np

SUPPORT = frozenset({1, 2, 10, 24, 25, 26})  # Roads, Sidewalks, Terrain, RoadLines, Ground, Bridge
OBSTACLE = frozenset({0, 3, 4, 5, 6, 7, 8, 9, 20, 22, 28, 29})  # None plus Buildings..GuardRail, Rock
DYNAMIC = frozenset({12, 13, 14, 15, 16, 17, 18, 19, 21})  # Pedestrians through Bicycle, plus Dynamic
SKY = 11

_RECORD = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("cos", "<f4"), ("idx", "<u4"), ("tag", "<u4"),
])
_STATIC = np.array(sorted(SUPPORT | OBSTACLE), dtype=np.uint32)


def select_static_points(raw):
    """Sensor-frame XYZ of static returns, plus how many returns were dropped."""
    buffer = memoryview(raw).cast("B")
    if len(buffer) % _RECORD.itemsize:
        raise ValueError("semantic lidar measurement is truncated")
    cloud = np.frombuffer(buffer, dtype=_RECORD)
    if len(cloud) == 0:
        return np.zeros((0, 3)), 0
    tags = cloud["tag"]
    keep = np.isin(tags, _STATIC)
    dropped = int(len(tags) - int(keep.sum()))
    xyz = np.stack((cloud["x"], cloud["y"], cloud["z"]), axis=1)
    return np.ascontiguousarray(xyz[keep], dtype=float), dropped
