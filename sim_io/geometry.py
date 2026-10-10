"""One coordinate conversion boundary; every observation uses its capture pose."""
import math
import numpy as np


def to_map_xyz(carla_xyz, origin):
    a = np.asarray(carla_xyz, dtype=float).copy()
    a -= np.asarray(origin)
    a[..., 1] *= -1
    return a


def to_carla_xyz(map_xyz, origin):
    a = np.asarray(map_xyz, dtype=float).copy()
    a[..., 1] *= -1
    return a + np.asarray(origin)


def transform_points(points, matrix, origin):
    p = np.asarray(points, dtype=float)
    m = np.asarray(matrix, dtype=float)
    return to_map_xyz(p @ m[:3, :3].T + m[:3, 3], origin)


def depth_points(image, fov, stride=12, max_range=12):
    """Decode CARLA RGB-D, distinguishing actual hits and explicit free rays.

    CARLA's 1000 m far plane is an explicit simulation convention. A missing
    image or nonfinite/zero depth never constitutes free evidence.
    Returns points in CARLA sensor axes (+X optical forward, +Y right, +Z up).
    """
    b = np.frombuffer(image.raw_data, np.uint8).reshape(image.height, image.width, 4)
    v, u = np.mgrid[stride//2:image.height:stride, stride//2:image.width:stride]
    sampled = b[v, u].astype(np.float64)
    depth = (sampled[..., 2] + sampled[..., 1]*256 + sampled[..., 0]*65536) / 16777215 * 1000
    f = image.width / (2 * math.tan(math.radians(fov)/2))
    rays = np.stack((np.ones_like(u), (u+.5-image.width/2)/f, -(v+.5-image.height/2)/f), axis=-1)
    points = rays * depth[..., None]
    distance = np.linalg.norm(points, axis=-1)
    good = np.isfinite(distance) & (depth > .05)
    hit = good & (distance <= max_range)
    free = good & (distance > max_range)
    truncated = points[free] * (max_range / distance[free])[:, None]
    return points[hit], truncated, depth[free] >= 999


def wrap_angle(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi
