"""Prepare real LiDAR returns for independent marking and clearing distances.

All coordinates use one capture-time map frame. Returned free endpoints must be
sent to OctoMap with ``include_endpoint=False``: free evidence ends before a real
return, never at its hit voxel. The separate far-hit evidence can refresh or
restore previously marked surfaces, but cannot first mark an unseen far cell.
The structured frame additionally protects all real far-hit voxels from *other*
rays in that frame. Cropped virtual endpoints do not qualify for protection.
Missing returns do not produce rays. This module never mutates a map.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PreparedLidarFrame:
    hit_points: np.ndarray
    free_endpoints: np.ndarray
    protected_hit_points: np.ndarray
    stats: dict



def observed_xy_azimuth_span_deg(points):
    """Smallest circular XY arc containing valid measured return directions.

    Points must be relative to the LiDAR origin (normally sensor-local XYZ).
    Only finite, nonzero XY vectors contribute; Z is irrelevant. The result is
    360 degrees minus the largest circular angular gap. This measures return
    support, NOT the emitted scan angle: no-return directions are unrepresented.
    Empty/single-direction input returns zero; an almost full circle remains
    slightly below 360 degrees because discrete returns still have angular gaps.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.size == 0:
        return 0.0
    if xyz.ndim != 2 or xyz.shape[1] < 2:
        raise ValueError('LiDAR points require an Nx2 or wider array')
    xy = xyz[:, :2]
    valid = np.isfinite(xy).all(axis=1) & np.any(xy != 0., axis=1)
    xy = xy[valid]
    if len(xy) <= 1:
        return 0.0
    angles = np.sort(np.degrees(np.arctan2(xy[:, 1], xy[:, 0])))
    gaps = np.diff(angles, append=angles[0] + 360.)
    return float(np.clip(360. - gaps.max(), 0., 360.))


def _voxel_keys(points, resolution):
    keys_float = np.floor(points / resolution)
    if not np.isfinite(keys_float).all() or np.any(np.abs(keys_float) >= 2**62):
        raise ValueError('voxel coordinate is out of range')
    return keys_float.astype(np.int64)


def _unique_endpoints(points):
    """Drop only identical rays; shared endpoint voxels need not share a path."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)
    _, first = np.unique(points, axis=0, return_index=True)
    return np.ascontiguousarray(points[np.sort(first)], dtype=np.float64)


def _unique_voxel_representatives(points, origin, resolution):
    """Keep the nearest *real* endpoint in each voxel; do not use voxel centers."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)
    keys = _voxel_keys(points, resolution)
    distance2 = np.einsum('ij,ij->i', points - origin, points - origin)
    order = np.argsort(distance2, kind='stable')
    _, first = np.unique(keys[order], axis=0, return_index=True)
    return np.ascontiguousarray(points[order[first]], dtype=np.float64)


def prepare_lidar_frame(points, origin, resolution, mark_range_m=9.0,
                        clear_range_m=40.0, bounds=None):
    """Return explicit near hits, free rays, and protected real far hits.

    Send ``protected_hit_points`` and ``mark_range_m`` to the mapper with the
    free rays. Protection excludes these surfaces from the entire frame's free
    evidence. They can restore previously marked cells, but cannot first mark
    unseen cells outside marking range.

    ``bounds`` is a 3-by-2 array of [minimum, maximum] map intervals, with each
    maximum excluded, matching the map service. ``origin`` must be inside it.
    All valid returns within marking distance and bounds contribute a hit voxel.
    Other returns contribute only their measured segment, clipped to clearing
    distance and the map boundary. Only identical endpoints are deduplicated:
    different returns in one endpoint voxel can cross different intermediate
    voxels. The mapper deduplicates hit/free voxel evidence after tracing every
    retained ray. Protection needs only one real representative per hit voxel.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.size == 0:
        xyz = xyz.reshape((0, 3))
    o = np.asarray(origin, dtype=np.float64)
    limits = np.asarray(bounds, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
        raise ValueError('LiDAR points must be finite Nx3 actual returns')
    if o.shape != (3,) or not np.isfinite(o).all():
        raise ValueError('LiDAR origin must be finite XYZ')
    if limits.shape != (3, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError('map bounds must be finite increasing XYZ intervals')
    resolution, mark_range_m, clear_range_m = map(float, (resolution, mark_range_m, clear_range_m))
    if not all(np.isfinite(x) for x in (resolution, mark_range_m, clear_range_m)) or not (resolution > 0 and 0 < mark_range_m <= clear_range_m):
        raise ValueError('positive finite resolution and 0 < mark range <= clear range required')
    lower, upper = limits[:, 0], limits[:, 1]
    if not np.all((o >= lower) & (o < upper)):
        raise ValueError('LiDAR origin must be inside map bounds')

    with np.errstate(over='ignore', invalid='ignore'):
        vectors = xyz - o
        distance = np.linalg.norm(vectors, axis=1)
    if not np.isfinite(distance).all():
        raise ValueError('LiDAR distances must be finite')
    nonzero = distance > 0
    inside = ((xyz >= lower) & (xyz < upper)).all(axis=1)
    mark = nonzero & inside & (distance <= mark_range_m)
    hits = _unique_endpoints(xyz[mark])
    # Choose from original measured endpoints *before* any clipping/dedup of
    # free rays. Excluding only each ray's own endpoint is insufficient: another
    # real ray can cross the same discretized voxel on its way to a farther hit.
    protected = nonzero & inside & (distance > mark_range_m) & (distance <= clear_range_m)
    protected_hits = _unique_voxel_representatives(xyz[protected], o, resolution)

    # This is segment clipping from an interior origin, not fabricated max-range
    # observations. The scale never exceeds 1, hence never extends a real ray.
    selected = nonzero & ~mark
    v = vectors[selected]
    d = distance[selected]
    t = np.minimum(1., clear_range_m / d)
    with np.errstate(divide='ignore', invalid='ignore'):
        exit_parameters = np.where(v > 0, (upper - o) / v,
                                   np.where(v < 0, (lower - o) / v, np.inf))
    exit_t = exit_parameters.min(axis=1)
    boundary_clipped = exit_t <= t
    # Stay just inside the boundary. A small geometric epsilon also survives the
    # mapper's float32 point representation; it is far below the voxel size.
    margin_m = max(resolution * 1e-5, np.max(np.abs(limits)) * np.finfo(np.float32).eps * 4)
    boundary_t = np.maximum(0., exit_t - margin_m / d)
    t = np.where(boundary_clipped, boundary_t, t)
    endpoints = o + v * t[:, None]
    valid = (t > 0) & ((endpoints >= lower) & (endpoints < upper)).all(axis=1)
    free = _unique_endpoints(endpoints[valid])
    stats = dict(ray_geometry_policy='all_distinct_measured_rays',
                 input_points=int(len(xyz)), zero_distance_points=int((~nonzero).sum()),
                 in_range_hit_points=int(mark.sum()),
                 unique_hit_voxels=int(len(np.unique(_voxel_keys(hits, resolution), axis=0))),
                 measured_free_rays=int(selected.sum()),
                 unique_free_endpoint_voxels=int(len(np.unique(_voxel_keys(free, resolution), axis=0))),
                 hit_ray_count=int(len(hits)), free_ray_count=int(len(free)),
                 total_ray_count=int(len(hits) + len(free)),
                 boundary_clipped_rays=int(boundary_clipped.sum()),
                 range_clipped_rays=int((d > clear_range_m).sum()),
                 discarded_zero_length_rays=int((~valid).sum()),
                 mark_range_m=mark_range_m, clear_range_m=clear_range_m,
                 real_far_hit_points=int(protected.sum()),
                 unique_protected_hit_voxels=int(len(protected_hits)),
                 free_include_endpoint=False)
    return PreparedLidarFrame(hits, free, protected_hits, stats)


def prepare_lidar_rays(points, origin, resolution, mark_range_m=9.0,
                       clear_range_m=40.0, bounds=None):
    """Compatibility tuple; use prepare_lidar_frame for complete map fusion.

    This historical tuple cannot transport cross-ray far-hit protection. New
    integrations must send the structured frame's protected_hit_points too.
    """
    frame = prepare_lidar_frame(points, origin, resolution, mark_range_m,
                                clear_range_m, bounds)
    return frame.hit_points, frame.free_endpoints, frame.stats
