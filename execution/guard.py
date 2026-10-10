"""Conservative near-step execution guard; pure geometry, no simulator imports.

All vectors are in the shared map frame, metres and simulation seconds. This is
an exact continuous swept AABB check, stronger than sampling every resolution/2.
It does not replace robot/session/revision or live-partner checks in the owner.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

_EPS = 1e-8
_MAX_CANDIDATE_CELLS = 250_000


def _vector(value, name):
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != 3:
        raise ValueError(f"{name} must have three coordinates")
    result = tuple(float(x) for x in value)
    if any(not math.isfinite(x) or abs(x) > 1e7 for x in result):
        raise ValueError(f"{name} must be finite and bounded")
    return result


def _segment_box(a, b, lower, upper):
    """Closed segment/AABB slabs. Includes tangent contact and zero length."""
    enter, leave = 0.0, 1.0
    for axis in range(3):
        step = b[axis] - a[axis]
        if abs(step) < 1e-12:
            if a[axis] < lower[axis] - _EPS or a[axis] > upper[axis] + _EPS:
                return False
        else:
            first = (lower[axis] - a[axis]) / step
            last = (upper[axis] - a[axis]) / step
            if first > last:
                first, last = last, first
            enter = max(enter, first)
            leave = min(leave, last)
            if enter > leave + _EPS:
                return False
    return True


def _body_intersects_box(start, end, half, lower, upper):
    return _segment_box(start, end,
                        tuple(lower[i] - half[i] for i in range(3)),
                        tuple(upper[i] + half[i] for i in range(3)))


def _enters_outside_initial_body(start, end, half, lower, upper):
    """Whether the swept body reaches this cell outside its initial body AABB.

    Subtract the actual initial body geometrically. Merely intersecting an
    initial voxel is insufficient to exempt the rest of that voxel: moving into
    an unknown portion of that same cell must still be blocked.
    """
    for axis in range(3):
        # Voxel intersection with each of six half spaces outside initial body.
        cut_upper = min(upper[axis], start[axis] - half[axis] - 4 * _EPS)
        if lower[axis] <= cut_upper:
            region_upper = list(upper)
            region_upper[axis] = cut_upper
            if _body_intersects_box(start, end, half, lower, region_upper):
                return True
        cut_lower = max(lower[axis], start[axis] + half[axis] + 4 * _EPS)
        if cut_lower <= upper[axis]:
            region_lower = list(lower)
            region_lower[axis] = cut_lower
            if _body_intersects_box(start, end, half, region_lower, upper):
                return True
    return False


def check_path_segment(cells, resolution, start_xyz, end_xyz,
                       body_half_extents, body_offset, sim_time,
                       max_free_age=3.0, bounds=None):
    """Check the complete swept body; return {ok, reason, blocked_key, ...}.

    ``cells`` maps signed floor(x/resolution) tuple keys to a mapping with
    ``state`` and ``last_observed_sim_time``. Known occupied always blocks,
    including inside the current body. Missing/unknown/stale free blocks only
    newly entered physical volume. Fresh evidence is inclusive at max_free_age.
    Explicit max_free_age=None retains observed free cells until their state
    changes; finite observation time is still required for newly entered cells.
    The runtime must independently keep live sensors and map revisions current.
    Bounds are {"min":[x,y,z], "max":[x,y,z]} (or [min,max]) and constrain the
    whole translated body. This function issues a check result only, not a
    durable motion token. Call it again immediately before each short step.
    """
    examined = 0
    skipped_initial = 0

    def result(ok, reason, key=None):
        return {"ok": ok, "reason": reason,
                "blocked_key": list(key) if key is not None else None,
                "checked_cells": examined,
                "initial_volume_exempt_cells": skipped_initial,
                "method": "continuous_swept_aabb"}

    try:
        if not isinstance(cells, Mapping):
            raise ValueError("cells must be a mapping")
        resolution = float(resolution)
        sim_time = float(sim_time)
        max_free_age = None if max_free_age is None else float(max_free_age)
        if not math.isfinite(resolution) or not 0.025 <= resolution <= 2:
            raise ValueError("resolution outside [0.025,2]")
        if not math.isfinite(sim_time) or (max_free_age is not None and
                (not math.isfinite(max_free_age) or max_free_age < 0)):
            raise ValueError("invalid simulation time or max_free_age")
        start = _vector(start_xyz, "start_xyz")
        end = _vector(end_xyz, "end_xyz")
        half = _vector(body_half_extents, "body_half_extents")
        offset = _vector(body_offset, "body_offset")
        if any(x <= 0 or x > 5 for x in half):
            raise ValueError("body half extents outside (0,5]")
        start = tuple(start[i] + offset[i] for i in range(3))
        end = tuple(end[i] + offset[i] for i in range(3))
        if bounds is not None:
            if isinstance(bounds, Mapping):
                lower, upper = bounds["min"], bounds["max"]
            else:
                lower, upper = bounds
            lower, upper = _vector(lower, "bounds.min"), _vector(upper, "bounds.max")
            if any(lower[i] >= upper[i] for i in range(3)):
                raise ValueError("empty operating bounds")
            if any(center[i] - half[i] < lower[i] - _EPS or
                   center[i] + half[i] > upper[i] + _EPS
                   for center in (start, end) for i in range(3)):
                return result(False, "body_out_of_bounds")
        first = tuple(math.floor((min(start[i], end[i]) - half[i] - _EPS) / resolution) for i in range(3))
        last = tuple(math.floor((max(start[i], end[i]) + half[i] + _EPS) / resolution) for i in range(3))
        count = math.prod(last[i] - first[i] + 1 for i in range(3))
        if count > _MAX_CANDIDATE_CELLS:
            return result(False, "segment_too_large")

        swept_cells = []
        for x in range(first[0], last[0] + 1):
            for y in range(first[1], last[1] + 1):
                for z in range(first[2], last[2] + 1):
                    key = (x, y, z)
                    lower = tuple(k * resolution for k in key)
                    upper = tuple((k + 1) * resolution for k in key)
                    if not _body_intersects_box(start, end, half, lower, upper):
                        continue
                    examined += 1
                    cell = cells.get(key)
                    if cell is not None and not isinstance(cell, Mapping):
                        return result(False, "invalid_cell", key)
                    if cell is not None and cell.get("state") == "occupied":
                        return result(False, "occupied", key)
                    swept_cells.append((key, lower, upper, cell))

        # Occupied is prioritized over missing evidence, even if it is farther
        # along the segment. Neither permits a partial unchecked movement.
        for key, lower, upper, cell in swept_cells:
            state = cell.get("state") if cell is not None else "unknown"
            if state not in ("unknown", "free", None):
                return result(False, "invalid_cell_state", key)
            if not _enters_outside_initial_body(start, end, half, lower, upper):
                skipped_initial += 1
                continue
            if state != "free":
                return result(False, "unknown", key)
            stamp = cell.get("last_observed_sim_time")
            if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                return result(False, "free_time_missing", key)
            if stamp > sim_time + _EPS:
                return result(False, "free_time_in_future", key)
            if max_free_age is not None and sim_time - stamp > max_free_age + _EPS:
                return result(False, "stale_free", key)
        return result(True, "fresh_free_swept_volume")
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        return dict(result(False, "invalid_input"), detail=str(exc))


def bounded_path_target(path_xyz, current_xyz, max_distance_m=0.25):
    """Return a near target on/approaching a path, at most max_distance_m away.

    Project to the closest polyline segment, then advance with only the
    remaining distance after the current-to-projection connector. The resulting
    straight execution chord must still pass check_path_segment. This helper
    performs no collision check and gives no execution authorization.
    """
    current = _vector(current_xyz, "current_xyz")
    points = [_vector(p, "path point") for p in path_xyz]
    max_distance_m = float(max_distance_m)
    if not points or not math.isfinite(max_distance_m) or max_distance_m <= 0:
        raise ValueError("nonempty path and positive finite distance required")

    def distance(a, b):
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def toward(a, b, length):
        d = distance(a, b)
        factor = min(1.0, length / d) if d else 0.0
        return tuple(a[i] + factor * (b[i] - a[i]) for i in range(3))

    best = (distance(current, points[0]), points[0], 0)
    for i, (a, b) in enumerate(zip(points, points[1:])):
        v = tuple(b[j] - a[j] for j in range(3))
        square = sum(x * x for x in v)
        fraction = max(0.0, min(1.0, sum((current[j] - a[j]) * v[j] for j in range(3)) / square)) if square else 0.0
        projection = tuple(a[j] + fraction * v[j] for j in range(3))
        candidate = (distance(current, projection), projection, i + 1)
        if candidate[0] < best[0] - _EPS or abs(candidate[0] - best[0]) <= _EPS and candidate[2] > best[2]:
            best = candidate
    gap, target, next_index = best
    if gap >= max_distance_m:
        return list(toward(current, target, max_distance_m))
    remaining = max_distance_m - gap
    for point in points[next_index:]:
        step = distance(target, point)
        if step > remaining:
            return list(toward(target, point, remaining))
        target = point
        remaining -= step
    return list(target)
