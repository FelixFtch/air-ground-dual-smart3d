"""Bounded egress from already intersecting occupancy, without changing a map.

The caller holds its map lock and retains sensor, revision, partner, and actual
physics checks. These functions never move a robot or consult simulator truth.
All coordinates are in the shared map frame. Body boxes match the guard's
all-yaw envelope; changing yaw never shrinks the collision footprint.
"""
from __future__ import annotations

from collections.abc import Mapping
from itertools import product
import math
from types import MappingProxyType

from .guard import (_EPS, _MAX_CANDIDATE_CELLS, _body_intersects_box,
                    _vector, check_path_segment)

_UNKNOWN = MappingProxyType({"state": "unknown", "last_observed_sim_time": None})
_MAX_ESCAPE_M = .6


class _InitialOccupiedAsUnknown(Mapping):
    """Read-only overlay: exemption is geometric, never fabricated free evidence."""
    def __init__(self, cells, keys):
        self._cells = cells
        self._keys = frozenset(keys)

    def __getitem__(self, key):
        return _UNKNOWN if key in self._keys else self._cells[key]

    def __iter__(self):
        return iter(self._cells)

    def __len__(self):
        return len(self._cells)


def _parameters(cells, resolution, start_xyz, half, offset, dimension, radius):
    if not isinstance(cells, Mapping):
        raise ValueError("cells must be a mapping")
    resolution, radius = float(resolution), float(radius)
    if not math.isfinite(resolution) or not .025 <= resolution <= 2:
        raise ValueError("invalid resolution")
    if not math.isfinite(radius) or not 0 < radius <= _MAX_ESCAPE_M:
        raise ValueError("recovery radius must be in (0, .6]")
    if isinstance(dimension, bool) or dimension not in (2, 3):
        raise ValueError("recovery dimension must be 2 or 3")
    position = _vector(start_xyz, "start_xyz")
    half, offset = _vector(half, "body_half_extents"), _vector(offset, "body_offset")
    if any(x <= 0 or x > 5 for x in half):
        raise ValueError("invalid body half extents")
    center = tuple(position[i] + offset[i] for i in range(3))
    return resolution, radius, position, center, half, offset


def _initial_contacts(cells, resolution, center, half):
    first = [math.floor((center[i]-half[i]-_EPS)/resolution) for i in range(3)]
    last = [math.floor((center[i]+half[i]+_EPS)/resolution) for i in range(3)]
    if math.prod(last[i]-first[i]+1 for i in range(3)) > _MAX_CANDIDATE_CELLS:
        raise ValueError("initial body exceeds cell budget")
    result = []
    for key in product(*(range(first[i], last[i]+1) for i in range(3))):
        value = cells.get(key)
        if value is None:
            continue
        if not isinstance(value, Mapping):
            raise ValueError("invalid cell")
        if value.get("state") != "occupied":
            continue
        lower = tuple(k*resolution for k in key)
        upper = tuple((k+1)*resolution for k in key)
        if _body_intersects_box(center, center, half, lower, upper):
            result.append((key, lower, upper))
    return result


def _monotone_exit(contacts, center, half, delta):
    """Every occupied intersection interval is nested for the whole straight move.

    For positive translation an interval's lower bound can only rise, but its
    upper bound must already be clipped by the voxel's upper face. Negative
    translation is the converse. This rules out crossing a thin wall and coming
    out the opposite side even when the final body is clear. It is conservative
    at tangent contacts: rejecting a recovery is preferable to adding contact.
    """
    for key, lower, upper in contacts:
        for axis in range(3):
            if delta[axis] > _EPS and center[axis]+half[axis] < upper[axis]-_EPS:
                return key
            if delta[axis] < -_EPS and center[axis]-half[axis] > lower[axis]+_EPS:
                return key
    return None


def check_recovery_segment(cells, resolution, start_xyz, end_xyz,
                           body_half_extents, body_offset, sim_time, *,
                           dimension=2, max_distance_m=.6, max_free_age=None,
                           bounds=None, require_clear_end=True):
    """Validate a straight, bounded egress; return a guard-style result.

    Only occupied cells intersecting the *actual initial body* get an unknown
    overlay. The normal guard still rejects newly entered parts of those same
    voxels, every other occupied cell, and unobserved new volume. No underlying
    dictionary or cell is mutated. Final targets must be wholly unoccupied.

    ``require_clear_end=False`` is for rechecking an immediate slow prefix of
    an already selected full escape. It never permits expanding old contact.
    Recheck against the current map before every physical control step.
    """
    contacts = []

    def result(ok, reason, key=None):
        return {"ok": ok, "reason": reason,
                "blocked_key": list(key) if key is not None else None,
                "initial_occupied_keys": [list(item[0]) for item in contacts],
                "method": "monotone_initial_overlap_egress"}

    try:
        resolution, radius, position, center, half, offset = _parameters(
            cells, resolution, start_xyz, body_half_extents, body_offset,
            dimension, max_distance_m)
        end = _vector(end_xyz, "end_xyz")
        delta = tuple(end[i]-position[i] for i in range(3))
        length = math.sqrt(sum(v*v for v in delta))
        if length <= _EPS or length > radius+_EPS:
            return result(False, "recovery_distance_out_of_bounds")
        if dimension == 2 and abs(delta[2]) > _EPS:
            return result(False, "recovery_not_planar")
        contacts = _initial_contacts(cells, resolution, center, half)
        if not contacts:
            return result(False, "no_initial_occupied_overlap")
        expanding = _monotone_exit(contacts, center, half, delta)
        if expanding is not None:
            return result(False, "occupied_overlap_expands", expanding)
        overlay = _InitialOccupiedAsUnknown(cells, (item[0] for item in contacts))
        normal = check_path_segment(overlay, resolution, position, end, half,
                                    offset, sim_time, max_free_age, bounds)
        if not normal["ok"]:
            return dict(normal, initial_occupied_keys=[list(item[0]) for item in contacts],
                        method="monotone_initial_overlap_egress")
        end_center = tuple(end[i]+offset[i] for i in range(3))
        if require_clear_end:
            for key, lower, upper in contacts:
                if _body_intersects_box(end_center, end_center, half, lower, upper):
                    return result(False, "end_still_occupied", key)
        return dict(result(True, "monotone_egress"),
                    checked_cells=normal["checked_cells"],
                    initial_volume_exempt_cells=normal["initial_volume_exempt_cells"])
    except (ValueError, TypeError, KeyError, OverflowError) as error:
        return dict(result(False, "invalid_input"), detail=str(error))


def find_recovery_target(cells, resolution, start_xyz, body_half_extents,
                         body_offset, sim_time, *, dimension=2,
                         max_distance_m=.6, max_free_age=None, bounds=None):
    """Return a safe short XYZ target or None from 8 XY / 26 XYZ directions.

    Each direction uses its shortest distance clearing all current occupied
    contacts plus a small positive margin. A longer move cannot fix a blocked
    prefix, so only one candidate per direction is necessary. Euclidean radius
    is capped at .6 m, including diagonal/vertical candidates. Shortest valid
    candidates are preferred; no goal, scene truth, or path teleport is used.
    """
    try:
        resolution, radius, position, center, half, offset = _parameters(
            cells, resolution, start_xyz, body_half_extents, body_offset,
            dimension, max_distance_m)
        contacts = _initial_contacts(cells, resolution, center, half)
        if not contacts:
            return None
        candidates = []
        for raw in product((-1, 0, 1), repeat=dimension):
            if not any(raw):
                continue
            length = math.sqrt(sum(v*v for v in raw))
            direction = tuple(v/length for v in raw) + ((0.,) if dimension == 2 else ())
            if _monotone_exit(contacts, center, half, direction) is not None:
                continue
            distance = 0.
            for _, lower, upper in contacts:
                exits = []
                for axis, value in enumerate(direction):
                    if value > 0:
                        exits.append((upper[axis]-(center[axis]-half[axis]))/value)
                    elif value < 0:
                        exits.append(((center[axis]+half[axis])-lower[axis])/(-value))
                distance = max(distance, min(exits))
            distance += max(1e-4, min(.01, resolution*.04))
            if distance <= radius:
                candidates.append((distance, tuple(position[i]+distance*direction[i] for i in range(3))))
        for _, end in sorted(candidates):
            verdict = check_recovery_segment(cells, resolution, position, end, half,
                offset, sim_time, dimension=dimension, max_distance_m=radius,
                max_free_age=max_free_age, bounds=bounds)
            if verdict["ok"]:
                return list(end)
    except (ValueError, TypeError, KeyError, OverflowError):
        pass
    return None
