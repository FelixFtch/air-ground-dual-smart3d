"""Robot-specific temporary people OHZ, shared by SMART, guard and RViz.

Normalized centers are in robot-reference configuration space: body offset is
subtracted and footprint already included. Do not inflate these shapes again.
Shape kinds: "box" (default; oriented box of the measured person extent swept
along velocity) and "ellipse" (legacy affine ellipse/ellipsoid). Box
narrowphase uses exact slab clipping; ellipse minimizes affine segment
distance to the unit sphere.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

SCHEMA = "air_ground.people_hazard/v1"
CONTACT_TOLERANCE = 1e-6  # normalized squared distance; matches native f32 boundary
BOX_TOLERANCE = 1e-9  # metres; closed-box boundary for f32 planner states


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: finite number required")
    value = float(value)
    if not math.isfinite(value) or abs(value) > 1e7:
        raise ValueError(f"{name}: finite bounded number required")
    return value


def _vector(value, name):
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{name}: three coordinates required")
    return [_number(v, name) for v in value]


def _string(value, name):
    if not isinstance(value, str) or not value or len(value) > 256:
        raise ValueError(f"{name}: nonempty bounded string required")
    return value


def build_hazards(people_document, robot_profile, sim_time, epoch, scene_id):
    """Return fully specified, sorted ellipse/ellipsoid constraints.

    An absent document means no temporary observations. Invalid identities or
    malformed observations raise ValueError so owners can surface rejection.
    Expired people are omitted. Source positions are already projected to their
    declared position_reference_sim_time; only the remaining capped extrapolation
    is applied. Vertical speed is never invented or inferred from a filled zero.

    Policy defaults: 1 s prediction, .15 m XY uncertainty (uncalibrated budget),
    .85 m extra height at each end of the visible quantile interval. Height is a
    conservative profile allowance, not a SAM3 full-body measurement. Override
    through robot_profile['people_hazard']; zero padding explicitly disables it.
    """
    now = _number(sim_time, "sim_time")
    if people_document is None:
        return []
    if not isinstance(people_document, Mapping) or people_document.get("schema") != "air_ground.people/v1":
        raise ValueError("people schema mismatch")
    if people_document.get("scene_epoch") != epoch or people_document.get("scene_id") != scene_id:
        raise ValueError("people scene identity mismatch")
    _string(epoch, "scene_epoch")
    _string(scene_id, "scene_id")
    reference = _number(people_document.get("reference_sim_time"), "reference_sim_time")
    if reference > now + 1e-6:
        raise ValueError("people reference time in future")
    records = people_document.get("people")
    if not isinstance(records, list) or len(records) > 128:
        raise ValueError("people must be an array of at most 128 records")
    dimension = robot_profile.get("dimension")
    if dimension not in (2, 3):
        raise ValueError("robot dimension must be 2 or 3")
    profile_id = _string(robot_profile.get("profile_id"), "profile_id")
    half = _vector(robot_profile.get("body_half_extents_m"), "body_half_extents_m")
    offset = _vector(robot_profile.get("body_offset_m", [0, 0, 0]), "body_offset_m")
    if any(v <= 0 or v > 5 for v in half):
        raise ValueError("robot body half extents outside (0,5]")
    policy = robot_profile.get("people_hazard", {})
    if not isinstance(policy, Mapping):
        raise ValueError("people_hazard profile policy must be a mapping")
    horizon = _number(policy.get("horizon_s", 1.), "horizon_s")
    uncertainty = _number(policy.get("position_uncertainty_m", .15), "position_uncertainty_m")
    vertical_padding = _number(policy.get("vertical_padding_m", .85), "vertical_padding_m")
    if not 0 <= horizon <= 3 or not 0 <= uncertainty <= 2 or not 0 <= vertical_padding <= 3:
        raise ValueError("people hazard policy out of bounds")
    shape_kind = policy.get("shape_kind", "box")
    if shape_kind not in ("box", "ellipse"):
        raise ValueError("people hazard shape_kind must be box or ellipse")
    inflation = _number(policy.get("inflation_m",
        robot_profile.get("cluster_inflation_m", .35)), "inflation_m")
    if not 0 <= inflation <= 2:
        raise ValueError("people hazard inflation_m out of bounds")
    planar_z = _number(robot_profile.get("planar_z", 0.), "planar_z")
    seen, out = set(), []
    for record in records:
        if not isinstance(record, Mapping):
            raise ValueError("person must be a mapping")
        identifier = _string(record.get("global_id", record.get("id")), "person id")
        if identifier in seen:
            raise ValueError("duplicate global person id")
        seen.add(identifier)
        capture = _number(record.get("capture_sim_time"), "capture_sim_time")
        until = _number(record.get("valid_until_sim_time"), "valid_until_sim_time")
        position_time = _number(record.get("position_reference_sim_time", reference), "position_reference_sim_time")
        if capture > position_time + 1e-6 or position_time > reference + 1e-6 or until <= capture or until - capture > 10:
            raise ValueError("invalid people capture/reference/expiry times")
        if capture > now + 1e-6 or position_time > now + 1e-6:
            raise ValueError("person measurement in future")
        center = _vector(record.get("center_xyz"), "center_xyz")
        measured_half = _vector(record.get("half_extents_xyz"), "half_extents_xyz")
        if any(v < 0 or v > 5 for v in measured_half):
            raise ValueError("person visible half extents outside [0,5]")
        velocity = _vector(record.get("velocity_xyz"), "velocity_xyz")
        xy_valid = record.get("velocity_xy_valid")
        if not isinstance(xy_valid, bool) or record.get("velocity_z_valid") is not False:
            raise ValueError("explicit XY quality and unknown vertical velocity required")
        if velocity[2] != 0:
            raise ValueError("unknown vertical velocity must use zero placeholder")
        if math.hypot(*velocity[:2]) > 12:
            raise ValueError("person XY speed outside quality gate")
        source_keys = record.get("source_keys")
        if not isinstance(source_keys, list) or not source_keys or len(source_keys) > 32 or any(not isinstance(k, str) or not k or len(k) > 1024 for k in source_keys):
            raise ValueError("person source evidence required")
        quality = record.get("quality")
        if not isinstance(quality, Mapping):
            raise ValueError("person quality metadata required")
        if until <= now:
            continue
        # Global registry already projects centers. Additional travel never takes
        # them beyond capture+1 s, even if delayed readers keep the lease alive.
        extrapolation = min(max(now - position_time, 0.), max(capture + 1. - position_time, 0.)) if xy_valid else 0.
        measured_center = list(center)
        for i in range(2):
            center[i] += velocity[i] * extrapolation
        speed = math.hypot(*velocity[:2]) if xy_valid else 0.
        direction = [velocity[0] / speed, velocity[1] / speed, 0.] if speed > 1e-8 else [1., 0., 0.]
        lateral = [-direction[1], direction[0], 0.]
        travel = speed * horizon
        if shape_kind == "box":
            # The person's measured 3D box, uniformly inflated by the same
            # margin the planner dilates clustered voxels with, then swept
            # forward along velocity by travel/2 on each side of the shift.
            half_z = measured_half[2] + inflation + vertical_padding
            if dimension == 2 and abs(planar_z + offset[2] - center[2]) > half_z:
                continue
            if speed > 1e-8:
                support = sum(measured_half[i] * abs(direction[i]) for i in range(2))
                side = sum(measured_half[i] * abs(lateral[i]) for i in range(2))
                axes = [direction, lateral, [0., 0., 1.]]
            else:
                support, side = measured_half[0], measured_half[1]
                axes = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
            semi_axes = [support + travel * .5 + inflation, side + inflation,
                         half_z if dimension == 3 else 0.]
            hazard_center = [center[i] + direction[i] * travel * .5 - offset[i] for i in range(3)]
            if dimension == 2:
                # Z is display-only for this explicitly planar risk geometry.
                hazard_center[2] = planar_z
            shape_name, policy_tag = "box", "visible_extent_velocity_box/v1"
        else:
            axes = [direction, lateral, [0., 0., 1.]]
            half_z = measured_half[2] + vertical_padding + half[2]
            if dimension == 2 and abs(planar_z + offset[2] - center[2]) > half_z:
                continue
            # Minkowski sum of circumscribed XY disks includes each footprint once.
            rho = math.hypot(*measured_half[:2]) + math.hypot(*half[:2]) + uncertainty
            # A cylinder (rho, half_z) fits a sqrt(2)-scaled ellipsoid. Affine
            # normalization then gives the same conservative swept-ball ellipse:
            # a=1+h, b=sqrt(1+h); support gap = h*(1-|c|)^2 >= 0.
            base_xy = rho if dimension == 2 else math.sqrt(2.) * rho
            base_z = half_z if dimension == 2 else math.sqrt(2.) * half_z
            normalized_major = 1. + travel / (2. * base_xy)
            semi_axes = [base_xy * normalized_major, base_xy * math.sqrt(normalized_major), base_z * math.sqrt(normalized_major)]
            hazard_center = [center[i] + direction[i] * travel * .5 - offset[i] for i in range(3)]
            if dimension == 2:
                hazard_center[2] = planar_z
                semi_axes[2] = 0.
            shape_name = "ellipse" if dimension == 2 else "ellipsoid"
            policy_tag = "visible_extent_plus_profile_envelope_swept_ellipsoid/v1"
        out.append({
            "schema": SCHEMA, "id": identifier, "scene_epoch": epoch,
            "scene_id": scene_id, "profile_id": profile_id, "dimension": dimension,
            "shape": shape_name,
            "center_xyz": hazard_center, "axes_xyz": axes,
            "semi_axes_m": semi_axes, "footprint_included": True,
            "capture_sim_time": capture, "position_reference_sim_time": now,
            "valid_until_sim_time": until, "horizon_s": horizon,
            "velocity_xyz": [velocity[0], velocity[1], 0.] if xy_valid else [0., 0., 0.],
            "velocity_xy_valid": xy_valid, "velocity_z_valid": False,
            "source_keys": sorted(set(source_keys)),
            "geometry_policy": policy_tag,
            "measurement": {"center_xyz": measured_center, "half_extents_xyz": measured_half,
                            "position_reference_sim_time": position_time,
                            "extent_kind": quality.get("extent_kind", "visible_surface_quantiles")},
            "prior": {"vertical_padding_each_side_m": vertical_padding,
                      "human_height_prior_applied": vertical_padding > 0,
                      "position_uncertainty_m": uncertainty,
                      "inflation_m": inflation,
                      "calibration_status": "uncalibrated"},
            "robot_envelope": {"half_extents_xyz": half, "body_offset_m": offset},
        })
    return sorted(out, key=lambda h: h["id"])


def validate_hazard(hazard):
    """Validate the common geometry representation; return dimension and arrays."""
    if not isinstance(hazard, Mapping) or hazard.get("schema") != SCHEMA:
        raise ValueError("people hazard schema mismatch")
    n = hazard.get("dimension")
    allowed = ("ellipse", "box") if n == 2 else ("ellipsoid", "box")
    if n not in (2, 3) or hazard.get("shape") not in allowed:
        raise ValueError("hazard dimension/shape mismatch")
    if hazard.get("footprint_included") is not True:
        raise ValueError("hazard must include robot footprint")
    center = _vector(hazard.get("center_xyz"), "center_xyz")
    radii = _vector(hazard.get("semi_axes_m"), "semi_axes_m")
    axes = hazard.get("axes_xyz")
    if not isinstance(axes, list) or len(axes) != 3:
        raise ValueError("hazard axes required")
    axes = [_vector(v, "axes_xyz") for v in axes]
    if any(not 0 < x <= 100 for x in radii[:n]) or (n == 2 and radii[2] != 0):
        raise ValueError("hazard semi axes outside (0,100]")
    for i in range(3):
        for j in range(3):
            if abs(sum(axes[i][k] * axes[j][k] for k in range(3)) - (1 if i == j else 0)) > 1e-6:
                raise ValueError("hazard axes must be orthonormal")
    if any(abs(axes[i][2]) > 1e-6 for i in range(2)) or axes[2] != [0., 0., 1.]:
        raise ValueError("unknown vertical velocity requires horizontal heading")
    return n, center, radii, axes


def hazard_segment_intersects(start_xyz, end_xyz, hazard):
    """Exact closed segment / oriented box or affine ellipsoid test; no sampling."""
    n, center, radii, axes = validate_hazard(hazard)
    a, b = _vector(start_xyz, "start_xyz"), _vector(end_xyz, "end_xyz")
    if hazard.get("shape") == "box":
        # Slab clipping in the box frame: exact for closed boxes, and a
        # zero-length segment degenerates to the containment check.
        to_frame = lambda p: [sum((p[k] - center[k]) * axes[i][k] for k in range(3)) for i in range(n)]
        start, end = to_frame(a), to_frame(b)
        t0, t1 = 0., 1.
        for i in range(n):
            reach = radii[i] + BOX_TOLERANCE
            d = end[i] - start[i]
            if abs(d) < 1e-15:
                if abs(start[i]) > reach:
                    return False
                continue
            lo, hi = (-reach - start[i]) / d, (reach - start[i]) / d
            if lo > hi:
                lo, hi = hi, lo
            t0, t1 = max(t0, lo), min(t1, hi)
            if t0 > t1:
                return False
        return True
    transform = lambda p: [sum((p[k] - center[k]) * axes[i][k] for k in range(n)) / radii[i] for i in range(n)]
    start, end = transform(a), transform(b)
    direction = [end[i] - start[i] for i in range(n)]
    length_squared = sum(x * x for x in direction)
    t = max(0., min(1., -sum(start[i] * direction[i] for i in range(n)) / length_squared)) if length_squared > 0 else 0.
    return sum((start[i] + t * direction[i]) ** 2 for i in range(n)) <= 1. + CONTACT_TOLERANCE


def check_hazard_segment(start_xyz, end_xyz, hazards, sim_time=None):
    """Return the first blocking normalized hazard, or None. Invalid data raises.

    With sim_time, expired entries are excluded; future hazard references raise.
    Owners build once per current identity then use these same entries for all
    consumers. This function never grants a durable motion authorization.
    """
    now = None if sim_time is None else _number(sim_time, "sim_time")
    for hazard in hazards:
        if now is not None:
            if _number(hazard.get("position_reference_sim_time"), "position_reference_sim_time") > now + 1e-6:
                raise ValueError("hazard reference in future")
            if _number(hazard.get("valid_until_sim_time"), "valid_until_sim_time") <= now:
                continue
        if hazard_segment_intersects(start_xyz, end_xyz, hazard):
            return hazard
    return None
