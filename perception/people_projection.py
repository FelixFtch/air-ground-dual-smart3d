"""Registered mask surfaces and robust per-camera, capture-time XY motion.

The measured bounds describe visible surface quantiles, never a full human body.
No simulator person actor metadata, position or velocity is consumed.
"""
from collections import OrderedDict, deque
import json
import math
import numpy as np

from .registered_rgbd import validate_cooperative_bodies

MAX_TRACKS = 32
MAX_SAMPLES = 2048
# At the measured ~1 Hz per camera, four observations span about three
# seconds. Keep bounded evidence long enough for that fit without extending
# active geometry TTL, allowed prediction, or consecutive-capture gap gates.
VELOCITY_WINDOW_SECONDS = 4.5
VELOCITY_MAX_GAP_SECONDS = 1.5
VELOCITY_HISTORY_SAMPLES = 6


def local_key(metadata, session, native_id):
    return json.dumps([metadata['scene_epoch'], metadata['robot_id'], metadata['camera_id'],
                       session, native_id], separators=(',', ':'))


def source_key(metadata):
    return (metadata['scene_epoch'], metadata['robot_id'], metadata['camera_id'])


class PeopleProjector:
    def __init__(self):
        self.history = OrderedDict()
        self.epoch = None

    def clear(self):
        self.history.clear()
        self.epoch = None

    def _velocity(self, key, stamp, xy):
        history = self.history.pop(key, deque(maxlen=VELOCITY_HISTORY_SAMPLES))
        reason = 'insufficient_history'
        if history:
            dt = stamp - history[-1][0]
            distance = float(np.linalg.norm(np.asarray(xy) - history[-1][1]))
            if dt <= 0 or dt > VELOCITY_MAX_GAP_SECONDS or distance > max(.6, 3. * dt):
                history.clear()
                reason = 'capture_gap_or_position_jump'
        history.append((stamp, np.asarray(xy)))
        while history and stamp - history[0][0] > VELOCITY_WINDOW_SECONDS:
            history.popleft()
        self.history[key] = history
        while len(self.history) > MAX_TRACKS:
            self.history.popitem(last=False)
        if len(history) < 4 or stamp - history[0][0] < .3:
            return None, False, reason, len(history)
        slopes = [(b[1] - a[1]) / (b[0] - a[0])
                  for i, a in enumerate(history) for b in list(history)[i+1:]
                  if b[0] - a[0] >= .2]
        if len(slopes) < 3:
            return None, False, 'insufficient_time_span', len(history)
        velocity = np.median(slopes, axis=0)
        residual = np.median(np.linalg.norm(np.asarray(slopes) - velocity, axis=1))
        if not np.isfinite(velocity).all() or np.linalg.norm(velocity) > 3. or residual > .6:
            return None, False, 'inconsistent_measured_motion', len(history)
        return velocity.tolist(), True, 'robust_same_source_capture_displacement', len(history)

    def project(self, metadata, result, xyz, stale=False):
        session = result['tracking_session_id']
        epoch = (source_key(metadata), metadata['calibration_id'], session)
        if epoch != self.epoch:
            self.clear()
            self.epoch = epoch
        stamp = metadata['capture_sim_time']
        output = dict(schema='air_ground.person_observations/v1', frame_id='map',
            scene_id=metadata['scene_id'], scene_epoch=metadata['scene_epoch'],
            robot_id=metadata['robot_id'], camera_id=metadata['camera_id'],
            calibration_id=metadata['calibration_id'], capture_sim_time=stamp,
            frame_token=metadata['frame_token'], tracking_session_id=session,
            stale=bool(stale), error=None, tracks=[], rejected=[],
            truncated=max(0, len(result['tracks']) - MAX_TRACKS))
        if stale or xyz is None:
            output['error'] = 'source_expired' if stale else 'registered_rgbd_unavailable'
            self.history.clear()
            return output
        height, width = metadata['height'], metadata['width']
        masks = result['masks']
        if (xyz.dtype != np.float32 or xyz.shape != (height, width, 3)
                or masks.shape != (len(result['tracks']), height, width)):
            raise ValueError('person_rgbd_mask_dimensions_mismatch')
        transform = np.asarray(metadata['camera_to_map'], dtype=float)
        if (transform.shape != (4, 4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6)
                or not np.allclose(transform[:3,:3].T @ transform[:3,:3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(transform[:3,:3]), 1., atol=1e-5)):
            raise ValueError('invalid_camera_to_map')
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
            raise ValueError('invalid_capture_time')
        rotation, translation = transform[:3, :3], transform[:3, 3]
        # Same-acquisition robot geometry is sensor self-filtering, not person
        # ground truth. Never exclude a person by distance to a latest robot pose.
        bodies = [(body['robot_id'], np.asarray(body['body_to_map'], dtype=float),
                   np.asarray(body['half_extents_xyz'], dtype=float),
                   np.asarray(body['body_offset_xyz'], dtype=float))
                  for body in validate_cooperative_bodies(metadata)]
        output['cooperative_body_filter'] = dict(available=bool(bodies),
            body_count=len(bodies), padding_m=.08, removed_samples=0)
        for old_key, points in list(self.history.items()):
            if stamp - points[-1][0] > VELOCITY_MAX_GAP_SECONDS or stamp < points[-1][0]:
                del self.history[old_key]
        seen = set()
        for track, raw_mask in zip(result['tracks'][:MAX_TRACKS], masks[:MAX_TRACKS]):
            track_id = track['id']
            if type(track_id) is not int or track_id < 0 or track_id in seen:
                output['rejected'].append(dict(track_id=str(track_id), reason='invalid_or_duplicate_native_id'))
                continue
            seen.add(track_id)
            key = local_key(metadata, session, track_id)
            mask = raw_mask.astype(bool)
            core = np.zeros_like(mask)
            core[1:-1, 1:-1] = (mask[1:-1, 1:-1] & mask[:-2, 1:-1]
                & mask[2:, 1:-1] & mask[1:-1, :-2] & mask[1:-1, 2:])
            indices = np.flatnonzero(core)
            if len(indices) > MAX_SAMPLES:
                indices = indices[np.linspace(0, len(indices)-1, MAX_SAMPLES).astype(int)]
            samples = xyz.reshape(-1, 3)[indices]
            valid = np.isfinite(samples).all(axis=1) & (samples[:, 2] >= .3) & (samples[:, 2] <= 15.)
            fraction = float(valid.mean()) if len(valid) else 0.
            samples = samples[valid]
            excluded_ids = []
            removed = 0
            if len(samples) and bodies:
                world_samples = samples @ rotation.T + translation
                excluded = np.zeros(len(samples), dtype=bool)
                for robot_id, body, half, offset in bodies:
                    local = (world_samples-body[:3, 3]) @ body[:3, :3] - offset
                    inside = np.all(np.abs(local) <= half + .08, axis=1)
                    if inside.any():
                        excluded_ids.append(robot_id)
                        excluded |= inside
                removed = int(excluded.sum())
                samples = samples[~excluded]
                output['cooperative_body_filter']['removed_samples'] += removed
            reason = None
            if len(samples) < 24 or fraction < .25:
                reason = 'cooperative_robot_surface' if removed and len(samples) < 24 else 'insufficient_mask_depth'
            else:
                depth = samples[:, 2]
                center_depth = np.median(depth)
                mad = float(np.median(np.abs(depth-center_depth)))
                inliers = np.abs(depth-center_depth) <= max(.18, min(.55, 3. * mad))
                support = float(inliers.mean())
                selected = samples[inliers]
                if len(selected) < 24 or support < .55:
                    reason = 'ambiguous_mask_depth'
                else:
                    world = selected @ rotation.T + translation
                    low, high = np.quantile(world, [.1, .9], axis=0)
                    if not np.isfinite(world).all() or float(np.linalg.norm(high[:2]-low[:2])) > 1.8:
                        reason = 'person_extent_too_large'
                    elif removed and high[2]-low[2] < .05:
                        # Removing a robot can leave only the floor visible
                        # through mask boundary pixels. Do not promote that
                        # planar remnant into a human with a height prior.
                        reason = 'cooperative_robot_surface'
            if reason:
                output['rejected'].append(dict(track_id=track_id, reason=reason,
                    excluded_robot_ids=excluded_ids, excluded_robot_samples=removed))
                self.history.pop(key, None)
                continue
            # Bounding-box center differs from the measured surface median; keep
            # both and use a consistent per-source median for displacement fits.
            position = np.median(world, axis=0)
            center = (low + high) / 2
            spread = float(np.quantile(selected[:, 2], .9) - np.quantile(selected[:, 2], .1))
            level = 'good' if len(selected) >= 80 and fraction >= .65 and support >= .75 else 'limited'
            velocity, velocity_valid, velocity_reason, count = self._velocity(key, stamp, position[:2])
            if level != 'good':
                velocity, velocity_valid, velocity_reason = None, False, 'limited_projection_quality'
                self.history.pop(key, None)
            output['tracks'].append(dict(track_id=track_id, native_track_id=track_id, track_key=key,
                source_key=key, camera_source_key=list(source_key(metadata)), label='person',
                tracking_session_id=session, score=float(track['score']),
                scene_id=metadata['scene_id'], scene_epoch=metadata['scene_epoch'],
                robot_id=metadata['robot_id'], camera_id=metadata['camera_id'],
                calibration_id=metadata['calibration_id'], frame_token=metadata['frame_token'],
                capture_sim_time=stamp, center_xyz=center.tolist(), position_map=position.tolist(),
                half_extents_xyz=((high-low)/2).tolist(), measured_min_xyz=low.tolist(),
                measured_max_xyz=high.tolist(), observation_samples=count,
                velocity_fit_window_seconds=VELOCITY_WINDOW_SECONDS,
                velocity_max_capture_gap_seconds=VELOCITY_MAX_GAP_SECONDS,
                quality=dict(level=level, depth_valid_fraction=fraction, inlier_fraction=support,
                    excluded_robot_ids=excluded_ids, excluded_robot_samples=removed,
                    sample_count=len(selected), depth_spread_m=spread,
                    position_uncertainty_m=max(.08, min(.5, spread/2)),
                    extent_kind='visible_surface_quantiles', human_height_prior_applied=False,
                    calibration_status='uncalibrated', surface_center_bias_unmodeled=True),
                velocity_xyz=velocity+[0.] if velocity_valid else [0., 0., 0.],
                velocity_xy_valid=velocity_valid, velocity_z_valid=False, velocity_reason=velocity_reason))
        return output
