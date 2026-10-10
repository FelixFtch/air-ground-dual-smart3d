"""Conservative cross-camera identity links with bounded capture-time validity.

No appearance ReID is claimed. New observations are merged only when the entire
candidate graph has one candidate on both sides. Close/occluded people remain
separate when spatial association is ambiguous.
"""
from collections import Counter
import copy
import math
import numpy as np


class PersonRegistry:
    def __init__(self, scene_id, scene_epoch, ttl_seconds=1.5, max_extrapolation_seconds=1.,
                 association_distance_m=.65, association_time_seconds=.75):
        self.scene_id, self.scene_epoch = scene_id, scene_epoch
        self.ttl = float(ttl_seconds)
        self.max_extrapolation = float(max_extrapolation_seconds)
        self.association_distance = float(association_distance_m)
        self.association_time = float(association_time_seconds)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.ttl, self.max_extrapolation, self.association_distance, self.association_time)):
            raise ValueError('invalid_registry_limits')
        self.groups = {}
        self.local_to_global = {}
        self.identity_evidence = {}
        self.counter = 0
        self.last_reference = None

    def clear(self):
        self.groups.clear()
        self.local_to_global.clear()
        self.identity_evidence.clear()

    @staticmethod
    def camera(obs):
        return (obs['scene_epoch'], obs['robot_id'], obs['camera_id'])

    def drop_source(self, source):
        source = tuple(source)
        for key, (camera, _) in list(self.identity_evidence.items()):
            if camera == source:
                self.identity_evidence.pop(key, None)
                self.local_to_global.pop(key, None)
        for gid, group in list(self.groups.items()):
            for key, obs in list(group.items()):
                if self.camera(obs) == source:
                    del group[key]
                    self.local_to_global.pop(key, None)
            if not group:
                del self.groups[gid]

    def _prune(self, reference):
        if not math.isfinite(reference) or reference < 0:
            raise ValueError('invalid_reference_time')
        if self.last_reference is not None and reference < self.last_reference - .001:
            self.clear()
        self.last_reference = reference
        for key, (_, stamp) in list(self.identity_evidence.items()):
            if reference >= stamp+self.ttl or stamp > reference+.1:
                self.identity_evidence.pop(key, None)
                self.local_to_global.pop(key, None)
        for gid, group in list(self.groups.items()):
            for key, obs in list(group.items()):
                stamp = obs['capture_sim_time']
                if reference >= stamp + self.ttl or stamp > reference + .1:
                    del group[key]
                    self.local_to_global.pop(key, None)
            if not group:
                del self.groups[gid]

    def _validate(self, obs, source, reference):
        if (obs.get('scene_id') != self.scene_id or obs.get('scene_epoch') != self.scene_epoch
                or self.camera(obs) != tuple(source)):
            raise ValueError('person_source_identity_mismatch')
        if (not isinstance(obs.get('source_key'), str) or not obs['source_key']
                or obs.get('velocity_z_valid') is not False
                or type(obs.get('velocity_xy_valid')) is not bool):
            raise ValueError('person_identity_or_velocity_flags_invalid')
        for field in ('center_xyz', 'half_extents_xyz', 'velocity_xyz'):
            vector = obs.get(field)
            if (not isinstance(vector, (list, tuple)) or len(vector) != 3
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in vector)):
                raise ValueError('invalid_person_' + field)
        if min(obs['half_extents_xyz']) < 0 or obs['velocity_xyz'][2] != 0:
            raise ValueError('invalid_person_extent_or_unmeasured_vertical_velocity')
        stamp = obs.get('capture_sim_time')
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
            raise ValueError('invalid_person_capture_time')
        return reference < stamp + self.ttl and stamp <= reference + .1

    def _position(self, obs, reference):
        center = np.asarray(obs['center_xyz'], dtype=float).copy()
        dt = max(0., min(self.max_extrapolation, reference - obs['capture_sim_time']))
        if obs['velocity_xy_valid']:
            center[:2] += np.asarray(obs['velocity_xyz'][:2]) * dt
        return center

    def _compatible(self, obs, group):
        # A camera can contribute at most one local track to a global person.
        if any(self.camera(other) == self.camera(obs) for other in group.values()):
            return False
        for other in group.values():
            if abs(obs['capture_sim_time'] - other['capture_sim_time']) > self.association_time:
                return False
            reference = max(obs['capture_sim_time'], other['capture_sim_time'])
            first, second = self._position(obs, reference), self._position(other, reference)
            if np.linalg.norm(first[:2]-second[:2]) > self.association_distance:
                return False
            # Views may observe different portions of the same body, but a
            # distant height slice is not silently joined to a nearby person.
            if abs(first[2]-second[2]) > .8:
                return False
            if obs['velocity_xy_valid'] and other['velocity_xy_valid']:
                if np.linalg.norm(np.asarray(obs['velocity_xyz'][:2])-other['velocity_xyz'][:2]) > .75:
                    return False
        return bool(group) and obs['quality']['level'] == 'good' and all(
            other['quality']['level'] == 'good' for other in group.values())

    def update(self, source, observations, reference_sim_time):
        self._prune(reference_sim_time)
        source = tuple(source)
        incoming = []
        keys = set()
        for raw in observations:
            if not self._validate(raw, source, reference_sim_time):
                continue
            if raw['source_key'] in keys:
                raise ValueError('duplicate_local_person_key')
            keys.add(raw['source_key'])
            incoming.append(copy.deepcopy(raw))
        # Latest accepted frame is negative evidence for absent local masks.
        # Sources awaiting a frame retain observations only until capture TTL.
        for gid, group in list(self.groups.items()):
            for key, obs in list(group.items()):
                if self.camera(obs) == source and key not in keys:
                    del group[key]
                    # Keep the native identity link within its capture TTL;
                    # negative evidence retires geometry immediately.
            if not group:
                del self.groups[gid]
        assignments, unmatched = {}, []
        for obs in incoming:
            key = obs['source_key']
            gid = self.local_to_global.get(key)
            if gid is not None and gid not in assignments.values():
                group = self.groups.setdefault(gid, {})
                previous = group.get(key)
                if previous is not None and obs['capture_sim_time'] <= previous['capture_sim_time']:
                    assignments[key] = gid
                    continue
                group[key] = obs
                self.identity_evidence[key] = (source, obs['capture_sim_time'])
                assignments[key] = gid
            else:
                unmatched.append(obs)
        candidates = {obs['source_key']: [gid for gid, group in self.groups.items()
                      if self._compatible(obs, group)] for obs in unmatched}
        counts = Counter(gid for values in candidates.values() for gid in values)
        for obs in unmatched:
            key = obs['source_key']
            eligible = candidates[key]
            if len(eligible) == 1 and counts[eligible[0]] == 1:
                gid = eligible[0]
                association = 'unambiguous_cross_camera_spatial'
            else:
                self.counter += 1
                gid = 'person-' + str(self.counter).zfill(6)
                self.groups[gid] = {}
                association = 'ambiguous_new_identity' if eligible else 'new_identity'
            obs['association'] = association
            self.groups[gid][key] = obs
            self.local_to_global[key] = gid
            self.identity_evidence[key] = (source, obs['capture_sim_time'])
            assignments[key] = gid
        return assignments

    def snapshot(self, reference_sim_time, generated_wall_time):
        self._prune(reference_sim_time)
        people = []
        for gid, group in sorted(self.groups.items()):
            observations = sorted(group.values(), key=lambda v: (-v['capture_sim_time'], v['source_key']))
            primary = observations[0]
            stamp = primary['capture_sim_time']
            position_time = min(reference_sim_time, stamp + self.max_extrapolation)
            centers = [self._position(obs, position_time) for obs in observations]
            lows = [center-np.asarray(obs['half_extents_xyz']) for center, obs in zip(centers, observations)]
            highs = [center+np.asarray(obs['half_extents_xyz']) for center, obs in zip(centers, observations)]
            low, high = np.min(lows, axis=0), np.max(highs, axis=0)
            velocities = [np.asarray(obs['velocity_xyz']) for obs in observations if obs['velocity_xy_valid']]
            valid = bool(velocities)
            if len(velocities) > 1 and any(np.linalg.norm(a[:2]-b[:2]) > .75
                    for i, a in enumerate(velocities) for b in velocities[i+1:]):
                valid = False
            velocity = np.mean(velocities, axis=0).tolist() if valid else [0., 0., 0.]
            # Mean velocities are fitted independently per source; alternating
            # camera surface centers never create extra displacement samples.
            quality = dict(primary['quality'],
                extent_kind='visible_surface_quantiles_union' if len(observations)>1 else 'visible_surface_quantiles',
                association='multi_source_spatial' if len(observations)>1 else primary.get('association','native_continuation'),
                source_count=len(observations), calibration_status='uncalibrated',
                velocity_reason='compatible_per_source_fits' if valid else 'insufficient_or_inconsistent_per_source_motion',
                bounded_extrapolation_seconds=max(0., position_time-stamp),
                maximum_extrapolation_seconds=self.max_extrapolation,
                human_height_prior_applied=False)
            people.append(dict(id=gid, global_id=gid, label='person',
                center_xyz=((low+high)/2).tolist(), half_extents_xyz=((high-low)/2).tolist(),
                measured_center_xyz=primary['center_xyz'], measured_half_extents_xyz=primary['half_extents_xyz'],
                velocity_xyz=velocity, velocity_xy_valid=valid, velocity_z_valid=False,
                capture_sim_time=stamp, position_reference_sim_time=position_time,
                valid_until_sim_time=min(obs['capture_sim_time']+self.ttl for obs in observations),
                source_keys=sorted(group), quality=quality,
                source_observations=observations))
        return dict(schema='air_ground.people/v1', scene_id=self.scene_id, scene_epoch=self.scene_epoch,
            epoch=self.scene_epoch, frame_id='map', reference_sim_time=reference_sim_time,
            generated_wall_time=generated_wall_time, people=people,
            truth_used_for_perception=False, prediction_calibration_status='uncalibrated')
