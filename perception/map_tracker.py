"""Map-frame identities for per-frame person detections (used by the YOLO26 back end).

Image-box trackers lose people at ~3 Hz once the camera moves: boxes stop overlapping between
frames. Each detection here already has a depth-projected map XY, so association uses a
constant-velocity prediction and a walking-speed gate. After a gap longer than ``short_gap_s``
a returning person must also match on clothing colour before the old identity is reused.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment

HUE_BINS, SATURATION_BINS, VALUE_BINS = 18, 8, 8
_UNASSIGNABLE = 1e6


def appearance(rgb, mask):
    """Normalised colour histogram of the masked pixels; None when too few pixels remain."""
    import cv2
    selected = np.asarray(mask, dtype=bool)
    if selected.sum() < 40:
        return None
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)[selected].astype(int)
    hue, saturation, value = hsv[:, 0], hsv[:, 1], hsv[:, 2]
    # Hue is meaningless for grey, black and white clothing; describe those by brightness.
    chromatic = (saturation >= 40) & (value >= 30)
    colour = np.bincount((hue[chromatic] * HUE_BINS // 180) * SATURATION_BINS
                         + saturation[chromatic] * SATURATION_BINS // 256,
                         minlength=HUE_BINS * SATURATION_BINS)
    grey = np.bincount(value[~chromatic] * VALUE_BINS // 256, minlength=VALUE_BINS)
    histogram = np.concatenate((colour, grey)).astype(float)
    return histogram / histogram.sum()


def similarity(a, b):
    """Bhattacharyya coefficient in [0, 1]; None when either side has no appearance."""
    if a is None or b is None:
        return None
    return float(np.sqrt(a * b).sum())


class MapTracker:
    # Defaults chosen on 100 s of empty-scene RGB-D scored against CARLA walkers (2026-10-04):
    # 2.4 identities per person per camera, the same as SAM3's 30 s video memory.
    def __init__(self, noise_m=.5, max_speed_mps=1.5, max_gate_m=5., lost_ttl_s=20., short_gap_s=1.5,
                 extrapolate_s=3., min_reid_similarity=.6, appearance_weight=.5):
        self.noise_m, self.max_speed_mps, self.max_gate_m = noise_m, max_speed_mps, max_gate_m
        self.lost_ttl_s, self.short_gap_s, self.extrapolate_s = lost_ttl_s, short_gap_s, extrapolate_s
        self.min_reid_similarity, self.appearance_weight = min_reid_similarity, appearance_weight
        self.tracks = {}
        self.next_id = 0

    def _cost(self, stamp, xy, histogram, track):
        dt = max(0., stamp - track['t'])
        predicted = track['xy'] + track['vel'] * min(dt, self.extrapolate_s)
        gate = min(self.noise_m + self.max_speed_mps * dt, self.max_gate_m)
        distance = float(np.linalg.norm(xy - predicted))
        if distance > gate:
            return None
        match = similarity(histogram, track['appearance'])
        if dt > self.short_gap_s and (match is None or match < self.min_reid_similarity):
            return None
        return distance / gate + self.appearance_weight * (1. - (.5 if match is None else match))

    def update(self, stamp, detections):
        """``detections``: (map_xy or None, appearance or None) per detection. Returns one id each.

        A detection without a map position gets a fresh id and is not tracked.
        """
        self.tracks = {k: v for k, v in self.tracks.items() if 0 <= stamp - v['t'] <= self.lost_ttl_s}
        ids = [None] * len(detections)
        usable = [i for i, (xy, _) in enumerate(detections) if xy is not None]
        keys = list(self.tracks)
        if usable and keys:
            cost = np.full((len(usable), len(keys)), _UNASSIGNABLE)
            for row, i in enumerate(usable):
                xy, histogram = np.asarray(detections[i][0], float), detections[i][1]
                for column, key in enumerate(keys):
                    value = self._cost(stamp, xy, histogram, self.tracks[key])
                    if value is not None:
                        cost[row, column] = value
            for row, column in zip(*linear_sum_assignment(cost)):
                if cost[row, column] < _UNASSIGNABLE:
                    ids[usable[row]] = keys[column]
        for i, (xy, histogram) in enumerate(detections):
            if ids[i] is None:
                ids[i] = self.next_id
                self.next_id += 1
                if xy is not None:
                    self.tracks[ids[i]] = dict(xy=np.asarray(xy, float), t=stamp, vel=np.zeros(2), appearance=histogram)
                continue
            track = self.tracks[ids[i]]
            xy = np.asarray(xy, float)
            dt = stamp - track['t']
            track['vel'] = (.5 * track['vel'] + .5 * (xy - track['xy']) / dt) if 0 < dt <= self.short_gap_s else np.zeros(2)
            track.update(xy=xy, t=stamp)
            if histogram is not None:
                blended = histogram if track['appearance'] is None else .8 * track['appearance'] + .2 * histogram
                track['appearance'] = blended / blended.sum()
        return ids
