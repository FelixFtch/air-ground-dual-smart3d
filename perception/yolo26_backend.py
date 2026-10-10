"""YOLO26-seg people back end with map-frame identities; same result contract as the SAM3 back end.

Runs in the Python environment named in provenance/yolo26_people.json (ultralytics + CUDA torch).
Importing this module loads neither torch nor weights.
"""
from dataclasses import dataclass
import json
from pathlib import Path
import time
import uuid

import numpy as np

from .map_tracker import MapTracker, appearance
from .people_projection import PeopleProjector
from .sam3_backend import file_sha

ROOT = Path(__file__).resolve().parents[1]
PROVENANCE = ROOT / 'provenance/yolo26_people.json'
PERSON_CLASS = 0


@dataclass
class CameraSession:
    tracker: MapTracker | None
    session_id: str
    frame_index: int = 0
    reset_reason: str = 'service_started'
    botsort: object = None


class Yolo26People:
    """Per-frame YOLO26 person masks.

    ``association='map'`` assigns ids from depth-projected positions.
    ``association='botsort'`` uses Ultralytics BoT-SORT. Each camera keeps its own tracker:
    one shared tracker would treat Go2 and Bee as a single video.
    """
    wants_registered_frame = True

    def __init__(self, provenance=PROVENANCE, device=0, conf=.25, imgsz=640, association=None):
        record = json.loads(Path(provenance).read_text())
        weights = ROOT / record['path']
        if weights.stat().st_size != record['bytes'] or file_sha(weights) != record['sha256']:
            raise ValueError('yolo26_weights_hash_mismatch')
        from ultralytics import YOLO
        self.model = YOLO(str(weights))
        self.device, self.conf, self.imgsz = device, conf, imgsz
        self.model.predict(np.zeros((360, 640, 3), np.uint8), classes=[PERSON_CLASS], imgsz=imgsz,
                           device=device, verbose=False)
        import os
        if association is None:
            association = os.environ.get('YOLO26_ASSOCIATION', 'map')
        if association not in ('map', 'botsort'):
            raise ValueError('unknown_yolo26_association')
        self.association = association
        self.identity = {'name': 'ultralytics/' + record['model'], 'implementation': 'Ultralytics YOLO26 instance segmentation',
                         'weights_sha256': record['sha256'], 'mode': 'per_frame_detection', 'query': 'person',
                         'confidence': conf,
                         'tracking_ids': 'ultralytics_botsort' if association == 'botsort' else 'map_frame_association/v1'}
        self.sessions = {}

    def reset(self, source_key, reason='service_started'):
        tracker = None if getattr(self, 'association', 'map') == 'botsort' else MapTracker()
        self.sessions[source_key] = CameraSession(tracker, uuid.uuid4().hex, reset_reason=reason)
        return self.sessions[source_key]

    def drop(self, source_key):
        self.sessions.pop(source_key, None)

    def detect(self, rgb):
        """Boxes, scores and full-resolution person masks for one RGB image."""
        height, width = rgb.shape[:2]
        found = self.model.predict(np.ascontiguousarray(rgb[:, :, ::-1]), classes=[PERSON_CLASS], conf=self.conf,
                                   imgsz=self.imgsz, device=self.device, retina_masks=True, verbose=False)[0]
        if found.masks is None or not len(found.boxes):
            return [], [], np.zeros((0, height, width), bool)
        masks = found.masks.data.cpu().numpy() > .5
        if masks.shape[1:] != (height, width):
            import cv2
            masks = np.stack([cv2.resize(m.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST) > 0
                              for m in masks])
        return found.boxes.xyxy.cpu().tolist(), found.boxes.conf.cpu().tolist(), masks

    def _botsort(self, state):
        """One BoT-SORT per camera, installed onto the shared predictor for this call only."""
        if state.botsort is None:
            import sys
            deps = str(ROOT / 'runtime/python_deps')
            if deps not in sys.path:
                sys.path.insert(0, deps)
            from ultralytics.trackers.bot_sort import BOTSORT
            from ultralytics.utils import YAML, IterableSimpleNamespace
            from ultralytics.utils.checks import check_yaml
            cfg = IterableSimpleNamespace(**YAML.load(check_yaml('botsort.yaml')))
            state.botsort = BOTSORT(args=cfg, frame_rate=30)
        # track() only registers its callbacks when predictor.trackers does not exist yet.
        # Create that once, then swap in this camera's own tracker before every frame.
        if not hasattr(self.model.predictor, 'trackers'):
            self.model.track(np.zeros((360, 640, 3), np.uint8), persist=True, tracker='botsort.yaml',
                             classes=[PERSON_CLASS], conf=self.conf, imgsz=self.imgsz, device=self.device, verbose=False)
        self.model.predictor.trackers = [state.botsort]
        return state.botsort

    def _detect_botsort(self, state, rgb):
        height, width = rgb.shape[:2]
        empty = [], [], [], np.zeros((0, height, width), bool)
        self._botsort(state)
        found = self.model.track(np.ascontiguousarray(rgb[:, :, ::-1]), persist=True, tracker='botsort.yaml',
                                 classes=[PERSON_CLASS], conf=self.conf, imgsz=self.imgsz, device=self.device,
                                 retina_masks=True, verbose=False)[0]
        if found.masks is None or found.boxes is None or found.boxes.id is None or not len(found.boxes):
            return empty
        masks = found.masks.data.cpu().numpy() > .5
        if masks.shape[1:] != (height, width):
            import cv2
            masks = np.stack([cv2.resize(m.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST) > 0
                              for m in masks])
        ids = [int(i) for i in found.boxes.id.cpu().tolist()]
        return ids, found.boxes.xyxy.cpu().tolist(), found.boxes.conf.cpu().tolist(), masks

    def infer(self, source_key, image, frame):
        if getattr(self, 'association', 'map') == 'botsort':
            return self._infer_botsort(source_key, image, frame)
        return self._infer_map(source_key, image, frame)

    def _infer_botsort(self, source_key, image, frame):
        state = self.sessions.get(source_key) or self.reset(source_key)
        rgb = np.asarray(image)
        started = time.monotonic()
        ids, boxes, scores, masks = self._detect_botsort(state, rgb)
        result = dict(tracks=[dict(id=int(k), box=[float(v) for v in b], score=float(s))
                              for k, b, s in zip(ids, boxes, scores)],
                      masks=masks, inference_ms=(time.monotonic() - started) * 1000,
                      session_frame_index=state.frame_index, tracking_session_id=state.session_id,
                      reset_reason=state.reset_reason,
                      lifecycle=dict(visible_ids=sorted(set(ids)), association='botsort'))
        state.frame_index += 1
        return result

    def _infer_map(self, source_key, image, frame):
        state = self.sessions.get(source_key) or self.reset(source_key)
        rgb, meta = np.asarray(image), frame['meta']
        started = time.monotonic()
        boxes, scores, masks = self.detect(rgb)
        # Association needs each detection's map position; reuse the shared projection unchanged.
        candidates = dict(tracking_session_id='association', masks=masks,
                          tracks=[dict(id=i, box=b, score=s) for i, (b, s) in enumerate(zip(boxes, scores))])
        positions = {o['track_id']: o['position_map'][:2]
                     for o in PeopleProjector().project(meta, candidates, frame['xyz_optical'])['tracks']}
        ids = state.tracker.update(meta['capture_sim_time'],
                                   [(positions.get(i), appearance(rgb, masks[i])) for i in range(len(boxes))])
        result = dict(tracks=[dict(id=int(k), box=[float(v) for v in b], score=float(s))
                              for k, b, s in zip(ids, boxes, scores)],
                      masks=masks, inference_ms=(time.monotonic() - started) * 1000,
                      session_frame_index=state.frame_index, tracking_session_id=state.session_id,
                      reset_reason=state.reset_reason,
                      lifecycle=dict(visible_ids=sorted(ids), retained_tracks=len(state.tracker.tracks),
                                     lost_ttl_seconds=state.tracker.lost_ttl_s))
        state.frame_index += 1
        return result
