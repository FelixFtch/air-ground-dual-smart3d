"""Serial, fair latest-frame people service for an isolated air/ground run.

One back end per run, never both: ``--backend sam3`` (SAM3 Python environment) or
``--backend yolo26`` (ultralytics environment, see provenance/yolo26_people.json). Either one
writes shared/people.json for the runtime's OHZ. This service has no motion or simulator API access.
"""
import argparse
from collections import deque
import fcntl
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import uuid

from .people_projection import PeopleProjector
from .person_registry import PersonRegistry
from .sam3_backend import IDENTITY, VideoModel

ROOT = Path(__file__).resolve().parents[1]
BACKENDS = ('sam3', 'yolo26')
COLORS = ((70, 230, 130), (70, 170, 255), (255, 190, 60), (230, 100, 255),
          (255, 90, 100), (50, 225, 225), (195, 230, 80))


def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_bytes(data)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path, value):
    atomic_write(path, (json.dumps(value, allow_nan=False, separators=(',', ':'))+'\n').encode())


class FrameSequence:
    """Consecutive model index is distinct from skipped camera frame sequence."""
    def __init__(self, mono=time.monotonic):
        self.last = None
        self.mono = mono
        self.unavailable_since = None

    def unavailable(self):
        if self.unavailable_since is None:
            self.unavailable_since = self.mono()

    def accept(self, metadata):
        if self.last and metadata['frame_token'] == self.last['frame_token']:
            if metadata.get('rgb_sha256') != self.last.get('rgb_sha256'):
                raise ValueError('same_frame_token_different_image')
            return False, None
        reason = None
        if self.last:
            if metadata['scene_epoch'] != self.last['scene_epoch']:
                reason = 'scene_epoch_changed'
            elif metadata['calibration_id'] != self.last['calibration_id']:
                reason = 'camera_calibration_changed'
            elif (metadata['capture_sim_time'] < self.last['capture_sim_time']
                  or metadata['frame_seq'] < self.last['frame_seq']):
                reason = 'source_clock_restarted'
            elif (metadata['frame_seq'] == self.last['frame_seq']
                  or metadata['capture_sim_time'] == self.last['capture_sim_time']):
                raise ValueError('same_capture_identity_different_token')
            elif self.unavailable_since is not None and self.mono()-self.unavailable_since > 5.:
                reason = 'camera_stream_interrupted'
        self.unavailable_since = None
        self.last = dict(metadata)
        return True, reason


class LatestFrameScheduler:
    """One newest pending manifest per source with non-starving round robin."""
    def __init__(self, sources):
        self.order = deque(sources)
        self.pending = {}
        self.seen = {}
        self.replaced = {key: 0 for key in sources}

    def offer(self, key, token, value):
        if self.seen.get(key) == token:
            return False
        self.seen[key] = token
        if key in self.pending:
            self.replaced[key] += 1
        self.pending[key] = value
        return True

    def pop(self):
        for _ in range(len(self.order)):
            key = self.order[0]
            self.order.rotate(-1)
            if key in self.pending:
                return key, self.pending.pop(key)
        return None


def overlay(rgb, tracks, masks, global_ids, rejections=()):
    import numpy as np
    from PIL import Image, ImageDraw
    height, width = rgb.shape[:2]
    if masks.shape != (len(tracks), height, width):
        raise ValueError('tracking_mask_dimensions_mismatch')
    pixels = np.array(rgb, copy=True)
    for track, mask in zip(tracks, masks):
        color = np.array(COLORS[track['id'] % len(COLORS)])
        selected = mask.astype(bool)
        pixels[selected] = (pixels[selected]*.55 + color*.45).astype(np.uint8)
    result = Image.fromarray(pixels)
    draw = ImageDraw.Draw(result)
    for track in tracks:
        box = track['box']
        color = COLORS[track['id'] % len(COLORS)]
        draw.rectangle(box, outline=color, width=2)
        excluded = any(r.get('track_id') == track['id'] and r.get('reason') == 'cooperative_robot_surface' for r in rejections)
        label = ('robot surface excluded' if excluded else global_ids.get(track['id'], 'unprojected')) + ' / local ' + str(track['id'])
        x, y = max(0, box[0]), max(0, box[1]-16)
        draw.rectangle((x, y, min(width, x+len(label)*7+6), y+15), fill=(15,25,30))
        draw.text((x+3,y+2),label,fill=color)
    stream = io.BytesIO()
    result.save(stream, format='JPEG', quality=90)
    return stream.getvalue()


class PeopleService:
    def __init__(self, run_root, backend, ttl_seconds=1.5, wall=time.time, mono=time.monotonic, name='sam3'):
        if name not in BACKENDS:
            raise ValueError('unknown_people_backend')
        self.root = Path(run_root).resolve()
        self.backend, self.wall, self.mono, self.name = backend, wall, mono, name
        self.identity = getattr(backend, 'identity', IDENTITY)
        profiles = json.loads((self.root/'resolved_profiles.json').read_text())
        state = self.read_state()
        self.scene_id, self.epoch = state['scene_id'], state['epoch']
        self.sources = [(self.epoch, r['robot_id'], 'front_rgbd') for r in profiles['robots']]
        self.registry = PersonRegistry(self.scene_id, self.epoch, ttl_seconds=ttl_seconds)
        self.scheduler = LatestFrameScheduler(self.sources)
        self.sequences = {key: FrameSequence(mono) for key in self.sources}
        self.projectors = {key: PeopleProjector() for key in self.sources}
        self.status = {key: dict(schema=f'air_ground.{name}_source/v1', state='waiting', valid=False,
            stale=True, scene_id=self.scene_id, scene_epoch=self.epoch, robot_id=key[1], camera_id=key[2],
            frame_token=None, image_sha256=None, overlay_sha256=None, tracks_count=0, processed_frames=0,
            model_identity=self.identity, error='awaiting_registered_rgbd') for key in self.sources}
        self.last_publish_mono = -math.inf

    def read_state(self):
        value = json.loads((self.root/'state.json').read_text())
        sim = value.get('sim_time')
        if type(sim) not in (int,float) or not math.isfinite(sim) or sim < 0:
            raise ValueError('invalid_run_sim_time')
        if type(value.get('wall_time')) not in (int,float) or not math.isfinite(value['wall_time']):
            raise ValueError('invalid_run_wall_time')
        try:
            clock = json.loads((self.root/'shared/sensor_clock.json').read_text())
            if (clock.get('scene_id') == value.get('scene_id')
                    and clock.get('scene_epoch') == value.get('epoch')
                    and type(clock.get('sim_time')) in (int,float) and math.isfinite(clock['sim_time'])
                    and type(clock.get('wall_time')) in (int,float) and math.isfinite(clock['wall_time'])
                    and -.2 <= self.wall()-clock['wall_time'] <= 1.5
                    and clock['sim_time'] >= value['sim_time']):
                value = dict(value, sim_time=clock['sim_time'], wall_time=clock['wall_time'],
                             reference_clock='sensor_clock')
        except (OSError, ValueError, TypeError):
            pass
        if not -.2 <= self.wall()-value['wall_time'] <= 3.:
            raise ValueError('run_state_not_fresh')
        if not value.get('running', False):
            raise ValueError('run_not_running')
        return value

    def _source_state(self, key, state, error=None, preserve_history=False):
        status = self.status[key]
        status.update(state=state, status=state, valid=False, stale=True, error=error,
                      generated_wall_time=self.wall())
        if not preserve_history:
            self.registry.drop_source(key)
        status.update(tracks_count=0, tracks=[], global_ids={})
        if 'observations' in status:
            status['observations'] = dict(status['observations'], tracks=[], rejected=[], stale=True, error=error or state)
        self.sequences[key].unavailable()
        self.scheduler.pending.pop(key, None)
        atomic_json(self.root/key[1]/self.name/'latest.json', status)

    def set_state(self, state, error=None):
        for key in self.sources:
            self._source_state(key, state, error)

    def scan(self, state):
        reference = state['sim_time']
        for key in self.sources:
            manifest = self.root/key[1]/'rgbd/latest.json'
            try:
                data = json.loads(manifest.read_text())
                if (data.get('scene_id') != self.scene_id or data.get('scene_epoch') != self.epoch
                        or data.get('robot_id') != key[1] or data.get('camera_id') != key[2]):
                    raise ValueError('manifest_source_identity_mismatch')
                stamp = data['capture_sim_time']
                received = data['receive_wall_time']
                if (type(stamp) not in (int,float) or not math.isfinite(stamp)
                        or type(received) not in (int,float) or not math.isfinite(received)):
                    raise ValueError('invalid_manifest_timestamps')
                delta = reference-stamp
                if -1. <= delta < 0:
                    # Telemetry can trail a genuine sensor callback. Wait for
                    # an authoritative clock tick; never erase valid history.
                    self.status[key]['input_state'] = 'awaiting_reference_clock'
                    self.scheduler.pending.pop(key, None)
                    continue
                if (not 0 <= delta < self.registry.ttl or not -.2 <= self.wall()-received <= 3.):
                    self._source_state(key, 'expired', 'capture_expired_before_inference', preserve_history=True)
                    continue
                self.status[key]['input_state'] = 'available'
                self.scheduler.offer(key, data['frame_token'], (manifest, self.mono()))
            except (OSError, KeyError, ValueError, TypeError) as exc:
                self._source_state(key, 'unavailable', type(exc).__name__+': '+str(exc)[:180])

    def publish_shared(self, state, service_state='running', error=None):
        value = self.registry.snapshot(state['sim_time'], self.wall())
        value.update(state=service_state, error=error, perception_backend=self.name,
            sources=[dict(status, latest_replacements=self.scheduler.replaced[key]) for key,status in self.status.items()],
            policy=dict(capture_ttl_seconds=self.registry.ttl,
                max_extrapolation_seconds=self.registry.max_extrapolation,
                association='unambiguous_cross_camera_spatial_no_appearance_reid',
                per_camera_identity=self.identity.get('tracking_ids'),
                velocity='independent_per_source_minimum_four_capture_measurements'))
        atomic_json(self.root/'shared/people.json', value)
        atomic_json(self.root/f'shared/{self.name}_status.json', dict(schema=f'air_ground.{self.name}_service/v1',
            state=service_state, error=error, scene_epoch=self.epoch, scene_id=self.scene_id,
            generated_wall_time=self.wall(), reference_sim_time=state['sim_time'],
            sources=[{k:v for k,v in status.items() if k not in ('tracks','observations')}
                     for status in self.status.values()], model_identity=self.identity,
            shared_model_instances=1, camera_sessions=len(getattr(self.backend,'sessions',{}))))
        self.last_publish_mono = self.mono()
        return value

    def step(self):
        from .registered_rgbd import load_capture
        from PIL import Image
        state = self.read_state()
        if state['scene_id'] != self.scene_id or state['epoch'] != self.epoch:
            raise ValueError('run_identity_changed_restart_service')
        self.scan(state)
        selected = self.scheduler.pop()
        if selected is None:
            if self.mono()-self.last_publish_mono > .2:
                self.publish_shared(state)
            return False
        key, (manifest, queued) = selected
        try:
            # Reader loads an immutable, hash-verified bundle into RAM. It is
            # safe if the producer rotates the on-disk ring during inference.
            frame = load_capture(manifest, expected={'scene_id':self.scene_id, 'scene_epoch':self.epoch,
                'robot_id':key[1], 'camera_id':key[2]})
            meta = frame['meta']
            if (meta['scene_id'] != self.scene_id or meta['scene_epoch'] != self.epoch
                    or meta['robot_id'] != key[1] or meta['camera_id'] != key[2]):
                raise ValueError('bundle_source_identity_mismatch')
            before = self.read_state()
            if -1. <= before['sim_time']-meta['capture_sim_time'] < 0:
                self.scheduler.seen.pop(key, None)
                self.status[key]['input_state'] = 'awaiting_reference_clock'
                return False
            if not 0 <= before['sim_time']-meta['capture_sim_time'] < self.registry.ttl:
                self._source_state(key, 'expired', 'capture_expired_before_inference', preserve_history=True)
                self.publish_shared(before)
                return False
            accepted, reset = self.sequences[key].accept(meta)
            if not accepted:
                return False
            if reset:
                self.backend.reset(key, reset)
                self.registry.drop_source(key)
                self.projectors[key].clear()
            started = self.mono()
            if getattr(self.backend, 'wants_registered_frame', False):
                result = self.backend.infer(key, Image.fromarray(frame['rgb']), frame)
            else:
                result = self.backend.infer(key, Image.fromarray(frame['rgb']))
            after = self.read_state()
            if after['scene_id'] != self.scene_id or after['epoch'] != self.epoch:
                raise ValueError('run_identity_changed_during_inference')
            stale = not 0 <= after['sim_time']-meta['capture_sim_time'] < self.registry.ttl
            observations = self.projectors[key].project(meta, result, frame['xyz_optical'], stale=stale)
            assignments = self.registry.update(key, observations['tracks'], after['sim_time'])
            globals_by_native = {obs['track_id']: assignments[obs['source_key']]
                for obs in observations['tracks'] if obs['source_key'] in assignments}
            jpeg = overlay(frame['rgb'], result['tracks'], result['masks'], globals_by_native, observations['rejected'])
            depth_rejections = [r for r in observations['rejected'] if r['reason'] != 'cooperative_robot_surface']
            state_name = ('expired' if stale else 'partial_depth' if depth_rejections else
                          'ready' if observations['tracks'] else 'ready_no_people')
            status = dict(self.status[key],
                **{name:item for name,item in result.items() if name != 'masks'})
            status.update(state=state_name, status=state_name, valid=not stale, stale=stale, error=observations['error'],
                overlay_sha256=hashlib.sha256(jpeg).hexdigest(), tracks_count=len(result['tracks']),
                frame_token=meta['frame_token'], source_frame_token=meta['frame_token'],
                source_rgb_sha256=meta.get('rgb_sha256'), source_bundle_path=meta.get('bundle_path'),
                source_payload_sha256=meta.get('payload_sha256'), image_sha256=hashlib.sha256(jpeg).hexdigest(),
                frame_seq=meta['frame_seq'], calibration_id=meta['calibration_id'],
                width=meta['width'],height=meta['height'],capture_sim_time=meta['capture_sim_time'],
                valid_until_sim_time=meta['capture_sim_time']+self.registry.ttl,
                receive_wall_time=meta['receive_wall_time'],generated_wall_time=self.wall(),
                queue_wait_ms=(started-queued)*1000, inference_total_ms=(self.mono()-started)*1000,
                capture_age_sim_seconds=after['sim_time']-meta['capture_sim_time'],
                processed_frames=self.status[key]['processed_frames']+1,
                global_ids={str(k):v for k,v in globals_by_native.items()}, observations=observations)
            self.status[key] = status
            # Image first, status last: readers reject any hash mismatch across
            # these two atomic replacements. A failed frame never rebrands it.
            atomic_write(self.root/key[1]/self.name/'latest.jpg', jpeg)
            atomic_json(self.root/key[1]/self.name/'latest.json', status)
            self.publish_shared(after)
            return True
        except Exception as exc:
            self._source_state(key, 'failed', type(exc).__name__+': '+str(exc)[:240])
            self.backend.drop(key)
            self.sequences[key] = FrameSequence(self.mono)
            self.publish_shared(self.read_state())
            return False


def make_backend(name):
    if name == 'yolo26':
        from .yolo26_backend import Yolo26People
        return Yolo26People()
    return VideoModel()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=BACKENDS, default='sam3', help='people back end; one per run')
    parser.add_argument('--run-root', type=Path)
    parser.add_argument('--max-hz', type=float, default=4., help='maximum total model frame rate across sources')
    parser.add_argument('--ttl-seconds',type=float,default=1.5)
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_hz) or args.max_hz <= 0:
        parser.error('--max-hz must be positive')
    run_root = args.run_root
    if run_root is None:
        run_root = Path(json.loads(((Path(os.environ['AIR_GROUND_RUNTIME']) if os.environ.get('AIR_GROUND_RUNTIME') else ROOT/'runtime')/'latest_run.json').read_text())['run_root'])
    run_root = run_root.resolve()
    (run_root/'shared').mkdir(parents=True,exist_ok=True)
    stop = threading.Event()
    for number in (signal.SIGTERM, signal.SIGINT):
        signal.signal(number, lambda *_: stop.set())
    status_path = run_root/f'shared/{args.backend}_status.json'
    schema = f'air_ground.{args.backend}_service/v1'
    # Both back ends write shared/people.json, so one lock admits a single people service per run.
    with open(run_root/'shared/people.worker.lock','a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        lock.seek(0);lock.truncate();lock.write(f'{os.getpid()} {args.backend}');lock.flush()
        atomic_json(status_path,dict(schema=schema,state='loading',generated_wall_time=time.time(),backend=args.backend))
        service = None
        try:
            backend = make_backend(args.backend)
            service = PeopleService(run_root, backend, ttl_seconds=args.ttl_seconds, name=args.backend)
            while not stop.is_set():
                started = time.monotonic()
                worked = service.step()
                stop.wait(max(.02, 1./args.max_hz-(time.monotonic()-started)) if worked else .05)
            service.set_state('stopped','signal_received')
            # Retire constraints even if the producer has already stopped.
            state = json.loads((run_root/'state.json').read_text())
            service.publish_shared(state, 'stopped', 'signal_received')
            return 0
        except Exception as exc:
            if service:
                service.set_state('failed', type(exc).__name__+': '+str(exc)[:240])
                state = json.loads((run_root/'state.json').read_text())
                service.publish_shared(state, 'failed', str(exc)[:240])
            else:
                atomic_json(status_path,dict(schema=schema,state='failed',generated_wall_time=time.time(),
                    backend=args.backend,error=type(exc).__name__+': '+str(exc)[:240]))
            raise


if __name__ == '__main__':
    raise SystemExit(main())
