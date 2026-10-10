"""Bounded, forward-only SAM3 Video history for the reviewed Transformers 5.17.0.

The caller keeps one streaming session, explicitly supplies consecutive frame_idx
starting at zero, uses only the text prompt 'person', and calls after_frame after
postprocessing each completed frame. No reset/reindex/reverse/revisit is supported.
Keep the checkpoint's detector-correction policy (every 16 frames, using detector
masks). Bound conditioning context to the four frames selected by the pinned
tracker for the next forward frame. This preserves its feature-memory selection;
it deliberately bounds the older conditioning object pointers too. It is not
numerically equivalent to upstream's unbounded all-conditioning pointer history.
"""
from __future__ import annotations

import hashlib
import inspect
import math
from pathlib import Path

REVIEWED_SOURCES = {
    'transformers.models.sam3_video.modeling_sam3_video':
        'a5c190d52ef87cd42fe69d08dbfddfead8f50ad36c2203aeaf759bc7e254f846',
    'transformers.models.sam3_tracker_video.modeling_sam3_tracker_video':
        '0881053eaa71c935f72ba7ddf11290f85d8845e69da0669aa277beb17bca30d1',
}
HISTORY = 128
MAX_OBJECTS = 16
LOST_TRACK_TTL_SECONDS = 30.
RECONDITION_EVERY = 16
CONDITIONING_FRAMES = 4


def _verify_sources(model):
    for value in (model, model.tracker_model):
        cls = type(value)
        expected = REVIEWED_SOURCES.get(cls.__module__)
        source = inspect.getsourcefile(cls)
        if not expected or not source or hashlib.sha256(Path(source).read_bytes()).hexdigest() != expected:
            raise ValueError('SAM3 bounded memory requires the exact reviewed Transformers 5.17.0 sources')


class CurrentFrameTimeline(dict):
    """One retained frame tensor; len is the logical timeline consumed by SAM3.

    This is intentionally a pinned SAM3 frame-store adapter, not a general mapping.
    The reviewed code uses only indexed assignment/read and len(processed_frames).
    Its nested tracker receives frame_idx without frame, so logical len is needed
    to preserve min(num_frames, max_object_pointers_in_encoder) exactly.
    """
    def __init__(self):
        super().__init__()
        self.total_frames = 0

    def __len__(self):
        return self.total_frames

    def __setitem__(self, frame_idx, frame):
        if type(frame_idx) is not int or frame_idx != self.total_frames:
            raise ValueError('SAM3 streaming frame indices must start at zero and increase by one')
        dict.clear(self)
        dict.__setitem__(self, frame_idx, frame)
        self.total_frames += 1

    @property
    def retained_frames(self):
        return dict.__len__(self)


def _trim_frame_dict(values, cutoff):
    for frame_idx in list(values):
        if frame_idx < cutoff:
            del values[frame_idx]


class BoundedVideoMemory:
    def __init__(self, model, session):
        _verify_sources(model)
        cfg = model.tracker_model.config
        if (model.training or model.tracker_model.training
                or model.recondition_every_nth_frame != RECONDITION_EVERY
                or model.recondition_on_trk_masks is not False
                or model.max_num_objects != MAX_OBJECTS
                or cfg.num_maskmem != 7 or cfg.max_object_pointers_in_encoder != 16
                or cfg.max_cond_frame_num != 4):
            raise ValueError('SAM3 bounded memory requires eval, detector correction every16, max_objects16 and reviewed tracker horizons')
        if session.processed_frames is not None or session.obj_ids or session.output_dict_per_obj:
            raise ValueError('Install bounded SAM3 history on a fresh streaming session only')
        if session.max_vision_features_cache_size != 1:
            raise ValueError('SAM3 vision cache must retain exactly one frame')
        self.session = session
        self.select_conditioning = model.tracker_model._select_closest_cond_frames
        self.timeline = CurrentFrameTimeline()
        self.last_completed_frame = -1
        session.processed_frames = self.timeline

    def after_frame(self, frame_idx):
        """Keep native-selected finite conditioning context after each output.

        Retained tensors, IDs, prompts, birth indices, keep-alive and occlusion
        state are unchanged. Feature-memory selection matches native forward
        selection. Old conditioning object pointers are deliberately discarded
        under the explicit finite-context policy; they otherwise grow forever.
        Validate all objects before pruning any of their state.
        """
        s = self.session
        if (type(frame_idx) is not int or frame_idx != self.last_completed_frame + 1
                or len(self.timeline) != frame_idx + 1 or s.processed_frames is not self.timeline
                or self.timeline.retained_frames != 1 or frame_idx not in self.timeline):
            raise ValueError('SAM3 bounded cleanup must follow exactly one new completed frame')
        if len(s.obj_ids) > MAX_OBJECTS or s.get_obj_num() > MAX_OBJECTS or s.output_buffer:
            raise ValueError('SAM3 streaming object/buffer contract changed')
        expected_indices = set(range(s.get_obj_num()))
        if set(s.output_dict_per_obj) != expected_indices:
            raise ValueError('SAM3 per-object state mapping changed')
        selected_by_object = {}
        retired_conditioning = 0
        for obj_idx, outputs in s.output_dict_per_obj.items():
            cond = outputs['cond_frame_outputs']
            if not 1 <= len(cond) <= CONDITIONING_FRAMES + 1 or any(type(idx) is not int or not 0 <= idx <= frame_idx for idx in cond):
                raise ValueError('SAM3 conditioning context must contain retained past frames plus at most one new frame')
            if any(type(idx) is not int or not 0 <= idx <= frame_idx for idx in outputs['non_cond_frame_outputs']):
                raise ValueError('SAM3 only supports completed past memory frames')
            # Use the exact upstream selector, including its dictionary order.
            # All frames are in the past: discarded older frames cannot become
            # a closest-four selection in a future strictly forward inference.
            selected, unselected = self.select_conditioning(frame_idx + 1, cond, CONDITIONING_FRAMES)
            if (not selected or len(selected) > CONDITIONING_FRAMES
                    or set(selected) & set(unselected) or set(selected) | set(unselected) != set(cond)
                    or any(selected[idx] is not cond[idx] for idx in selected)
                    or min(selected) < max(unselected, default=-1)):
                raise ValueError('Pinned SAM3 conditioning selection contract changed')
            # Native feature memory can fall back to an unselected conditioning
            # frame in its recent six-frame window. Detector correction every16
            # guarantees retired frames are older; assert instead of approximating.
            if any(idx >= frame_idx + 1 - 6 for idx in unselected):
                raise ValueError('Cannot retire a conditioning frame in native recent-memory window')
            selected_by_object[obj_idx] = dict(selected)
            retired_conditioning += len(unselected)
        cutoff = max(0, frame_idx - HISTORY + 1)
        for obj_idx, outputs in s.output_dict_per_obj.items():
            outputs['cond_frame_outputs'].clear()
            outputs['cond_frame_outputs'].update(selected_by_object[obj_idx])
            _trim_frame_dict(outputs['non_cond_frame_outputs'], cutoff)
        for per_obj in (s.frames_tracked_per_obj, s.mask_inputs_per_obj, s.point_inputs_per_obj):
            for obj_idx, frames in per_obj.items():
                # Preserve inputs for retained conditioning frames only.
                preserve = s.output_dict_per_obj[obj_idx]['cond_frame_outputs']
                for idx in list(frames):
                    if idx < cutoff and idx not in preserve:
                        del frames[idx]
        for values in (s.obj_id_to_tracker_score_frame_wise, s.suppressed_obj_ids):
            _trim_frame_dict(values, cutoff)
        # These lists are only counted by disabled, future-frame hotstart removal.
        # Keep recent diagnostics; preserve keep-alive counters and birth indices.
        for values in (s.unmatched_frame_inds, s.overlap_pair_to_frame_inds):
            for key, frames in values.items():
                values[key] = [idx for idx in frames if idx >= cutoff]
        self.last_completed_frame = frame_idx
        return dict(total_frames=len(self.timeline), retained_frames=self.timeline.retained_frames,
                    tracked_objects=len(s.obj_ids), history_frames=HISTORY,
                    conditioning_frame_limit_per_object=CONDITIONING_FRAMES,
                    retired_conditioning_this_frame=retired_conditioning,
                    conditioning_policy='native_next_forward_closest_four_finite_pointer_context',
                    conditioning_frames=sum(len(v['cond_frame_outputs']) for v in s.output_dict_per_obj.values()),
                    nonconditioning_frames=sum(len(v['non_cond_frame_outputs']) for v in s.output_dict_per_obj.values()))


def configure_streaming_model(model):
    """Apply the explicit public continuous-tracking policy before a new session."""
    _verify_sources(model)
    model.config.recondition_every_nth_frame = RECONDITION_EVERY
    model.config.recondition_on_trk_masks = False
    model.config.max_num_objects = MAX_OBJECTS
    model.recondition_every_nth_frame = RECONDITION_EVERY
    model.recondition_on_trk_masks = False
    model.max_num_objects = MAX_OBJECTS
    # Native streaming keeps hotstart removal disabled. Empty masks must still
    # consume the native keep-alive budget before our conservative TTL can expire.
    model.config.decrease_trk_keep_alive_for_empty_masklets = True
    model.decrease_trk_keep_alive_for_empty_masklets = True
    return dict(recondition_every_nth_frame=RECONDITION_EVERY, recondition_on_trk_masks=False,
                max_num_objects=MAX_OBJECTS,
                decrease_trk_keep_alive_for_empty_masklets=True,
                lost_track_ttl_seconds=LOST_TRACK_TTL_SECONDS,
                retained_nonconditioning_frames=HISTORY,
                retained_conditioning_frames_per_object=CONDITIONING_FRAMES,
                conditioning_policy='native_next_forward_closest_four_finite_pointer_context',
                upstream_all_conditioning_pointer_equivalence=False,
                frame_storage='current_tensor_with_logical_total_count',
                direction='forward_only', resets_for_memory=False)


def configure_streaming_session(session, model):
    memory = BoundedVideoMemory(model, session)
    session._factory_bounded_memory = memory
    session._factory_lost_tracks = LostTrackLifecycle(session)
    return memory


def trim_streaming_session(session, current_idx, config):
    if (config.recondition_every_nth_frame != RECONDITION_EVERY
            or config.recondition_on_trk_masks is not False or config.max_num_objects != MAX_OBJECTS
            or config.tracker_config.num_maskmem != 7
            or config.tracker_config.max_object_pointers_in_encoder != 16
            or config.tracker_config.max_cond_frame_num != 4):
        raise ValueError('SAM3 bounded-memory configuration changed during streaming')
    memory = getattr(session, '_factory_bounded_memory', None)
    if not isinstance(memory, BoundedVideoMemory):
        raise ValueError('Configure the fresh SAM3 streaming session before inference')
    return memory.after_frame(current_idx)


def _clean_retired_metadata(session, retired):
    """Public remove_object remaps tensor slots but leaves these ID-keyed maps."""
    for name in ('obj_id_to_score', 'obj_id_to_last_occluded', 'obj_first_frame_idx',
                 'unmatched_frame_inds', 'trk_keep_alive', 'obj_id_to_prompt_id'):
        values = getattr(session, name)
        for obj_id in retired:
            values.pop(obj_id, None)
    for scores in session.obj_id_to_tracker_score_frame_wise.values():
        for obj_id in retired:
            scores.pop(obj_id, None)
    for pair in list(session.overlap_pair_to_frame_inds):
        if retired.intersection(pair):
            del session.overlap_pair_to_frame_inds[pair]
    for suppressed in session.suppressed_obj_ids.values():
        suppressed.difference_update(retired)
    session.removed_obj_ids.difference_update(retired)
    session.hotstart_removed_obj_ids.difference_update(retired)
    session.obj_with_new_inputs = [obj_id for obj_id in session.obj_with_new_inputs if obj_id not in retired]


class LostTrackLifecycle:
    """Expire only invisible, exhausted native tracks; never reset a session.

    Visibility comes from final rendered nonempty masks, not ground truth, class
    scores or a new detector. A returning person after expiry receives a new native
    ID. A lone dormant track remains because upstream removes the final object by
    resetting session state. It is eligible again once another native track exists.
    """
    def __init__(self, session):
        self.session = session
        self.last_visible_or_birth = {}
        self.last_frame = -1
        self.last_time = None
        self.retired_total = 0
        self.last_expiration = []

    def after_frame(self, frame_idx, visible_ids, now):
        s = self.session
        if (type(now) not in (int, float) or not math.isfinite(now)
                or self.last_time is not None and now < self.last_time
                or type(frame_idx) is not int or frame_idx != self.last_frame + 1):
            raise ValueError('SAM3 lifecycle requires consecutive frames and a monotonic time')
        active = set(s.obj_ids)
        visible = set(visible_ids)
        if (len(active) != len(s.obj_ids) or len(active) != s.get_obj_num()
                or not visible <= active or s.output_buffer
                or any(type(obj_id) is not int or obj_id < 0 for obj_id in active | visible)):
            raise ValueError('SAM3 lifecycle native object/visible-output contract changed')
        indices = set(range(s.get_obj_num()))
        for name in ('point_inputs_per_obj', 'mask_inputs_per_obj', 'output_dict_per_obj', 'frames_tracked_per_obj'):
            if set(getattr(s, name)) != indices:
                raise ValueError('SAM3 lifecycle requires dense native per-object slots')
        if set(s._obj_id_to_idx) != active or set(s._obj_idx_to_id.values()) != active:
            raise ValueError('SAM3 lifecycle native ID mappings disagree')
        for obj_id in active:
            alive = s.trk_keep_alive.get(obj_id)
            if type(alive) not in (int, float) or not math.isfinite(alive):
                raise ValueError('SAM3 lifecycle requires a finite native keep-alive signal')
        for obj_id in active:
            self.last_visible_or_birth.setdefault(obj_id, now)
        for obj_id in visible:
            self.last_visible_or_birth[obj_id] = now
        eligible = sorted((obj_id for obj_id in active - visible
            if now - self.last_visible_or_birth[obj_id] >= LOST_TRACK_TTL_SECONDS
            and s.trk_keep_alive[obj_id] <= 0),
            key=lambda obj_id: (self.last_visible_or_birth[obj_id], obj_id))
        allocator = s.max_obj_id
        frames, prompts, cache = s.processed_frames, s.prompts, s.cache
        retired = []
        dormant = []
        for obj_id in eligible:
            if s.get_obj_num() <= 1:
                dormant.append(obj_id)
                continue
            event = dict(id=obj_id, absent_seconds=now - self.last_visible_or_birth[obj_id],
                         native_keep_alive=s.trk_keep_alive[obj_id],
                         reason='no_rendered_mask_30s_and_native_keep_alive_exhausted')
            s.remove_object(obj_id, strict=True)
            if (s.max_obj_id != allocator or s.processed_frames is not frames
                    or s.prompts is not prompts or s.cache is not cache or obj_id in s.obj_ids):
                raise ValueError('SAM3 public removal changed allocator or session identity')
            retired.append(event)
        retired_ids = {item['id'] for item in retired}
        if retired_ids:
            _clean_retired_metadata(s, retired_ids)
            self.last_expiration = [dict(event, frame_index=frame_idx) for event in retired]
        self.last_visible_or_birth = {obj_id: last for obj_id, last in self.last_visible_or_birth.items()
                                     if obj_id in s.obj_ids}
        self.retired_total += len(retired)
        self.last_frame, self.last_time = frame_idx, now
        return dict(lost_ttl_seconds=LOST_TRACK_TTL_SECONDS, visible_ids=sorted(visible),
                    visible_count=len(visible), native_track_ids=list(s.obj_ids),
                    native_track_count=len(s.obj_ids), available_track_slots=MAX_OBJECTS - len(s.obj_ids),
                    expired_this_frame=retired, expired_total=self.retired_total,
                    last_expiration=self.last_expiration,
                    last_dormant_retained_ids=dormant, max_native_id=allocator,
                    absent_tracks=[dict(id=obj_id, absent_seconds=now - self.last_visible_or_birth[obj_id],
                                        native_keep_alive=s.trk_keep_alive[obj_id])
                                   for obj_id in s.obj_ids if obj_id not in visible],
                    reappearance_policy='new_native_id_after_expiration', session_reset=False)


def expire_lost_streaming_tracks(session, frame_idx, visible_ids, now, config):
    if (config.recondition_every_nth_frame != RECONDITION_EVERY
            or config.recondition_on_trk_masks is not False or config.max_num_objects != MAX_OBJECTS
            or config.decrease_trk_keep_alive_for_empty_masklets is not True):
        raise ValueError('SAM3 lost-track lifecycle policy changed during streaming')
    lifecycle = getattr(session, '_factory_lost_tracks', None)
    memory = getattr(session, '_factory_bounded_memory', None)
    if (not isinstance(lifecycle, LostTrackLifecycle) or not isinstance(memory, BoundedVideoMemory)
            or memory.last_completed_frame != frame_idx - 1
            or len(memory.timeline) != frame_idx + 1):
        raise ValueError('Run SAM3 lifecycle after inference and before bounded frame cleanup')
    return lifecycle.after_frame(frame_idx, visible_ids, now)
