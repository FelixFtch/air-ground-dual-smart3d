"""No torch/model/GPU: execute exact pinned upstream public session methods."""
import ast
from collections import OrderedDict, defaultdict
import copy
import hashlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('candidate_memory',ROOT/'perception/vendor/sam3_video_memory.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
SOURCE=Path('/data/robotics/sam3_video_people/.venv/lib/python3.10/site-packages/transformers/models/sam3_video/modeling_sam3_video.py')
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==m.REVIEWED_SOURCES['transformers.models.sam3_video.modeling_sam3_video']
tree=ast.parse(SOURCE.read_text())
session_node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Sam3VideoInferenceSession')
hotstart_node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_process_hotstart')
TRACKER_SOURCE=SOURCE.parents[1]/'sam3_tracker_video/modeling_sam3_tracker_video.py'
assert hashlib.sha256(TRACKER_SOURCE.read_bytes()).hexdigest()==m.REVIEWED_SOURCES['transformers.models.sam3_tracker_video.modeling_sam3_tracker_video']
tracker_tree=ast.parse(TRACKER_SOURCE.read_text())
select_node=next(n for n in ast.walk(tracker_tree) if isinstance(n,ast.FunctionDef) and n.name=='_select_closest_cond_frames')
class Cache:
    def __init__(self,**kwargs):self.cleared=0
    def clear_all(self):self.cleared+=1
ns=dict(OrderedDict=OrderedDict,defaultdict=defaultdict,Sam3VideoInferenceCache=Cache)
module=ast.Module(body=[ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0),copy.deepcopy(session_node),copy.deepcopy(hotstart_node),copy.deepcopy(select_node)],type_ignores=[])
exec(compile(ast.fix_missing_locations(module),'exact_pinned_upstream_session','exec'),ns)
Session=ns['Sam3VideoInferenceSession']

def session():
    s=Session();s.add_prompt('person');s.obj_with_new_inputs=[]
    return s

def birth(s, frame=0, alive=0):
    s.max_obj_id+=1;obj=s.max_obj_id;idx=s.obj_id_to_idx(obj)
    s.output_dict_per_obj[idx]['cond_frame_outputs'][frame]={'identity':object()}
    s.mask_inputs_per_obj[idx][frame]=object();s.frames_tracked_per_obj[idx][frame]={'reverse':False}
    s.obj_id_to_prompt_id[obj]=0;s.obj_id_to_score[obj]=.00001
    s.obj_id_to_last_occluded[obj]=frame;s.obj_first_frame_idx[obj]=frame
    s.trk_keep_alive[obj]=alive;s.unmatched_frame_inds[obj]=[frame]
    s.obj_id_to_tracker_score_frame_wise[frame][obj]=.00001
    s.suppressed_obj_ids[frame].add(obj);s.removed_obj_ids.add(obj);s.hotstart_removed_obj_ids.add(obj)
    s.overlap_pair_to_frame_inds[(0,obj)]=[frame]
    return obj

def config_model():
    c=NS(num_maskmem=7,max_object_pointers_in_encoder=16,max_cond_frame_num=4)
    tracker=NS(training=False,config=c,_select_closest_cond_frames=lambda *args:ns['_select_closest_cond_frames'](None,*args))
    model=NS(training=False,tracker_model=tracker,recondition_every_nth_frame=16,recondition_on_trk_masks=False,max_num_objects=16,
             config=NS(tracker_config=c,recondition_every_nth_frame=16,recondition_on_trk_masks=False,max_num_objects=16,
                       decrease_trk_keep_alive_for_empty_masklets=True))
    return model

class LifecycleTests(unittest.TestCase):
    def test_30_second_boundary_capacity_recovery_last_dormant_and_no_id_reuse(self):
        s=session();[birth(s) for _ in range(16)];life=m.LostTrackLifecycle(s)
        life.after_frame(0,[],0.)
        self.assertEqual(life.after_frame(1,[],29.999)['expired_this_frame'],[])
        receipt=life.after_frame(2,[],30.)
        self.assertEqual(len(receipt['expired_this_frame']),15)
        self.assertEqual(receipt['last_dormant_retained_ids'],[15]);self.assertEqual(s.obj_ids,[15])
        self.assertEqual(receipt['available_track_slots'],15);self.assertEqual(s.max_obj_id,15)
        self.assertEqual(s.cache.cleared,0);self.assertEqual(s.prompts,{0:'person'})
        new=birth(s,3);self.assertEqual(new,16)
        receipt=life.after_frame(3,[16],31.)
        self.assertEqual([r['id'] for r in receipt['expired_this_frame']],[15])
        self.assertEqual(s.obj_ids,[16]);self.assertEqual(s.max_obj_id,16)
        self.assertEqual(s.cache.cleared,0)

    def test_visible_even_zero_native_keepalive_and_low_score_never_expires(self):
        s=session();birth(s);birth(s);life=m.LostTrackLifecycle(s)
        life.after_frame(0,[0],0.)
        result=life.after_frame(1,[0],1000.)
        self.assertEqual(s.obj_ids,[0]);self.assertEqual(result['visible_ids'],[0])
        self.assertEqual(s.obj_id_to_score[0],.00001)
        self.assertEqual(result['expired_this_frame'][0]['id'],1)

    def test_positive_keepalive_blocks_expiry_and_reappearance_restarts_absence_clock(self):
        s=session();birth(s,alive=1);birth(s);life=m.LostTrackLifecycle(s)
        life.after_frame(0,[],0.)
        result=life.after_frame(1,[1],40.)
        self.assertEqual(result['expired_this_frame'],[])
        s.trk_keep_alive[0]=0
        life.after_frame(2,[0],41.)
        self.assertEqual(life.after_frame(3,[1],70.999)['expired_this_frame'],[])
        self.assertEqual(life.after_frame(4,[1],71.)['expired_this_frame'][0]['id'],0)

    def test_public_slot_remap_preserves_remaining_memory_and_aux_refs_are_clean(self):
        s=session();[birth(s) for _ in range(3)];life=m.LostTrackLifecycle(s)
        kept=s.output_dict_per_obj[2];prompt=s.prompts;cache=s.cache
        life.after_frame(0,[2],0.);life.after_frame(1,[2],31.)
        self.assertEqual(s.obj_ids,[2]);self.assertIs(s.output_dict_per_obj[0],kept)
        self.assertEqual(s._obj_id_to_idx,{2:0});self.assertEqual(s._obj_idx_to_id,{0:2})
        self.assertIs(s.prompts,prompt);self.assertIs(s.cache,cache);self.assertEqual(cache.cleared,0)
        for name in ('obj_id_to_score','obj_id_to_last_occluded','obj_first_frame_idx','unmatched_frame_inds','trk_keep_alive','obj_id_to_prompt_id'):
            self.assertEqual(set(getattr(s,name)),{2},name)
        for scores in s.obj_id_to_tracker_score_frame_wise.values():self.assertTrue(set(scores)<={2})
        for items in s.suppressed_obj_ids.values():self.assertTrue(items<={2})
        self.assertFalse(s.overlap_pair_to_frame_inds)
        self.assertTrue(s.removed_obj_ids<={2});self.assertTrue(s.hotstart_removed_obj_ids<={2})

    def test_native_empty_mask_keepalive_decrements_only_with_public_policy(self):
        s=session();birth(s,alive=30)
        meta=dict(obj_first_frame_idx=s.obj_first_frame_idx,unmatched_frame_inds=s.unmatched_frame_inds,
                  trk_keep_alive=s.trk_keep_alive,overlap_pair_to_frame_inds=s.overlap_pair_to_frame_inds,
                  removed_obj_ids=set(),suppressed_obj_ids=defaultdict(set))
        model=NS(hotstart_delay=15,init_trk_keep_alive=30,max_trk_keep_alive=30,min_trk_keep_alive=-1,
                 decrease_trk_keep_alive_for_empty_masklets=True)
        for i in range(31):
            ns['_process_hotstart'](model,s,i,False,{},[],[0],[],meta,streaming=True)
        self.assertEqual(s.trk_keep_alive[0],-1)

    def test_empty_frame_no_objects_and_clock_contract(self):
        s=session();life=m.LostTrackLifecycle(s)
        self.assertEqual(life.after_frame(0,[],2.)['native_track_count'],0)
        for index,now in [(0,3.),(1,float('nan')),(1,1.)]:
            with self.assertRaises(ValueError):life.after_frame(index,[],now)
        self.assertEqual(life.last_frame,0)

    def test_expiry_and_bounded_memory_together_preserve_timeline_and_2000_new_ids(self):
        s=session();model=config_model()
        with patch.object(m,'_verify_sources'):m.configure_streaming_session(s,model)
        birth(s);expected_max=0
        for frame in range(2001):
            s.processed_frames[frame]=object()
            if frame:
                expected_max+=1;self.assertEqual(birth(s,frame),expected_max)
            receipt=m.expire_lost_streaming_tracks(s,frame,[],frame*31.,model.config)
            counts=m.trim_streaming_session(s,frame,model.config)
            self.assertEqual(s.max_obj_id,expected_max);self.assertLessEqual(len(s.obj_ids),2)
            self.assertEqual(counts['retained_frames'],1);self.assertEqual(counts['total_frames'],frame+1)
            self.assertLessEqual(len(s.trk_keep_alive),2)
            self.assertLessEqual(len(s.overlap_pair_to_frame_inds),1)
            self.assertEqual(s.cache.cleared,0)
        self.assertGreater(receipt['expired_total'],1990)

if __name__=='__main__':unittest.main(verbosity=2)
