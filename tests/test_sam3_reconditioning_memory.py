"""Execute pinned upstream selection/correction methods, without model/GPU load."""
import ast
from collections import OrderedDict, defaultdict
import copy
import hashlib
import importlib.util
from pathlib import Path
import random
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('candidate_reconditioning',ROOT/'perception/vendor/sam3_video_memory.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
UPSTREAM=Path('/data/robotics/sam3_video_people/.venv/lib/python3.10/site-packages/transformers/models')


def load_methods(folder, names):
    path=UPSTREAM/folder/('modeling_'+folder+'.py')
    assert hashlib.sha256(path.read_bytes()).hexdigest()==m.REVIEWED_SOURCES['transformers.models.'+folder+'.modeling_'+folder]
    tree=ast.parse(path.read_text())
    selected=[]
    for name in names:
        node=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name))
        node.decorator_list=[]
        selected.append(node)
    module=ast.Module(body=[ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0),*selected],type_ignores=[])
    env=dict(OrderedDict=OrderedDict,defaultdict=defaultdict,Sam3VideoInferenceCache=lambda **kwargs:NS())
    exec(compile(ast.fix_missing_locations(module),str(path),'exec'),env)
    return env


tracker=load_methods('sam3_tracker_video',['_select_closest_cond_frames','_gather_memory_frame_outputs','_get_object_pointers'])
video=load_methods('sam3_video',['Sam3VideoInferenceSession','_prepare_recondition_masks'])
select=tracker['_select_closest_cond_frames']


class Tracker:
    training=False
    num_maskmem=7
    config=NS(num_maskmem=7,max_object_pointers_in_encoder=16,max_cond_frame_num=4)
    _select_closest_cond_frames=select
    _gather_memory_frame_outputs=tracker['_gather_memory_frame_outputs']
    _get_object_pointers=tracker['_get_object_pointers']


class Token:
    """Identity-preserving stand-in for a tensor's device transfer."""
    def __init__(self,frame):self.frame=frame
    def to(self,device):return self


def output(frame):return dict(object_pointer=Token(frame),maskmem_features=Token(frame))


def model():
    return NS(training=False,tracker_model=Tracker(),recondition_every_nth_frame=16,recondition_on_trk_masks=False,
        max_num_objects=16,config=NS(recondition_every_nth_frame=16,recondition_on_trk_masks=False,
            max_num_objects=16,tracker_config=Tracker.config,decrease_trk_keep_alive_for_empty_masklets=True))


def new_session():
    session=video['Sam3VideoInferenceSession']();session.add_prompt('person');session.obj_with_new_inputs=[]
    with patch.object(m,'_verify_sources'):m.configure_streaming_session(session,model())
    return session


def birth(session,frame,obj_id=0):
    session.max_obj_id=max(session.max_obj_id,obj_id)
    index=session.obj_id_to_idx(obj_id)
    session.output_dict_per_obj[index]['cond_frame_outputs'][frame]=output(frame)
    session.frames_tracked_per_obj[index][frame]={'reverse':False}
    session.mask_inputs_per_obj[index][frame]=Token(frame)
    session.obj_first_frame_idx[obj_id]=frame;session.obj_id_to_score[obj_id]=.9
    session.obj_id_to_prompt_id[obj_id]=0;session.trk_keep_alive[obj_id]=30


class CorrectionMask:
    """Small mask stub that exposes which upstream source was selected."""
    def __init__(self,label):self.label=label
    def __getitem__(self,key):return CorrectionMask(self.label)
    def unsqueeze(self,*args):return self
    def __ge__(self,threshold):return CorrectionMask(self.label+'_threshold_'+str(threshold))


class ReconditioningTests(unittest.TestCase):
    def test_public_configuration_restores_checkpoint_correction_and_not_tracking_mask_validation(self):
        mdl=model();mdl.recondition_every_nth_frame=0
        with patch.object(m,'_verify_sources'):policy=m.configure_streaming_model(mdl)
        self.assertEqual(mdl.recondition_every_nth_frame,16)
        self.assertEqual(mdl.config.recondition_every_nth_frame,16)
        self.assertFalse(mdl.recondition_on_trk_masks)
        self.assertFalse(mdl.config.recondition_on_trk_masks)
        self.assertEqual(policy['retained_conditioning_frames_per_object'],4)
        self.assertFalse(policy['upstream_all_conditioning_pointer_equivalence'])
        self.assertFalse(policy['resets_for_memory'])

    def test_exact_native_correction_uses_detection_mask_for_same_native_id(self):
        session=NS(obj_id_to_idx=lambda oid:{7:0}[oid])
        masks,ids=video['_prepare_recondition_masks'](NS(high_conf_thresh=.8,recondition_on_trk_masks=False),session,16,
            {'mask':CorrectionMask('detector')},CorrectionMask('tracker'),{7:2},[.95])
        self.assertEqual(ids,{7});self.assertEqual(set(masks),{0})
        self.assertEqual(masks[0].label,'detector_threshold_0.5')
        masks,ids=video['_prepare_recondition_masks'](NS(high_conf_thresh=.8,recondition_on_trk_masks=False),session,16,
            {'mask':CorrectionMask('detector')},CorrectionMask('tracker'),{7:2},[.7])
        self.assertFalse(masks);self.assertFalse(ids)

    def test_exact_native_selector_has_before_after_and_closest_semantics(self):
        frames={i:output(i) for i in (0,16,32,48,64,80)}
        chosen,_=select(None,49,frames,4)
        self.assertEqual(list(chosen),[48,64,32,80])
        chosen,_=select(None,81,frames,4)
        self.assertEqual(list(chosen),[80,64,48,32])

    def test_fifth_condition_retired_after_output_retains_exact_native_next_feature_selection_order(self):
        s=new_session();birth(s,0)
        full={0:s.output_dict_per_obj[0]['cond_frame_outputs'][0]}
        for frame in range(65):
            s.processed_frames[frame]=Token(frame)
            if frame and frame%16==0:
                value=output(frame);s.output_dict_per_obj[0]['cond_frame_outputs'][frame]=value;full[frame]=value
            receipt=m.trim_streaming_session(s,frame,model().config)
            expected,_=select(None,frame+1,full,4)
            got=s.output_dict_per_obj[0]['cond_frame_outputs']
            self.assertEqual(list(got),list(expected))
            self.assertTrue(all(got[key] is expected[key] for key in got))
        self.assertEqual(list(got),[64,48,32,16]);self.assertEqual(receipt['retired_conditioning_this_frame'],1)
        self.assertEqual(s.obj_ids,[0]);self.assertEqual(s.obj_first_frame_idx[0],0)
        self.assertEqual(s.obj_id_to_score[0],.9);self.assertEqual(s.obj_id_to_prompt_id[0],0)

    def test_sparse_correction_and_late_birth_keep_native_feature_memory_for_all_future_forward_frames(self):
        rng=random.Random(17)
        for birth_frame in (0,1,15,16,17,39):
            s=new_session();full={};mdl=model()
            for frame in range(600):
                s.processed_frames[frame]=Token(frame)
                if frame==birth_frame:
                    birth(s,frame);full[frame]=s.output_dict_per_obj[0]['cond_frame_outputs'][frame]
                elif frame>birth_frame:
                    value=output(frame)
                    if frame%16==0 and rng.random()<.6:
                        s.output_dict_per_obj[0]['cond_frame_outputs'][frame]=value;full[frame]=value
                    else:s.output_dict_per_obj[0]['non_cond_frame_outputs'][frame]=value
                m.trim_streaming_session(s,frame,mdl.config)
                if frame<birth_frame:continue
                expected,_=select(None,frame+1,full,4)
                got=s.output_dict_per_obj[0]['cond_frame_outputs']
                self.assertEqual(list(got),list(expected))
                for future in (frame+1,frame+7,frame+1000):
                    original=NS(output_dict_per_obj={0:dict(cond_frame_outputs=full,non_cond_frame_outputs=s.output_dict_per_obj[0]['non_cond_frame_outputs'])})
                    self.assertEqual(mdl.tracker_model._gather_memory_frame_outputs(s,0,future),
                        mdl.tracker_model._gather_memory_frame_outputs(original,0,future))

    def test_10000_frames_keep_all_state_bounded_and_pointer_budget_honest(self):
        s=new_session();birth(s,0);mdl=model()
        for frame in range(10000):
            s.processed_frames[frame]=Token(frame)
            if frame:
                key='cond_frame_outputs' if frame%16==0 else 'non_cond_frame_outputs'
                s.output_dict_per_obj[0][key][frame]=output(frame)
                s.frames_tracked_per_obj[0][frame]={'reverse':False}
            s.obj_id_to_tracker_score_frame_wise[frame][0]=.9;s.suppressed_obj_ids[frame]=set()
            receipt=m.trim_streaming_session(s,frame,mdl.config)
            self.assertLessEqual(len(s.output_dict_per_obj[0]['cond_frame_outputs']),4)
            self.assertLessEqual(len(s.output_dict_per_obj[0]['non_cond_frame_outputs']),128)
            self.assertLessEqual(len(s.frames_tracked_per_obj[0]),132)
            offsets,pointers,nominal=mdl.tracker_model._get_object_pointers(s,0,frame+1,frame+1,'cpu')
            self.assertLessEqual(len(pointers),19)  # Native limit16 caps non-cond offsets, not added cond pointers.
            self.assertEqual(s.max_obj_id,0);self.assertEqual(s.obj_ids,[0])
            self.assertEqual(s.processed_frames.retained_frames,1)
        self.assertEqual(receipt['total_frames'],10000)
        self.assertEqual(receipt['conditioning_frames'],4)

    def test_no_false_unbounded_pointer_equivalence_claim(self):
        mdl=model();cond={i:output(i) for i in range(0,160,16)}
        full=NS(output_dict_per_obj={0:dict(cond_frame_outputs=cond,non_cond_frame_outputs={})})
        chosen,_=select(None,160,cond,4)
        finite=NS(output_dict_per_obj={0:dict(cond_frame_outputs=chosen,non_cond_frame_outputs={})})
        self.assertEqual(len(mdl.tracker_model._get_object_pointers(full,0,160,160,'cpu')[1]),10)
        self.assertEqual(len(mdl.tracker_model._get_object_pointers(finite,0,160,160,'cpu')[1]),4)

    def test_future_frame_and_dense_unselected_recent_conditioning_rejected_before_mutation(self):
        s=new_session();birth(s,0);s.processed_frames[0]=Token(0)
        s.output_dict_per_obj[0]['cond_frame_outputs'][1]=output(1)
        before=dict(s.output_dict_per_obj[0]['cond_frame_outputs'])
        with self.assertRaises(ValueError):m.trim_streaming_session(s,0,model().config)
        self.assertEqual(s.output_dict_per_obj[0]['cond_frame_outputs'],before)
        s=new_session();birth(s,0)
        for frame in range(4):
            s.processed_frames[frame]=Token(frame)
            if frame:s.output_dict_per_obj[0]['cond_frame_outputs'][frame]=output(frame)
            m.trim_streaming_session(s,frame,model().config)
        s.processed_frames[4]=Token(4);s.output_dict_per_obj[0]['cond_frame_outputs'][4]=output(4)
        before=dict(s.output_dict_per_obj[0]['cond_frame_outputs'])
        with self.assertRaisesRegex(ValueError,'recent-memory'):m.trim_streaming_session(s,4,model().config)
        self.assertEqual(s.output_dict_per_obj[0]['cond_frame_outputs'],before)

    def test_config_drift_and_frame_revisit_fail(self):
        s=new_session();birth(s,0);s.processed_frames[0]=Token(0)
        config=model().config;config.recondition_every_nth_frame=0
        with self.assertRaises(ValueError):m.trim_streaming_session(s,0,config)
        m.trim_streaming_session(s,0,model().config)
        with self.assertRaises(ValueError):m.trim_streaming_session(s,0,model().config)


if __name__=='__main__':unittest.main()
