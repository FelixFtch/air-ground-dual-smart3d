"""Live people safety/display must not wait for a slow planner result."""
import copy
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
import sim_io.runtime as module
from sim_io.runtime import Runtime
from observability.tree_snapshots import TreeSnapshotWriter


def profile(dimension):
    return dict(profile_id=f'robot-{dimension}d',dimension=dimension,
        body_half_extents_m=[.4,.4,.3],body_offset_m=[0.,0.,0.],
        planar_z=.33,mobility='ground_xy_yaw' if dimension==2 else 'xyz_yaw')


def document(reference=10.,capture=9.5,position=10.,until=11.5):
    return dict(schema='air_ground.people/v1',scene_epoch='epoch',scene_id='test',
        reference_sim_time=reference,people=[dict(global_id='person-1',
            capture_sim_time=capture,position_reference_sim_time=position,
            valid_until_sim_time=until,center_xyz=[2.,0.,1.],
            half_extents_xyz=[.2,.2,.6],velocity_xyz=[.5,0.,0.],
            velocity_xy_valid=True,velocity_z_valid=False,
            source_keys=['camera-session-1'],quality={'extent_kind':'visible_surface_quantiles'})])


def fixture(tmp_path):
    runtime=Runtime.__new__(Runtime)
    runtime.root=tmp_path;runtime.epoch='epoch'
    runtime.scene=dict(scene_id='test',person_prediction_enabled=True,resolution=.25,ground_z_m=0.)
    runtime.people_documents={};runtime.people_error=None
    runtime.sim_time=runtime.capture_sim_time=10.
    runtime.events=[]
    runtime.event=lambda kind,**fields:runtime.events.append((kind,fields))
    runtime.robots={rid:dict(instance=dict(robot_config=profile(d),route=[[5.,0.,.33]]),
        route_index=0,actor=SimpleNamespace(id=d),position=[0.,0.,.33],yaw=0.,
        planner=dict(sim_time=7.,people_hazards=[{'id':'expired-planner-person','valid_until_sim_time':8.}]),
        status='moving',sensor_fresh={}) for rid,d in [('go2',2),('uav',3)]}
    (tmp_path/'shared').mkdir()
    write(runtime,document())
    return runtime


def write(runtime,doc):
    (runtime.root/'shared/people.json').write_text(json.dumps(doc))


def test_both_robots_use_one_current_document_and_reference(tmp_path,monkeypatch):
    runtime=fixture(tmp_path)
    old_read=Path.read_text;reads=[]
    def read(path,*args,**kwargs):
        value=old_read(path,*args,**kwargs)
        if path.name=='people.json':
            reads.append(path)
            # Current time advances during I/O. Pick it only after the read.
            runtime.capture_sim_time=10.2
        return value
    monkeypatch.setattr(Path,'read_text',read)
    reference,hazards=runtime.person_hazard_snapshot(('go2','uav'))
    assert len(reads)==1 and reference==10.2
    for rid,shape in [('go2','box'),('uav','box')]:
        assert len(hazards[rid])==1
        assert hazards[rid][0]['shape']==shape
        assert hazards[rid][0]['position_reference_sim_time']==reference
        assert hazards[rid][0]['valid_until_sim_time']==11.5
    assert runtime.people_error is None


def test_publication_refreshes_live_hazards_preserves_planner_evidence(tmp_path,monkeypatch):
    runtime=fixture(tmp_path)
    runtime.state_lock=threading.RLock();runtime.map_lock=threading.RLock()
    runtime.display_index=SimpleNamespace(snapshot=lambda:([],[],[]))
    runtime.tree_snapshots=TreeSnapshotWriter(tmp_path,runtime.epoch)
    for rid,state in runtime.robots.items():
        state['planner'].update(robot_id=rid,session_id='epoch-'+rid,tree_instance_id=1,
            path_xyz=[[0.,0.,.33],[2.,0.,.33]],
            tree_nodes=[{'id':0,'xyz':[0.,0.,.33]}],tree_edges=[])
    runtime.cache=SimpleNamespace(revision=1,cells={})
    runtime.map_watermark=9.;runtime.world_id=1;runtime.stop=threading.Event()
    runtime.people=None;runtime.map_timing=[];runtime.sensor_stats={};runtime.fusion_metrics={}
    runtime.publish_timing=[];runtime.map_error=None;runtime.fatal=None
    runtime.motion_rpc_latency_s=None;runtime.gc_metrics={}
    old_plans={rid:copy.deepcopy(s['planner']) for rid,s in runtime.robots.items()}
    captured={}
    monkeypatch.setattr(module,'atomic_json',lambda path,data:captured.update({Path(path).name:copy.deepcopy(data)}))
    runtime.publish()
    state=captured['state.json']
    assert state['sim_time']==state['people_hazard_reference_sim_time']==10.
    for bot in state['robots']:
        assert bot['people_hazard_reference_sim_time']==10.
        assert bot['people_hazards'][0]['id']=='person-1'
        assert bot['people_hazards'][0]['valid_until_sim_time']==11.5
        assert bot['planner']['people_hazards']==old_plans[bot['robot_id']]['people_hazards']
        assert bot['planner']['path_xyz']==old_plans[bot['robot_id']]['path_xyz']
        assert bot['planner']['sim_time']==7.
        assert 'tree_nodes' not in bot['planner'] and bot['planner']['tree_ref'] is not None
        assert runtime.robots[bot['robot_id']]['planner']==old_plans[bot['robot_id']]


def test_only_future_publication_watermark_can_be_rebased(tmp_path):
    runtime=fixture(tmp_path)
    source=document(reference=10.05)
    write(runtime,source)
    reference,hazards=runtime.person_hazard_snapshot(('go2','uav'),10.)
    assert reference==10. and all(hazards.values()) and runtime.people_error is None
    assert source['reference_sim_time']==10.05  # The original document is immutable.
    for values in hazards.values():
        assert values[0]['capture_sim_time']==9.5
        assert values[0]['valid_until_sim_time']==11.5
    assert runtime.people_documents['go2']['reference_sim_time']==10.


@pytest.mark.parametrize('field',['capture_sim_time','position_reference_sim_time'])
def test_true_future_measurement_or_projection_keeps_previous_evidence(tmp_path,field):
    runtime=fixture(tmp_path)
    previous=runtime.person_hazards('go2',10.)
    source=document(reference=10.05)
    source['people'][0][field]=10.02
    write(runtime,source)
    result=runtime.person_hazards('go2',10.)
    assert result==previous and 'future' in runtime.people_error
    assert runtime.people_documents['go2']['reference_sim_time']==10.
    assert result[0]['valid_until_sim_time']==11.5


@pytest.mark.parametrize('bad',[None,True,'10.05',float('nan'),float('inf'),1e100])
def test_bad_publication_watermark_is_not_repaired(tmp_path,bad):
    runtime=fixture(tmp_path)
    source=document(reference=bad)
    write(runtime,source)
    assert runtime.person_hazards('go2',10.)==[]
    assert runtime.people_error is not None


@pytest.mark.parametrize('bad',[None,True,'9.5',float('nan'),float('inf')])
@pytest.mark.parametrize('field',['capture_sim_time','position_reference_sim_time'])
def test_bad_observation_time_is_not_hidden_by_watermark_rebase(tmp_path,bad,field):
    runtime=fixture(tmp_path)
    source=document(reference=10.05);source['people'][0][field]=bad
    write(runtime,source)
    assert runtime.person_hazards('go2',10.)==[]
    assert runtime.people_error is not None


def test_watermark_rebase_keeps_non_time_validation_strict(tmp_path):
    runtime=fixture(tmp_path)
    source=document(reference=10.05);source['people'][0]['half_extents_xyz']=[-1.,.1,.1]
    write(runtime,source)
    assert runtime.person_hazards('go2',10.)==[]
    assert 'half extents' in runtime.people_error


def test_fallback_and_live_evidence_expire_at_original_capture_deadline(tmp_path):
    runtime=fixture(tmp_path)
    assert runtime.person_hazards('go2',10.)
    (runtime.root/'shared/people.json').write_text('{')
    assert runtime.person_hazards('go2',11.49)[0]['valid_until_sim_time']==11.5
    assert runtime.person_hazards('go2',11.5)==[]
    write(runtime,document(reference=11.6))
    assert runtime.person_hazards('go2',11.5)==[]


def test_explicit_planner_reference_does_not_advance_with_runtime_clock(tmp_path):
    runtime=fixture(tmp_path);runtime.capture_sim_time=10.5
    reference,hazards=runtime.person_hazard_snapshot(('go2','uav'),10.)
    assert reference==10.
    assert all(values[0]['position_reference_sim_time']==10. for values in hazards.values())
