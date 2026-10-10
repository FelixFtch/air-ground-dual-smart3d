"""Controller contract tests: recovery uses the same identities and 3D limits."""
import threading,time
from types import SimpleNamespace
import pytest
from sim_io.runtime import Runtime
import sim_io.runtime as module

def example(dimension=3):
    runtime=Runtime.__new__(Runtime);runtime.epoch='e';runtime.sim_time=5.;runtime.capture_sim_time=5.;runtime.map_lock=threading.RLock()
    runtime.scene={'bounds':[[-30,30],[-30,30],[-.5,8]],'flight_bounds':[[-20,20],[-20,20],[.8,6]],'resolution':.25,'carla_world_origin_m':[0,0,.22]}
    runtime.cache=SimpleNamespace(cells={},identity={'scene_epoch':'e'},revision=10)
    runtime.event=lambda *a,**kw:None;runtime.dynamic_boxes=lambda rid:[];runtime.person_hazards=lambda rid:[]
    cfg={'dimension':dimension,'body_half_extents_m':[.7,.7,.36],'body_offset_m':[0,0,0],'goal_tolerance_m':.5,'execution_guard':{'sensor_max_age_s':1.2,'free_evidence_max_age_s':None}}
    state={'instance':{'robot_id':'r','robot_config':cfg,'route':[[10,10,2]]},'actor':SimpleNamespace(id=1),'position':[0,0,.8],'yaw':0,'speed':0,'route_index':0,'goal_revision':1,'sensor_fresh':{'lidar':{'sim_time':5}},'last_plan_wall':time.monotonic(),'planner':{'robot_id':'r','session_id':'e-r','map':{'scene_epoch':'e','revision':10},'goal_revision':1,'path_xyz':[],'path_status':'current_blocked_or_out_of_bounds'}}
    return runtime,state

@pytest.mark.parametrize('bad',['stale','session','map','goal'])
def test_recovery_does_not_bypass_normal_identity_and_age_gates(monkeypatch,bad):
    runtime,state=example();called=[];runtime.recover_overlap=lambda *a:called.append(True)
    if bad=='stale':state['last_plan_wall']-=5
    elif bad=='session':state['planner']['session_id']='other'
    elif bad=='map':state['planner']['map']['scene_epoch']='other'
    else:state['planner']['goal_revision']=0
    assert runtime.control(state,.03) is None
    assert not called

def test_uav_recovery_uses_flight_bounds_and_three_dimensional_prefix(monkeypatch):
    runtime,state=example();seen={}
    def target(*a,**kw):seen.update(kw);return [0,0,.9]
    def check(*a,**kw):seen['prefix_dimension']=kw['dimension'];return {'ok':True}
    monkeypatch.setattr(module,'find_recovery_target',target);monkeypatch.setattr(module,'check_recovery_segment',check)
    assert runtime.recover_overlap(state,.03) is not None
    assert seen['dimension']==seen['prefix_dimension']==3
    assert seen['bounds']['min'][2]==pytest.approx(.8-.36)
    assert seen['bounds']['max'][2]==pytest.approx(6+.36)


def test_recovered_session_is_accepted_but_old_session_stays_rejected():
    runtime,state=example();called=[]
    runtime.recover_overlap=lambda *a:called.append(True)
    state['planner_session_id']='e-r-recovery1'
    assert runtime.control(state,.03) is None
    assert state['status']=='waiting_map_revision' and not called
    state['planner']['session_id']='e-r-recovery1'
    runtime.control(state,.03)
    assert called==[True]


def test_goal_transition_keeps_verified_tree_and_revokes_old_path():
    runtime,state=example()
    state['instance']['route']=[[0,0,.8],[10,10,2]]
    state['completed_goals']=0
    state['planner'].update(path_xyz=[[0,0,.8],[1,0,.8]],path_status='candidate_path',
        tree_nodes=[{'id':0,'xyz':[0,0,.8],'parent':None,'tree_id':0},
                    {'id':1,'xyz':[1,0,.8],'parent':0,'tree_id':0}],tree_edges=[[1,0]],tree_instance_id=5)
    nodes,edges=state['planner']['tree_nodes'],state['planner']['tree_edges']
    assert runtime.control(state,.03) is None
    assert state['goal_revision']==2 and state['completed_goals']==1
    assert state['planner']['tree_nodes'] is nodes and state['planner']['tree_edges'] is edges
    assert state['planner']['goal_revision']==1 and state['planner']['path_xyz'] is None
    assert state['planner']['path_status']=='awaiting_goal_update'
    assert runtime.control(state,.03) is None
    assert state['status']=='waiting_path' and state['speed']==0


def test_recovered_session_nonempty_path_passes_both_execution_identity_checks():
    runtime,state=example()
    cfg=state['instance']['robot_config']
    cfg.update(body_half_extents_m=[.1,.1,.1],max_speed_mps=1.,max_acceleration_mps2=2.,max_yaw_rate_rps=1.)
    cfg['execution_guard']['near_path_m']=.4
    state['instance']['route']=[[2.,0.,.8]]
    state['planner_session_id']='e-r-recovery1'
    state['planner'].update(session_id='e-r-recovery1',path_xyz=[[0.,0.,.8],[2.,0.,.8]],path_status='ready')
    runtime.cache.cells={(x,y,z):{'state':'free','last_observed_sim_time':5.}
                         for x in range(-2,12) for y in range(-2,3) for z in range(1,6)}
    assert runtime.control(state,.03) is not None
    assert state['status']=='moving' and state['speed']>0
    state['planner']['session_id']='e-r'
    assert runtime.control(state,.03) is None
    assert state['status']=='waiting_map_revision' and state['speed']==0
