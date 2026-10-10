"""Known robot surfaces must not become human OHZ; nearby humans remain."""
import copy
import numpy as np
import pytest
from perception.people_projection import PeopleProjector
from test_people_perception import metadata, frame, result


def add_body(meta, robot='partner', center=(0,0,3), half=(.4,.7,.1), angle=0., offset=(0,0,0)):
    c,s=np.cos(angle),np.sin(angle)
    pose=np.array([[c,-s,0,center[0]],[s,c,0,center[1]],[0,0,1,center[2]],[0,0,0,1.]])
    body=dict(robot_id=robot,frame_seq=meta['frame_seq'],capture_sim_time=meta['capture_sim_time'],
              body_to_map=pose.tolist(),half_extents_xyz=list(half),body_offset_xyz=list(offset))
    meta.setdefault('cooperative_bodies',[]).append(body)
    if not any(b['robot_id']==meta['robot_id'] for b in meta['cooperative_bodies']):
        meta['cooperative_bodies'].append(dict(robot_id=meta['robot_id'],frame_seq=meta['frame_seq'],
            capture_sim_time=meta['capture_sim_time'],body_to_map=copy.deepcopy(meta['body_to_map']),
            half_extents_xyz=[.01,.01,.01],body_offset_xyz=[0,0,0]))
    return body


def test_complete_robot_mask_removed_and_old_velocity_history_retired():
    p=PeopleProjector();f=frame()
    assert p.project(f['meta'],result(),f['xyz_optical'])['tracks']
    assert p.history
    add_body(f['meta'])
    out=p.project(f['meta'],result(),f['xyz_optical'])
    assert out['tracks']==[] and not p.history
    assert out['rejected'][0]['reason']=='cooperative_robot_surface'
    assert out['rejected'][0]['excluded_robot_ids']==['partner']
    assert out['cooperative_body_filter']['removed_samples']>1000


def test_mixed_mask_removes_only_body_samples_preserves_adjacent_person():
    f=frame();add_body(f['meta'],half=(.2,.7,.1))
    # Left mask half is the robot. Right half contains a person beside it.
    f['xyz_optical'][:,20:,0]+=1.
    f['xyz_optical'][:,20:,2]+=np.linspace(-.2,.2,40)[:,None]
    out=PeopleProjector().project(f['meta'],result(),f['xyz_optical'])
    assert len(out['tracks'])==1
    person=out['tracks'][0]
    assert person['measured_min_xyz'][0]>1.
    assert person['quality']['excluded_robot_samples']>100
    assert person['quality']['excluded_robot_ids']==['partner']


def test_person_same_xy_different_height_is_not_filtered_by_radius():
    f=frame();add_body(f['meta'],center=(0,0,.33),half=(.43,.43,.28))
    out=PeopleProjector().project(f['meta'],result(),f['xyz_optical'])
    assert len(out['tracks'])==1
    assert out['tracks'][0]['quality']['excluded_robot_samples']==0


def test_body_rotation_and_offset_are_used_not_axis_aligned_latest_position():
    f=frame();f['xyz_optical'][:,:,:2]=[.8,0]
    add_body(f['meta'],half=(.1,.9,.1),angle=np.pi/2,offset=(0,.1,0))
    out=PeopleProjector().project(f['meta'],result(),f['xyz_optical'])
    assert not out['tracks']
    assert out['rejected'][0]['reason']=='cooperative_robot_surface'
    f['meta']['cooperative_bodies'][0]['body_to_map']=np.diag([1.,1.,1.,1.]).tolist()
    f['meta']['cooperative_bodies'][0]['body_to_map'][2][3]=3.
    assert len(PeopleProjector().project(f['meta'],result(),f['xyz_optical'])['tracks'])==1


def test_same_capture_geometry_does_not_exclude_where_robot_went_later():
    f=frame();add_body(f['meta'],center=(5,0,3))
    out=PeopleProjector().project(f['meta'],result(),f['xyz_optical'])
    assert len(out['tracks'])==1
    # No current runtime pose is an input to PeopleProjector.
    assert out['cooperative_body_filter']['removed_samples']==0


def test_mismatched_body_frame_or_time_cannot_filter_person():
    for key,value in [('frame_seq',99),('capture_sim_time',11.)]:
        f=frame();body=add_body(f['meta']);body[key]=value
        with pytest.raises(ValueError):PeopleProjector().project(f['meta'],result(),f['xyz_optical'])


def test_unavailable_optional_geometry_preserves_legacy_projection():
    f=frame();out=PeopleProjector().project(f['meta'],result(),f['xyz_optical'])
    assert len(out['tracks'])==1 and out['cooperative_body_filter']['available'] is False


def test_robot_removed_does_not_promote_remaining_floor_to_a_person():
    f=frame();f['meta']['camera_to_map'][2][3]=-3.
    f['xyz_optical'][:,:,:2]=0.
    f['xyz_optical'][:,:28,2]=3.5  # robot surface
    f['xyz_optical'][:,28:,0]=1.  # floor outside body, depth is still valid
    add_body(f['meta'],center=(0,0,.5),half=(.3,.3,.1))
    out=PeopleProjector().project(f['meta'],result(),f['xyz_optical'])
    assert out['tracks']==[]
    assert out['rejected'][0]['reason']=='cooperative_robot_surface'
    assert out['rejected'][0]['excluded_robot_samples']>100


def test_rejecting_false_people_does_not_remove_partner_execution_constraint():
    """Even with empty human observations and free voxels, a partner blocks travel."""
    import threading
    from test_runtime_recovery_integration import example
    runtime,state=example();runtime.state_lock=threading.RLock()
    cfg=state['instance']['robot_config']
    cfg.update(body_half_extents_m=[.1,.1,.1],max_speed_mps=1.,max_acceleration_mps2=2.,max_yaw_rate_rps=1.)
    cfg['execution_guard']['near_path_m']=.4
    state['instance']['route']=[[2.,0.,.8]]
    state['planner'].update(path_xyz=[[0.,0.,.8],[2.,0.,.8]],path_status='ready')
    runtime.cache.cells={(x,y,z):{'state':'free','last_observed_sim_time':5.}
                         for x in range(-2,12) for y in range(-2,3) for z in range(1,6)}
    partner=dict(instance={'robot_config':{'body_half_extents_m':[.1,.1,.1]}},position=[.35,0.,.8])
    runtime.robots={'r':state,'partner':partner}
    del runtime.dynamic_boxes  # Exercise the actual live partner geometry method.
    assert runtime.person_hazards('r')==[]
    runtime.control(state,.03)
    assert state['status']=='guard_live_partner' and state['speed']==0
    partner['position']=[3.,0.,.8]
    runtime.control(state,.03)
    assert state['status']=='moving' and state['speed']>0
