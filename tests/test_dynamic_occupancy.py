"""Real OctoMap regression: a visible transient must not lose to old free votes."""
from pathlib import Path
import os
from mapping.client import OctomapClient
import pytest
ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def mapper():
    with OctomapClient(os.environ.get('OCTOMAP_SERVICE_BIN',ROOT/'mapping/build/octomap_service')) as c:
        c.request('init',scene_id='s',scene_epoch='e',content_version='v',frame_id='map',resolution=.25,
                  prob_hit=.9,prob_miss=.1,clamp_min=.12,clamp_max=.88)
        yield c

def scan(c,seq,stamp,hit,robot='go2'):
    return c.request('integrate',scene_id='s',scene_epoch='e',robot_id=robot,sensor_id='lidar',
        frame_seq=seq,capture_sim_time=stamp,receive_wall_time=0,calibration_id='c',
        origin=[.125,.125,1.125],points=[[2.125 if hit else 4.125,.125,1.125]])

def cell(c):return c.request('query',points=[[2.125,.125,1.125]])['cells'][0]

def test_one_current_hit_marks_long_observed_free_and_one_clear_ray_removes(mapper):
    for i in range(20):scan(mapper,i,i*.1,False)
    assert cell(mapper)['state']=='free'
    scan(mapper,20,2.,True)
    assert cell(mapper)['state']=='occupied'
    for i in range(21,40):scan(mapper,i,i*.1,True)
    scan(mapper,40,4.,False)
    assert cell(mapper)['state']=='free'

def test_delayed_other_camera_free_does_not_erase_new_person(mapper):
    scan(mapper,1,1.,True)
    scan(mapper,1,.9,False,'uav')
    assert cell(mapper)['state']=='occupied'
    assert cell(mapper)['last_observed_sim_time']==1.

def test_delayed_hit_does_not_restore_person_after_new_clear_ray(mapper):
    scan(mapper,1,1.,False)
    scan(mapper,1,.9,True,'uav')
    assert cell(mapper)['state']=='free'

@pytest.mark.parametrize('hit_first',[True,False])
def test_equal_capture_timestamp_hit_wins_regardless_of_delivery_order(mapper,hit_first):
    scan(mapper,1,1.,hit_first)
    scan(mapper,1,1.,not hit_first,'uav')
    assert cell(mapper)['state']=='occupied'
