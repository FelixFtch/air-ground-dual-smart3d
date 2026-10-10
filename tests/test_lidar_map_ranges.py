"""Actual OcTree check: far clearing preserves its real obstacle endpoint."""
import os
from pathlib import Path
from mapping.client import OctomapClient
from sim_io.lidar_rays import prepare_lidar_rays

def test_near_marking_and_far_clearing_keep_true_surface():
    root=Path(__file__).resolve().parents[1]
    with OctomapClient(os.environ.get('OCTOMAP_SERVICE_BIN',root/'mapping/build/octomap_service')) as c:
        c.request('init',scene_id='r',scene_epoch='e',content_version='v',frame_id='map',resolution=.25,prob_hit=.9,prob_miss=.1,clamp_min=.12,clamp_max=.88,bounds={'min':[-50,-50,-1],'max':[50,50,8]})
        def scan(seq,origin,returns):
            hits,free,_=prepare_lidar_rays(returns,origin,.25,9,40,[[-50,50],[-50,50],[-1,8]])
            return c.request('integrate',scene_id='r',scene_epoch='e',robot_id='g',sensor_id='lidar',frame_seq=seq,capture_sim_time=float(seq),receive_wall_time=0.,calibration_id='c',origin=origin,points=hits.tolist(),free_rays=[{'endpoint':p.tolist(),'source':'lidar_measured_beyond_range','include_endpoint':False} for p in free],max_range_m=40,compact=True)
        scan(1,[20.125,.125,1.125],[[22.125,.125,1.125]])
        scan(2,[33.125,.125,1.125],[[35.125,.125,1.125]])
        scan(3,[.125,.125,1.125],[[35.125,.125,1.125]])
        cells=c.request('query',points=[[22.125,.125,1.125],[35.125,.125,1.125],[36.125,.125,1.125]])['cells']
        assert [v['state'] for v in cells]==['free','occupied','unknown']
        scan(4,[.125,2.125,1.125],[[8.125,2.125,1.125],[10.125,2.125,1.125]])
        cells=c.request('query',points=[[8.125,2.125,1.125],[10.125,2.125,1.125]])['cells']
        assert [v['state'] for v in cells]==['occupied','unknown']
