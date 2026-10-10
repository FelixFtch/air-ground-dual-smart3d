"""Exact mapping semantics while reducing live integration/cache overhead."""
import math
import os
from pathlib import Path
import unittest
from mapping.client import MapCache,MapError,OctomapClient
ROOT=Path(__file__).resolve().parents[1]
BINARY=Path(os.environ.get('OCTOMAP_SERVICE_BIN',ROOT/'mapping/build/octomap_service'))

class HotpathContractTests(unittest.TestCase):
    def test_free_prefix_excludes_real_far_surface(self):
        with OctomapClient(BINARY) as client:
            ident=dict(scene_id='test',scene_epoch='epoch')
            client.request('init',**ident,content_version='v1',frame_id='map',resolution=.25,prob_hit=.9,prob_miss=.1,clamp_min=.12,clamp_max=.88)
            frame=dict(**ident,robot_id='go2',sensor_id='lidar',calibration_id='v1',receive_wall_time=1,origin=[.125,.125,1.125],max_range_m=40,compact=True)
            client.request('integrate',**frame,frame_seq=1,capture_sim_time=1,points=[[2.125,.125,1.125],[4.125,.125,1.125]])
            client.request('integrate',**frame,frame_seq=2,capture_sim_time=2,points=[],free_rays=[dict(endpoint=[4.125,.125,1.125],source='lidar_measured_beyond_range',include_endpoint=False)])
            cells=client.request('query',points=[[2.125,.125,1.125],[4.125,.125,1.125],[4.375,.125,1.125]])['cells']
            self.assertEqual([c['state'] for c in cells],['free','occupied','unknown'])
            self.assertEqual(cells[1]['last_observed_sim_time'],1)
            # The compatible default still includes a cropped endpoint.
            client.request('integrate',**frame,frame_seq=3,capture_sim_time=3,points=[],free_rays=[dict(endpoint=[4.125,.125,1.125],source='lidar_measured_beyond_range')])
            self.assertEqual(client.request('query',points=[[4.125,.125,1.125]])['cells'][0]['state'],'free')

    def cache(self):
        self.identity=dict(scene_id='test',scene_epoch='epoch',content_version='v1',frame_id='map',resolution=.25)
        return MapCache(dict(**self.identity,revision=1,cells=[dict(key=[0,0,0],state='free',log_odds=-1.,last_observed_sim_time=2.)]))

    def delta(self,**fields):
        return dict(**self.identity,base_revision=1,revision=2,changed_cells=[],refreshes=[],**fields)

    def test_invalid_refresh_does_not_partially_mutate(self):
        cache=self.cache();before=cache.snapshot()
        d=self.delta();d['changed_cells']=[dict(key=[1,0,0],state='occupied',log_odds=1.,last_observed_sim_time=3.)]
        d['refreshes']=[dict(capture_sim_time=3.,keys=[[0,0,0]]),dict(capture_sim_time=4.,keys=[[9,0,0]])]
        with self.assertRaisesRegex(ValueError,'snapshot_required'):cache.apply(d)
        self.assertEqual(cache.snapshot(),before)

    def test_refresh_replacement_uses_committed_cell_and_newest_time(self):
        cache=self.cache();d=self.delta();d['changed_cells']=[dict(key=[0,0,0],state='occupied',log_odds=1.,last_observed_sim_time=3.)]
        d['refreshes']=[dict(capture_sim_time=5.,keys=[[0,0,0]]),dict(capture_sim_time=4.,keys=[[0,0,0]])]
        cache.apply(d);self.assertEqual(cache.cells[(0,0,0)]['state'],'occupied');self.assertEqual(cache.cells[(0,0,0)]['last_observed_sim_time'],5.)

    def test_unknown_replacement_cannot_be_refreshed(self):
        cache=self.cache();d=self.delta();d['changed_cells']=[dict(key=[0,0,0],state='unknown',log_odds=None,last_observed_sim_time=None)];d['refreshes']=[dict(capture_sim_time=5.,keys=[[0,0,0]])]
        with self.assertRaisesRegex(ValueError,'snapshot_required'):cache.apply(d)
        self.assertEqual(cache.cells[(0,0,0)]['state'],'free')
