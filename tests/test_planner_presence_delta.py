"""Planner-only evidence-presence deltas retain geometry and restore semantics.

These messages intentionally do not certify timestamp freshness. The execution
cache continues consuming full integrate deltas.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from mapping.client import MapCache,MapError,OctomapClient

ROOT=Path(__file__).resolve().parents[1]
BINARY=Path(os.environ.get('OCTOMAP_SERVICE_BIN',ROOT/'mapping/build/octomap_service'))
IDENTITY=dict(scene_id='presence',content_version='v1',frame_id='map',resolution=.25)

class PresenceDeltaTests(unittest.TestCase):
    def setUp(self):
        self.mapper=OctomapClient(BINARY)
        self.mapper.request('init',**IDENTITY,scene_epoch='before',prob_hit=.9,prob_miss=.1,clamp_min=.12,clamp_max=.88,history_limit=3)
    def tearDown(self):self.mapper.close()
    def scan(self,sequence,*,mapper=None,epoch='before',endpoint=.875):
        return (mapper or self.mapper).request('integrate',scene_id=IDENTITY['scene_id'],scene_epoch=epoch,robot_id='r',sensor_id='lidar',frame_seq=sequence,capture_sim_time=float(sequence),receive_wall_time=100.,calibration_id='v1',origin=[.125,.125,.375],points=[[endpoint,.125,.375]],max_range_m=10,compact=True)
    def changes(self,since,mapper=None,**extra):
        return (mapper or self.mapper).request('changes',since_revision=since,compact=True,refresh_semantics='presence_only',**extra)
    def test_default_refreshes_preserved_while_presence_view_omits_repeats(self):
        first=self.scan(1);cache=MapCache(self.mapper.request('snapshot'))
        full=self.scan(2);cache.apply(full)
        self.assertTrue(full['refreshes']);self.assertNotIn('refresh_semantics',full)
        d=self.changes(first['revision']);self.assertEqual(d['changed_cells'],[]);self.assertEqual(d['refreshes'],[]);self.assertEqual(d['refresh_semantics'],'presence_only')
        normal=self.mapper.request('changes',since_revision=first['revision'],compact=True)
        self.assertTrue(normal['refreshes']);self.assertNotIn('refresh_semantics',normal)
        self.assertEqual(cache.cells[(1,0,1)]['last_observed_sim_time'],2.)
        self.assertEqual(d['revision'],full['revision'])
    def test_restore_first_same_state_observation_is_classified(self):
        self.scan(1)
        with tempfile.TemporaryDirectory() as folder:
            path=str(Path(folder)/'map.ot');self.mapper.request('save',path=path)
            with OctomapClient(BINARY) as restored:
                restored.request('init',**IDENTITY,scene_epoch='after')
                old=restored.request('load',path=path)
                self.assertTrue(all(c['last_observed_sim_time'] is None for c in old['cells']))
                first=self.scan(1,mapper=restored,epoch='after')
                presence=self.changes(old['revision'],restored)
                free=next(c for c in presence['changed_cells'] if c['key']==[1,0,1])
                self.assertEqual(free['state'],'free');self.assertEqual(free['last_observed_sim_time'],1.)
                second=self.scan(2,mapper=restored,epoch='after')
                repeated=self.changes(first['revision'],restored)
                self.assertEqual(repeated['changed_cells'],[]);self.assertEqual(repeated['refreshes'],[])
                self.assertTrue(second['refreshes'])
    def test_mark_clear_forget_readd_are_retained(self):
        first=self.scan(1);hit=self.scan(2,endpoint=.375)
        d=self.changes(first['revision']);self.assertEqual(next(c for c in d['changed_cells'] if c['key']==[1,0,1])['state'],'occupied')
        cleared=self.scan(3);d=self.changes(hit['revision']);self.assertEqual(next(c for c in d['changed_cells'] if c['key']==[1,0,1])['state'],'free')
        forgotten=self.mapper.request('forget',scene_id='presence',scene_epoch='before',keys=[[1,0,1]],reason='regression',compact=True)
        d=self.changes(cleared['revision']);self.assertEqual(next(c for c in d['changed_cells'] if c['key']==[1,0,1])['state'],'unknown')
        self.scan(4);d=self.changes(forgotten['revision']);self.assertEqual(next(c for c in d['changed_cells'] if c['key']==[1,0,1])['state'],'free')
    def test_gap_and_bad_view_are_rejected(self):
        for i in range(1,6):self.scan(i)
        with self.assertRaises(MapError) as exc:self.changes(0)
        self.assertEqual(exc.exception.code,'snapshot_required')
        for spec in [dict(compact=True,refresh_semantics='arbitrary'),dict(compact=False,refresh_semantics='presence_only')]:
            with self.assertRaises(MapError) as exc:self.mapper.request('changes',since_revision=5,**spec)
            self.assertEqual(exc.exception.code,'invalid_refresh_semantics')
    def test_native_accepts_marker_and_first_observation_in_both_dimensions(self):
        native=ROOT/'planning/target/release/air-ground-smart3d-worker'
        if not native.exists():self.skipTest('Build native planner first')
        self.scan(1)
        with tempfile.TemporaryDirectory() as folder:
            path=str(Path(folder)/'map.ot');self.mapper.request('save',path=path)
            with OctomapClient(BINARY) as restored:
                restored.request('init',**IDENTITY,scene_epoch='after')
                old=restored.request('load',path=path)
                first=self.scan(1,mapper=restored,epoch='after');d1=self.changes(old['revision'],restored)
                self.scan(2,mapper=restored,epoch='after');d2=self.changes(first['revision'],restored)
            for dimension in (2,3):
                with self.subTest(dimension=dimension):
                    start=[.35,.125,.375][:dimension];goal=[.45,.125,.375][:dimension]
                    init=dict(protocol='air_ground_smart3d.v1',op='init',robot_id='r',session_id='presence',dimension=dimension,goal_revision=0,start=start,goal=goal,ranges=[[-1,1],[-1,1],[-.5,1]][:dimension],profile=dict(profile_id='presence',body_half_extents_m=[.01]*3,body_offset_m=[0]*3,planar_z=.375),map_snapshot=old,sim_time=3.,initial_iterations=100,initial_budget_ms=1000,include_tree=False)
                    base=dict(op='update',robot_id='r',session_id='presence',goal_revision=0,current=start,sim_time=3.,include_tree=False)
                    result=subprocess.run([str(native)],input='\n'.join(json.dumps(x) for x in [init,dict(base,map_delta=d1),dict(base,map_delta=d2)])+'\n',text=True,capture_output=True,timeout=15)
                    self.assertEqual(result.returncode,0,result.stderr)
                    rows=[json.loads(x) for x in result.stdout.splitlines()];self.assertEqual(len(rows),3)
                    for row in rows:self.assertTrue(row['ok'],row)
                    self.assertFalse(rows[0]['path_all_known_free'])
                    self.assertTrue(rows[1]['path_all_known_free']);self.assertTrue(rows[2]['path_all_known_free'])
                    self.assertEqual(rows[0]['tree_instance_id'],rows[2]['tree_instance_id'])
