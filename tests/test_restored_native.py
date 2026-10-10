"""Real OcTree save/load snapshot admission by native SMART2 and SMART3."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from mapping.client import OctomapClient
from execution.guard import check_path_segment

ROOT=Path('/data/robotics/air_ground_smart3d')


class RestoredNativeMap(unittest.TestCase):
    def test_real_restore_preserves_obstacles_and_uncertified_free_in_both_dimensions(self):
        binary=Path(os.environ.get('AIR_GROUND_TEST_WORKER',str(ROOT/'planning/target/release/air-ground-smart3d-worker')))
        mapper_binary=ROOT/'mapping/build/octomap_service'
        if not binary.exists() or not mapper_binary.exists():
            self.skipTest('Build mapper and native worker first')
        for dimension in (2,3):
            with self.subTest(dimension=dimension), tempfile.TemporaryDirectory(prefix='restored-native-') as folder:
                z=.375 if dimension==2 else 1.125
                identity=dict(scene_id='restore-fixture',content_version='geometry-1',frame_id='map',resolution=.25)
                path=str(Path(folder)/'scene.ot')
                with OctomapClient(mapper_binary) as mapper:
                    mapper.request('init',**identity,scene_epoch='before')
                    metadata=dict(scene_id=identity['scene_id'],scene_epoch='before',robot_id='source',sensor_id='lidar',calibration_id='fixture',origin=[-2.125,.125,z],max_range_m=10,receive_wall_time=1.)
                    mapper.request('integrate',**metadata,frame_seq=1,capture_sim_time=1.,points=[[.125,.125,z]])
                    mapper.request('save',path=path)
                with OctomapClient(mapper_binary) as restored:
                    restored.request('init',**identity,scene_epoch='after')
                    snapshot=restored.request('load',path=path)
                    self.assertTrue(snapshot['cells'])
                    self.assertTrue(all(c['last_observed_sim_time'] is None for c in snapshot['cells']))
                    occupied=[c for c in snapshot['cells'] if c['state']=='occupied']
                    free=[c for c in snapshot['cells'] if c['state']=='free']
                    self.assertTrue(occupied);self.assertTrue(free)
                    profile=dict(profile_id='restored-test',body_half_extents_m=[.1,.1,.1],body_offset_m=[0,0,0],planar_z=z)
                    request=dict(protocol='air_ground_smart3d.v1',op='init',robot_id='r',session_id='restore',dimension=dimension,goal_revision=0,
                        start=[-2.,0.,z][:dimension],goal=[2.,0.,z][:dimension],ranges=[[-3.,3.],[-3.,3.],[.5,3.]][:dimension],profile=profile,
                        map_snapshot=snapshot,sim_time=2.,initial_iterations=350,initial_budget_ms=1000)
                    blocked=dict(request,goal=[.125,.125,z][:dimension])
                    result=subprocess.run([str(binary)],input='\n'.join(json.dumps(q) for q in [request,blocked])+'\n',text=True,capture_output=True,timeout=20)
                    self.assertEqual(result.returncode,0,result.stderr)
                    accepted,rejected=[json.loads(line) for line in result.stdout.splitlines()]
                    self.assertTrue(accepted['ok'],accepted)
                    self.assertEqual(accepted['metrics']['invalid_tree_edges'],0)
                    self.assertFalse(accepted['path_all_known_free'])
                    self.assertFalse(rejected['ok'],rejected)
                    self.assertEqual(rejected['reason'],'goal_blocked_or_out_of_bounds')
                    # A null timestamp in saved free space remains ineligible
                    # for new execution travel, including retain-free policy.
                    cell=free[len(free)//2];center=[(k+.5)*.25 for k in cell['key']]
                    end=[center[0]+.05,center[1],center[2]]
                    guard=check_path_segment({tuple(cell['key']):cell},.25,center,end,[.01]*3,[0]*3,2.,max_free_age=None)
                    self.assertFalse(guard['ok']);self.assertEqual(guard['reason'],'free_time_missing')


if __name__=='__main__':unittest.main()
