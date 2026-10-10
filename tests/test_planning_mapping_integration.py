"""Protocol integration fixture with real OcTree + native SMART executables.
No simulator ground truth or navigation acceptance claim; rays are test inputs.
"""
import json
from pathlib import Path
import subprocess
import unittest
from mapping.client import OctomapClient

ROOT = Path(__file__).resolve().parents[1] if Path(__file__).resolve().parent.name == 'tests' else Path('/data/robotics/air_ground_smart3d')

class MappingPlannerIntegration(unittest.TestCase):
    def test_real_octree_compact_deltas_keep_native_tree(self):
        worker_bin = ROOT/'planning/target/release/air-ground-smart3d-worker'
        mapper_bin = ROOT/'mapping/build/octomap_service'
        if not worker_bin.exists() or not mapper_bin.exists():
            self.skipTest('Build mapper and planning worker first')
        planner = subprocess.Popen([str(worker_bin)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        def request(payload):
            planner.stdin.write(json.dumps(payload)+'\n');planner.stdin.flush()
            response=json.loads(planner.stdout.readline())
            self.assertTrue(response['ok'],response)
            self.assertEqual(response['metrics']['invalid_tree_edges'],0)
            return response
        try:
            with OctomapClient(mapper_bin) as mapper:
                mapper.request('init',scene_id='integration',scene_epoch='test-epoch',content_version='fixture-v1',frame_id='map',resolution=.25)
                spec=json.loads((ROOT/'planning/examples/uav.jsonl').read_text().splitlines()[0])
                spec.update(map_snapshot=mapper.request('snapshot'),start=[-2,.125,1.125],goal=[2,.125,1.125],ranges=[[-3,3],[-3,3],[.5,3]],initial_iterations=350,initial_budget_ms=1000)
                spec['profile']['body_half_extents_m']=[.12,.12,.12]
                first=request(spec);self.assertIsInstance(first['path_xyz'],list)
                metadata=dict(scene_id='integration',scene_epoch='test-epoch',robot_id='sensor-robot',sensor_id='fixture-lidar',calibration_id='fixture-v1',origin=[-2.125,.125,1.125],max_range_m=10,receive_wall_time=1.,compact=True)
                delta=mapper.request('integrate',**metadata,frame_seq=1,capture_sim_time=1.,points=[[.125,.125,1.125]])
                base=dict(robot_id=spec['robot_id'],session_id=spec['session_id'],goal_revision=0,current=spec['start'],include_tree=False)
                added=request(dict(base,op='update',map_delta=delta))
                self.assertEqual(added['tree_instance_id'],first['tree_instance_id'])
                self.assertGreater(added['metrics']['occupied_added'],0)
                # Re-observe the same class: use the mapper's compact refresh path.
                delta=mapper.request('integrate',**metadata,frame_seq=2,capture_sim_time=2.,points=[[.125,.125,1.125]])
                self.assertGreater(len(delta['refreshes']),0)
                refresh=request(dict(base,op='update',map_delta=delta))
                self.assertEqual(refresh['map']['revision'],delta['revision'])
                last=refresh;clears=0
                for seq in range(3,10):
                    delta=mapper.request('integrate',**metadata,frame_seq=seq,capture_sim_time=float(seq),points=[],free_rays=[dict(endpoint=[2.125,.125,1.125],source='registered_depth_measured_beyond_range')])
                    last=request(dict(base,op='update',map_delta=delta))
                    clears+=last['metrics']['occupied_cleared']
                self.assertGreater(clears,0)
                self.assertEqual(last['tree_instance_id'],first['tree_instance_id'])
                self.assertIsInstance(last['path_xyz'],list)
                self.assertEqual(last['path_xyz'][0],last['current_xyz'])
                self.assertEqual(last['map']['revision'],mapper.request('status')['revision'])
        finally:
            planner.stdin.close()
            planner.wait(timeout=10)
            planner.stdout.close()

if __name__=='__main__':unittest.main()
