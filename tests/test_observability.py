import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("air_ground_ros_view",ROOT/"observability/ros_view.py")
view=importlib.util.module_from_spec(spec);spec.loader.exec_module(view)


class ObservabilityTests(unittest.TestCase):
    def planner(self):
        return {"tree_exported":True,"tree_nodes":[
            {"id":0,"xyz":[0,0,1],"parent":None,"tree_id":0},
            {"id":1,"xyz":[1,0,2],"parent":0,"tree_id":0},
            {"id":2,"xyz":[2,0,3],"parent":None,"tree_id":2}],
            "tree_edges":[[1,0]],"path_xyz":[[0,0,1],[1,0,2]]}

    def test_only_real_exported_parent_edges(self):
        planner=self.planner();planner["tree_edges"] += [[2,0],[1,99],[0,0],[1,0]]
        edges,omitted=view.actual_tree_edges(planner)
        self.assertEqual(edges["main"],[(1.,0.,2.),(0.,0.,1.)])
        self.assertEqual(edges["disconnected"],[])
        self.assertEqual(omitted,3)

    def test_omitted_tree_is_not_reconstructed(self):
        planner=self.planner();planner["tree_exported"]=False
        edges,_=view.actual_tree_edges(planner)
        self.assertFalse(any(edges.values()))

    def test_ground_projection_unknown_and_collision_precedence(self):
        result=view.occupancy_projection({"resolution":.25,"free_xy":[[-1,0],[1,0]],"blocked_xy":[[1,0]]})
        self.assertEqual(result["origin"],[-.25,0.,0.])
        self.assertEqual(result["data"],[0,-1,100])
        self.assertIsNone(view.occupancy_projection({"occupied":[[0,0,0]]}))

    def test_uav_height_and_robot_namespace_preserved(self):
        robots=[]
        for identifier,z in [("go2_01",.349),("uav_01",2.)]:
            robots.append({"robot_id":identifier,"position_xyz":[0,0,z],"yaw":.4,
                           "body_half_extents_m":[.4,.4,.3],"planner":self.planner()})
        specs,transforms=view.scene_specs({"robots":robots},None)
        self.assertEqual([t["child"] for t in transforms],["go2_01/base_link","uav_01/base_link"])
        self.assertEqual([m["ns"] for m in specs["paths"]],["go2_01/actual_path","uav_01/actual_path"])
        self.assertEqual(specs["paths"][1]["points"][-1][2],2.)

    def test_invalid_path_does_not_bridge_missing_point(self):
        robot={"robot_id":"uav_01","position_xyz":[0,0,2],"body_half_extents_m":[.4]*3,
               "planner":{"path_xyz":[[0,0,2],[float("nan"),0,2],[2,0,2]]}}
        specs,_=view.scene_specs({"robots":[robot]},None)
        self.assertEqual(specs["paths"],[])

    def test_null_path_keeps_real_tree_and_body(self):
        planner=self.planner();planner["path_xyz"]=None
        robot={"robot_id":"go2_01","position_xyz":[0,0,.349],"body_half_extents_m":[.43,.43,.28],"planner":planner}
        specs,transforms=view.scene_specs({"robots":[robot]},None)
        self.assertEqual(specs["paths"],[])
        self.assertEqual(len(specs["trees"]),1)
        self.assertEqual(len(transforms),1)
        planner["tree_nodes"]=None;planner["tree_edges"]=None
        specs,_=view.scene_specs({"robots":[robot]},None)
        self.assertEqual(specs["trees"],[])

    def test_robot_locator_starts_at_real_pose_and_floor_remains_occupied(self):
        pose=[3.,-2.,.33]
        robot={"robot_id":"go2_01","position_xyz":pose,"body_half_extents_m":[.43,.43,.28],"planner":self.planner()}
        specs,transforms=view.scene_specs({"robots":[robot]},
            {"resolution":.25,"ground_z":-.01875,"occupied":[[0,0,-2],[0,0,-1],[0,0,0]]})
        markers={m["ns"]:m for group in specs.values() for m in group}
        self.assertEqual(markers["go2_01/position_locator"]["points"],[tuple(pose),(3.,-2.,2.33)])
        self.assertEqual(markers["go2_01/name"]["text"],"GO2_01")
        self.assertEqual(markers["go2_01/body_envelope"]["position"],tuple(pose))
        self.assertEqual(transforms[0]["position"],tuple(pose))
        self.assertEqual(len(markers["shared/occupied_floor"]["points"]),2)
        self.assertEqual(markers["shared/occupied_obstacles"]["points"],[[.125,.125,.125]])

    def test_image_sources_are_bound_to_runtime_episode(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/"go2_01/sensor").mkdir(parents=True);(root/"uav_01").mkdir()
            go2=root/"go2_01/sensor/latest_rgb.jpg";go2.write_bytes(b"fixture")
            uav=root/"uav_01/latest_rgb.jpg";uav.write_bytes(b"fixture")
            state={"_view_run_root":folder,"robots":[{"robot_id":"go2_01"},{"robot_id":"uav_01"},{"robot_id":"../unsafe"}]}
            self.assertEqual(view.robot_image_files(state),{"go2_01":go2,"uav_01":uav})

    def test_lidar_capture_contract_and_finite_coordinates(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"latest_lidar.npz"
            points=np.array([[1.25,-2.5,3.75],[-4.,5.,6.]],dtype=np.float32)
            np.savez(path,points=points,frame=127,capture_sim_time=4.25)
            frame,stamp,actual=view.read_lidar_frame(path)
            self.assertEqual((frame,stamp),(127,4.25))
            self.assertEqual(actual.tobytes(),points.astype("<f4").tobytes())
            self.assertEqual(actual.shape,(2,3))
            points[0,0]=float("nan");np.savez(path,points=points,frame=128,capture_sim_time=4.3)
            with self.assertRaises(ValueError):view.read_lidar_frame(path)

    def test_cross_episode_map_hidden(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/"shared").mkdir()
            (root/"latest.json").write_text(json.dumps({"run_root":folder}))
            (root/"state.json").write_text(json.dumps({"epoch":"new","scene_id":"lab"}))
            (root/"shared/display.json").write_text(json.dumps({"scene_epoch":"old","occupied":[[0,0,0]]}))
            _,display,issues=view.read_runtime(root/"latest.json")
            self.assertIsNone(display)
            self.assertIn("map epoch mismatch; map hidden",issues)


if __name__=="__main__":unittest.main()
