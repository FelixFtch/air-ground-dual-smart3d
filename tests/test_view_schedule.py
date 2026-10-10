"""Display scheduling contracts: never infer navigation or tree geometry."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('view', ROOT/'observability/ros_view.py')
view=importlib.util.module_from_spec(spec);spec.loader.exec_module(view)


class ViewSchedulingTests(unittest.TestCase):
    def robot(self):
        return {'robot_id':'go2_01','position_xyz':[1.,2.,.33],
                'body_half_extents_m':[.43,.43,.28],
                'planner':{'session_id':'e-go2','tree_instance_id':1,'received_wall':100.,
                           'path_id':'e:0:1','tree_exported':True,
                           'tree_nodes':[{'id':0,'xyz':[0,0,.33],'parent':None},
                                         {'id':1,'xyz':[1,0,.33],'parent':0}],
                           'tree_edges':[[1,0]],'path_xyz':[[0,0,.33],[1,0,.33]]}}

    def test_fast_path_does_not_visit_tree_edges(self):
        state={'epoch':'e','robots':[self.robot()]}
        with patch.object(view,'actual_tree_edges',side_effect=AssertionError('tree traversed')):
            specs,tf=view.scene_specs(state,None,include_trees=False)
        self.assertEqual(specs['trees'],[])
        self.assertEqual(len(specs['paths']),1)
        self.assertEqual(tf[0]['position'],(1.,2.,.33))

    def test_tree_identity_independent_of_body_and_sim_clock(self):
        state={'epoch':'e','sim_time':1.,'robots':[self.robot()]}
        before=view.tree_snapshot_signature(state)
        state['sim_time']=2.;state['robots'][0]['position_xyz']=[2.,2.,.33]
        self.assertEqual(before,view.tree_snapshot_signature(state))
        state['robots'][0]['planner']['received_wall']=101.
        self.assertEqual(before,view.tree_snapshot_signature(state))
        state['robots'][0]['planner']['tree_nodes'][1]['xyz']=[2,0,.33]
        self.assertNotEqual(before,view.tree_snapshot_signature(state))

    def test_tree_identity_changes_on_epoch_or_disappearing_robot(self):
        state={'epoch':'e','robots':[self.robot()]}
        before=view.tree_snapshot_signature(state)
        state['epoch']='next';self.assertNotEqual(before,view.tree_snapshot_signature(state))
        state['epoch']='e';state['robots']=[]
        self.assertNotEqual(before,view.tree_snapshot_signature(state))
        self.assertIsNone(view.tree_snapshot_signature(None))

    def test_atomic_replacement_same_size_and_mtime_invalidates_cache(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'state.json';path.write_text('{"n":1}')
            cache=view.AtomicJsonCache();first=cache.read(path)
            self.assertIs(first,cache.read(path))
            stamp=path.stat().st_mtime_ns
            replacement=Path(folder)/'next';replacement.write_text('{"n":2}')
            os.utime(replacement,ns=(stamp,stamp));replacement.replace(path)
            self.assertEqual(cache.read(path),{'n':2})

    def test_bad_snapshot_does_not_poison_cache(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'state.json';path.write_text('{"n":1}')
            cache=view.AtomicJsonCache();cache.read(path)
            replacement=Path(folder)/'bad';replacement.write_text('{broken');replacement.replace(path)
            with self.assertRaises(ValueError):cache.read(path)
            replacement=Path(folder)/'fixed';replacement.write_text('{"n":3}');replacement.replace(path)
            self.assertEqual(cache.read(path),{'n':3})

    def test_fast_snapshot_does_not_read_or_parse_map(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'shared').mkdir()
            (root/'latest.json').write_text(json.dumps({'run_root':folder}))
            (root/'state.json').write_text(json.dumps({'epoch':'e','scene_id':'lab'}))
            (root/'shared/display.json').write_text('{incomplete')
            cache=view.AtomicJsonCache()
            state,display,issues=view.read_runtime(root/'latest.json',cache=cache,include_display=False)
            self.assertIsNone(display);self.assertEqual(issues,[])
            self.assertEqual(state['epoch'],'e')
            self.assertNotIn(root/'shared/display.json',cache.entries)
            self.assertNotIn('_view_run_root',cache.entries[root/'state.json'][1])


class MarkerDeltaTests(unittest.TestCase):
    def test_startup_clears_prior_publisher_once_then_updates_in_place(self):
        tracker=view.MarkerDeltaTracker()
        keys=[('shared/occupied_floor',0),('shared/occupied_obstacles',0)]
        self.assertEqual(tracker.update(keys,'e'),(True,()))
        self.assertEqual(tracker.update(keys,'e'),(False,()))
        self.assertEqual(tracker.update(keys,'e'),(False,()))

    def test_disappeared_geometry_is_deleted_without_clearing_other_markers(self):
        tracker=view.MarkerDeltaTracker()
        tracker.update([('floor',0),('obstacle',0)],'e')
        self.assertEqual(tracker.update([('floor',0)],'e'),(False,(('obstacle',0),)))
        self.assertEqual(tracker.update([('floor',0)],'e'),(False,()))

    def test_empty_or_invalid_scene_removes_all_old_geometry_once(self):
        tracker=view.MarkerDeltaTracker()
        tracker.update([('tree/main',0),('tree/disconnected',0)],'e')
        self.assertEqual(tracker.update([],None),
                         (False,(('tree/disconnected',0),('tree/main',0))))
        self.assertEqual(tracker.update([],None),(False,()))

    def test_new_epoch_deletes_even_reused_ids_before_new_adds(self):
        tracker=view.MarkerDeltaTracker()
        tracker.update([('robot',0),('goal',0)],'old')
        self.assertEqual(tracker.update([('robot',0)],'new'),
                         (False,(('goal',0),('robot',0))))
        self.assertEqual(tracker.update([('robot',0)],'new'),(False,()))

    def test_topics_do_not_share_marker_identity(self):
        trees=view.MarkerDeltaTracker();maps=view.MarkerDeltaTracker()
        trees.update([('same',0)],'e');maps.update([('same',0)],'e')
        self.assertEqual(trees.update([],'e'),(False,(('same',0),)))
        self.assertEqual(maps.update([('same',0)],'e'),(False,()))

    def test_tree_visibility_changes_no_geometry_or_connectivity(self):
        robot=ViewSchedulingTests().robot()
        robot['planner']['tree_nodes'].append(
            {'id':2,'xyz':[2,0,.33],'parent':1,'tree_id':2})
        robot['planner']['tree_edges'].append([2,1])
        groups,_=view.scene_specs({'robots':[robot]},None)
        trees={s['ns']:s for s in groups['trees']}
        main=trees['go2_01/tree/main'];disconnected=trees['go2_01/tree/disconnected']
        self.assertEqual(main['points'],[(1.,0.,.33),(0.,0.,.33)])
        self.assertEqual(disconnected['points'],[(2.,0.,.33),(1.,0.,.33)])
        self.assertEqual((main['scale'][0],main['alpha']),(.025,.35))
        self.assertEqual((disconnected['scale'][0],disconnected['alpha']),(.02,.20))
