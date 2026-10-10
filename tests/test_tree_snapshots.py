"""Separate display transport preserves native geometry and rejects mixed trees."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from observability import ros_view as view
from observability import tree_snapshots as trees


def planner(session='epoch-go2', instance=1):
    return {'ok':True,'robot_id':'go2_01','session_id':session,'tree_instance_id':instance,
            'received_wall':100.,'tree_exported':True,'path_id':'a:1',
            'path_xyz':[[0,0,.33],[1,0,.33]],
            'tree_nodes':[{'id':0,'xyz':[0,0,.33],'parent':None,'tree_id':0},
                          {'id':1,'xyz':[1,0,.33],'parent':0,'tree_id':0}],
            'tree_edges':[[1,0]]}


def state(root, p, **kw):
    value={'epoch':'epoch','running':True,'wall_time':100.,'_view_run_root':str(root),
           'robots':[{'robot_id':'go2_01','position_xyz':[0,0,.33],
                      'body_half_extents_m':[.43,.43,.28],'planner':p}]}
    value.update(kw)
    return value


class TreeSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.writer=trees.TreeSnapshotWriter(self.root,'epoch')
        self.reader=trees.TreeSnapshotReader()

    def tearDown(self):
        self.temp.cleanup()

    def hydrate(self, p, now=100., mono=10., **kwargs):
        return self.reader.hydrate(state(self.root,p,**kwargs),now_wall=now,now_monotonic=mono)

    def test_geometry_and_markers_equal_with_compact_paths_untouched(self):
        source=planner();before=copy.deepcopy(source)
        small=self.writer.compact('go2_01',source)
        self.assertNotIn('tree_nodes',small);self.assertNotIn('tree_edges',small)
        self.assertEqual((small['tree_nodes_count'],small['tree_edges_count']),(2,1))
        self.assertEqual(small['path_xyz'],source['path_xyz'])
        hydrated,issues=self.hydrate(small)
        self.assertEqual(issues,[])
        self.assertEqual(view.scene_specs(state(self.root,source),None),view.scene_specs(hydrated,None))
        self.assertEqual(source,before)
        ref=small['tree_ref'];data=(self.root/ref['path']).read_bytes()
        self.assertEqual(ref['sha256'],hashlib.sha256(data).hexdigest())

    def test_same_object_does_not_hash_or_encode_twice(self):
        source=planner();first=self.writer.compact('go2_01',source)
        with patch.object(trees,'_encode',side_effect=AssertionError('encoded twice')):
            again=self.writer.compact('go2_01',source)
        self.assertEqual(first,again)

    def test_unchanged_geometry_ignores_new_path_timestamp_and_reuses_file(self):
        source=planner();first=self.writer.compact('go2_01',source)
        newer=copy.deepcopy(source);newer.update(received_wall=101.,path_id='a:2',path_xyz=[[0,0,.33]])
        second=self.writer.compact('go2_01',newer)
        self.assertEqual(first['tree_ref'],second['tree_ref'])
        self.assertEqual(len(list(self.root.rglob('tree-*.json'))),1)
        self.assertEqual(view.tree_snapshot_signature(state(self.root,first)),view.tree_snapshot_signature(state(self.root,second)))

    def test_deleted_edge_changes_version_and_marker_disappears(self):
        first=self.writer.compact('go2_01',planner());before,_=self.hydrate(first)
        source=planner();source['tree_edges']=[];source['tree_nodes'][1]['parent']=None
        after=self.writer.compact('go2_01',source);hydrated,_=self.hydrate(after)
        self.assertNotEqual(first['tree_ref']['geometry_sha256'],after['tree_ref']['geometry_sha256'])
        tracker=view.MarkerDeltaTracker()
        old,_=view.scene_specs(before,None);new,_=view.scene_specs(hydrated,None)
        tracker.update([(s['ns'],0) for s in old['trees']],'epoch')
        self.assertEqual(tracker.update([(s['ns'],0) for s in new['trees']],'epoch'),(False,(('go2_01/tree/main',0),)))

    def test_new_session_and_tree_instance_are_distinct_even_when_geometry_same(self):
        first=self.writer.compact('go2_01',planner())
        second=self.writer.compact('go2_01',planner(session='recovery',instance=2))
        self.assertEqual(first['tree_ref']['geometry_sha256'],second['tree_ref']['geometry_sha256'])
        self.assertNotEqual(first['tree_ref']['sha256'],second['tree_ref']['sha256'])
        self.assertNotEqual(view.tree_snapshot_signature(state(self.root,first)),view.tree_snapshot_signature(state(self.root,second)))

    def test_hash_mismatch_is_rejected_and_corrected_version_retried(self):
        small=self.writer.compact('go2_01',planner());path=self.root/small['tree_ref']['path'];data=path.read_bytes()
        path.write_bytes(data+b' ')
        hydrated,issues=self.hydrate(small)
        self.assertTrue(issues);self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])
        self.assertFalse(self.reader.entries)
        path.write_bytes(data)
        hydrated,issues=self.hydrate(small)
        self.assertFalse(issues);self.assertEqual(len(hydrated['robots'][0]['planner']['tree_nodes']),2)

    def test_reference_identity_mismatch_cannot_display_cached_geometry(self):
        small=self.writer.compact('go2_01',planner());self.hydrate(small)
        for field,bad in [('epoch','other'),('robot_id','uav_01'),('session_id','old'),('tree_instance_id',9)]:
            forged=copy.deepcopy(small);forged['tree_ref'][field]=bad
            hydrated,issues=self.hydrate(forged)
            self.assertTrue(issues);self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])

    def test_document_identity_mismatch_rejected_even_with_correct_full_hash(self):
        small=self.writer.compact('go2_01',planner());ref=small['tree_ref']
        document=json.loads((self.root/ref['path']).read_bytes());document['session_id']='wrong'
        data=trees._encode(document);digest=hashlib.sha256(data).hexdigest()
        ref.update(sha256=digest,path=f'shared/tree_snapshots/go2_01/tree-{digest}.json')
        (self.root/ref['path']).write_bytes(data)
        hydrated,issues=self.hydrate(small)
        self.assertTrue(issues);self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])

    def test_absolute_parent_wrong_robot_and_symlink_paths_rejected(self):
        small=self.writer.compact('go2_01',planner())
        for path in ('/tmp/tree.json','../tree.json','shared/tree_snapshots/uav_01/tree-a.json',
                     'shared/tree_snapshots/go2_01/../../tree.json'):
            forged=copy.deepcopy(small);forged['tree_ref']['path']=path
            hydrated,issues=self.hydrate(forged)
            self.assertTrue(issues);self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])
        path=self.root/small['tree_ref']['path'];payload=path.read_bytes();path.unlink()
        outside=self.root.parent/(self.root.name+'-outside.json')
        try:
            outside.write_bytes(payload);path.symlink_to(outside)
            hydrated,issues=self.hydrate(small)
            self.assertTrue(issues);self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])
        finally:
            outside.unlink(missing_ok=True)

    def test_atomic_missing_version_keeps_whole_old_tree_only_three_seconds(self):
        first=self.writer.compact('go2_01',planner());self.hydrate(first,mono=0.)
        source=planner();source['tree_nodes'][1]['xyz']=[2,0,.33]
        second=self.writer.compact('go2_01',source);path=self.root/second['tree_ref']['path'];data=path.read_bytes();path.unlink()
        held,issues=self.hydrate(second,mono=1.)
        self.assertTrue(issues);self.assertEqual(held['robots'][0]['planner']['tree_ref'],first['tree_ref'])
        hidden,_=self.hydrate(second,mono=4.01)
        self.assertFalse(hidden['robots'][0]['planner']['tree_nodes'])
        path.write_bytes(data)
        updated,issues=self.hydrate(second,mono=5.)
        self.assertFalse(issues);self.assertEqual(updated['robots'][0]['planner']['tree_nodes'][1]['xyz'],[2,0,.33])

    def test_missing_new_session_does_not_reuse_old_tree(self):
        first=self.writer.compact('go2_01',planner());self.hydrate(first)
        second=self.writer.compact('go2_01',planner(session='recovery'))
        (self.root/second['tree_ref']['path']).unlink()
        hydrated,_=self.hydrate(second)
        self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])

    def test_explicit_failure_or_stale_publisher_never_uses_grace(self):
        first=self.writer.compact('go2_01',planner());self.hydrate(first)
        for change in ({'ok':False},{'tree_exported':False}):
            failed=dict(first,**change);hydrated,_=self.hydrate(failed)
            self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])
        for change in ({'fatal':'panic'},{'running':False},{'_view_state_age_s':3.1}):
            hydrated,_=self.hydrate(first,**change)
            self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])
        source=state(self.root,first);source['robots'][0]['planner_error']='panic'
        hydrated,_=self.reader.hydrate(source,now_wall=100.)
        self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])
        hydrated,_=self.hydrate(first,now=104.)
        self.assertFalse(hydrated['robots'][0]['planner']['tree_nodes'])

    def test_empty_and_failed_planner_clear_reference(self):
        self.writer.compact('go2_01',planner())
        for source in ({},None,dict(planner(),ok=False),dict(planner(),tree_exported=False)):
            result=self.writer.compact('go2_01',source)
            self.assertIsNone(result['tree_ref']);self.assertFalse(self.writer.entries)

    def test_disk_failure_does_not_change_control_path(self):
        source=planner()
        with patch.object(trees.os,'replace',side_effect=OSError('disk')):
            result=self.writer.compact('go2_01',source)
        self.assertIsNone(result['tree_ref']);self.assertEqual(result['path_xyz'],source['path_xyz'])
        self.assertIn('OSError',result['tree_snapshot_error'])

    def test_immutable_file_cache_does_not_read_or_decode_again(self):
        small=self.writer.compact('go2_01',planner());self.hydrate(small)
        with patch.object(Path,'open',side_effect=AssertionError('opened again')):
            hydrated,issues=self.hydrate(small)
        self.assertFalse(issues);self.assertTrue(hydrated['robots'][0]['planner']['tree_nodes'])

    def test_rolling_files_protect_current_and_recent_reader_versions(self):
        now=[time.time()];writer=trees.TreeSnapshotWriter(self.root,'epoch',retain_versions=2,grace_s=6,clock=lambda:now[0])
        for i in range(8):
            source=planner();source['tree_nodes'][1]['xyz']=[i,0,.33]
            result=writer.compact('go2_01',source)
        self.assertEqual(len(list(self.root.rglob('tree-*.json'))),8)
        now[0]+=10
        writer.compact('go2_01',source)
        self.assertLessEqual(len(list(self.root.rglob('tree-*.json'))),3)  # tied mtimes may retain current separately
        self.assertTrue((self.root/result['tree_ref']['path']).is_file())

    def test_memory_cache_is_bounded_and_disappeared_robot_is_removed(self):
        writer=trees.TreeSnapshotWriter(self.root,'epoch',max_cached_robots=2)
        reader=trees.TreeSnapshotReader(max_cached_robots=2)
        robots=[]
        for i in range(5):
            rid=f'robot_{i}';source=planner();source['robot_id']=rid
            small=writer.compact(rid,source);robots.append(dict(robot_id=rid,planner=small))
        reader.hydrate(dict(state(self.root,{}),robots=robots),now_wall=100.)
        self.assertEqual(len(writer.entries),2);self.assertEqual(len(reader.entries),2)
        reader.hydrate(dict(state(self.root,{}),robots=[]),now_wall=100.)
        self.assertFalse(reader.entries)

    def test_legacy_inline_geometry_still_works_and_timestamp_does_not_change_it(self):
        old=planner();hydrated,issues=self.hydrate(old)
        self.assertFalse(issues);self.assertEqual(hydrated['robots'][0]['planner']['tree_edges'],old['tree_edges'])
        first=view.tree_snapshot_signature(hydrated)
        old['received_wall']=101.;old['path_id']='different'
        hydrated,_=self.hydrate(old)
        self.assertEqual(first,view.tree_snapshot_signature(hydrated))
        old['tree_nodes'][1]['xyz']=[3,0,.33]
        self.assertNotEqual(first,view.tree_snapshot_signature(state(self.root,old)))
