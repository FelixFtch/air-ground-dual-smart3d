"""Long-lived voxel cache should not force repeated scans of key lists."""
import gc
import json
import sys
import unittest
from mapping.client import MapCache

class MapCacheRepresentationTests(unittest.TestCase):
    def snapshot(self):
        return dict(scene_id='map',scene_epoch='epoch',content_version='v1',frame_id='map',resolution=.25,revision=1,cells=[dict(key=[1,2,3],state='free',log_odds=-1.,last_observed_sim_time=2.)])

    def test_immutable_cache_keys_keep_exact_wire_snapshot(self):
        original=self.snapshot();cache=MapCache(original);key=next(iter(cache.cells));self.assertIs(cache.cells[key]['key'],key)
        self.assertEqual(cache.snapshot(),original)
        self.assertEqual(json.loads(json.dumps(cache.snapshot())),original)
        original['cells'][0]['key'][0]=99
        self.assertEqual(cache.cells[key]['key'],(1,2,3))
        detached=cache.snapshot();detached['cells'][0]['state']='occupied';detached['cells'][0]['key'][0]=77
        self.assertEqual(cache.cells[key]['state'],'free');self.assertEqual(cache.cells[key]['key'],(1,2,3))

    def test_changed_cells_use_same_scalar_representation(self):
        original=self.snapshot();cache=MapCache(original)
        delta={k:v for k,v in original.items() if k not in ('revision','cells')};delta.update(base_revision=1,revision=2,changed_cells=[dict(key=[4,5,6],state='occupied',log_odds=2.,last_observed_sim_time=3.)])
        cache.apply(delta);self.assertEqual(cache.cells[(4,5,6)]['key'],(4,5,6));self.assertIsInstance(cache.cells[(4,5,6)]['key'],tuple)
        delta['changed_cells'][0]['key'][0]=99
        self.assertEqual(cache.cells[(4,5,6)]['key'],(4,5,6))
        if sys.implementation.name=='cpython':
            gc.collect(2);gc.collect(2)
            self.assertFalse(gc.is_tracked(cache.cells[(4,5,6)]))
            self.assertFalse(gc.is_tracked(cache.cells[(1,2,3)]))


def test_execution_cache_rejects_presence_only_delta():
    from mapping.client import MapCache
    import pytest
    identity=dict(scene_id="s",scene_epoch="e",content_version="v",frame_id="map",resolution=.25)
    cache=MapCache(dict(**identity,revision=0,cells=[]))
    with pytest.raises(ValueError,match="execution_cache_requires_full_freshness"):
        cache.apply(dict(**identity,base_revision=0,revision=1,changed_cells=[],refreshes=[],refresh_semantics="presence_only"))
    assert cache.revision==0
