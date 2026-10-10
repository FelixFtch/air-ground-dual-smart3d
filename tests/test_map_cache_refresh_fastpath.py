"""Full-freshness refresh paths remain atomic and monotonically timed."""
import copy
import math
import random
import unittest
from mapping.client import MapCache

class CacheRefreshFastpathTests(unittest.TestCase):
    def cache(self):
        return MapCache(dict(scene_id='s',scene_epoch='e',content_version='v',frame_id='map',resolution=.25,revision=1,cells=[dict(key=[i,0,0],state='free',log_odds=-1.,last_observed_sim_time=(None if i%3==0 else 5.)) for i in range(30)]))
    def delta(self,cache,changes,groups):
        return dict(cache.identity,base_revision=cache.revision,revision=cache.revision+1,changed_cells=changes,refreshes=groups)
    def test_repeated_out_of_order_groups_preserve_maximum_with_or_without_changes(self):
        rng=random.Random(184)
        for changed in (False,True):
            with self.subTest(changed=changed):
                cache=self.cache();reference={tuple(c['key']):c for c in cache.snapshot()['cells']}
                changes=[dict(key=[4,0,0],state='occupied',log_odds=2.,last_observed_sim_time=3.)] if changed else []
                groups=[dict(capture_sim_time=rng.uniform(0,20),keys=[[rng.randrange(30),0,0] for _ in range(50)]) for _ in range(20)]
                for c in changes:reference[tuple(c['key'])]=copy.deepcopy(c)
                for group in groups:
                    for key in group['keys']:
                        cell=reference[tuple(key)];old=cell['last_observed_sim_time'];t=group['capture_sim_time'];cell['last_observed_sim_time']=t if old is None else max(t,old)
                d=self.delta(cache,changes,groups);before=copy.deepcopy(d);cache.apply(d)
                self.assertEqual(d,before)
                self.assertEqual({tuple(c['key']):c for c in cache.snapshot()['cells']},reference)
    def test_missing_late_refresh_is_atomic_in_both_paths(self):
        for changed in (False,True):
            cache=self.cache();before=cache.snapshot();changes=[dict(key=[4,0,0],state='occupied',log_odds=2.,last_observed_sim_time=3.)] if changed else []
            groups=[dict(capture_sim_time=9.,keys=[[i,0,0] for i in range(30)]),dict(capture_sim_time=10.,keys=[[99,0,0]])]
            with self.assertRaisesRegex(ValueError,'snapshot_required'):cache.apply(self.delta(cache,changes,groups))
            self.assertEqual(cache.snapshot(),before)
    def test_invalid_late_timestamp_does_not_commit_prior_refresh(self):
        for invalid in (float('nan'),float('inf'),'12'):
            cache=self.cache();before=cache.snapshot();groups=[dict(capture_sim_time=9.,keys=[[1,0,0]]),dict(capture_sim_time=invalid,keys=[[2,0,0]])]
            with self.assertRaisesRegex(ValueError,'invalid_refresh_time'):cache.apply(self.delta(cache,[],groups))
            self.assertEqual(cache.snapshot(),before)
