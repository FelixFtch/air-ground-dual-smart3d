"""Lossless optional packed refresh coordinates; default protocol stays intact."""
import base64
import copy
import json
import os
from pathlib import Path
import struct
import tempfile
import unittest
from mapping.client import MapCache,MapError,OctomapClient

ROOT=Path(__file__).resolve().parents[1]
BINARY=Path(os.environ.get('OCTOMAP_SERVICE_BIN',ROOT/'mapping/build/octomap_service'))
IDENTITY=dict(scene_id='binary-refresh',scene_epoch='e',content_version='v1',frame_id='map',resolution=.25)

def packed(keys,stamp):
    return dict(capture_sim_time=stamp,key_count=len(keys),keys_i16le_b64=base64.b64encode(b''.join(struct.pack('<hhh',*k) for k in keys)).decode('ascii'))

def unpack(groups):
    result=[]
    for group in groups:
        if 'keys_i16le_b64' in group:
            result.append(dict(capture_sim_time=group['capture_sim_time'],keys=[list(k) for k in struct.iter_unpack('<hhh',base64.b64decode(group['keys_i16le_b64'],validate=True))]))
        else:result.append(group)
    return result

class BinaryRefreshTests(unittest.TestCase):
    def cache(self):
        return MapCache(dict(IDENTITY,revision=1,cells=[dict(key=k,state='free',log_odds=-1.,last_observed_sim_time=t) for k,t in [([-32768,32767,-1],None),([0,0,0],4.),([3,-2,10],5.)]]))
    def delta(self,cache,groups,changes=None):
        return dict(cache.identity,base_revision=cache.revision,revision=cache.revision+1,refresh_encoding='i16le_base64',changed_cells=changes or [],refreshes=groups)
    def test_extreme_negative_keys_repeats_mixed_groups_and_timestamps(self):
        cache=self.cache();baseline=self.cache()
        groups=[packed([[-32768,32767,-1],[0,0,0],[0,0,0]],7.),dict(capture_sim_time=9.,keys=[[0,0,0],[3,-2,10]]),packed([[0,0,0],[3,-2,10]],3.),packed([],10.)]
        d=self.delta(cache,groups);old=copy.deepcopy(d);cache.apply(d)
        full=self.delta(baseline,unpack(groups));full.pop('refresh_encoding');baseline.apply(full)
        self.assertEqual(cache.snapshot(),baseline.snapshot());self.assertEqual(d,old)
    def test_changed_cell_override_and_deleted_key_rejection(self):
        cache=self.cache();changes=[dict(key=[0,0,0],state='occupied',log_odds=2.,last_observed_sim_time=None)]
        cache.apply(self.delta(cache,[packed([[0,0,0]],8.)],changes));self.assertEqual(cache.cells[(0,0,0)]['state'],'occupied');self.assertEqual(cache.cells[(0,0,0)]['last_observed_sim_time'],8.)
        before=cache.snapshot();changes=[dict(key=[0,0,0],state='unknown',log_odds=None,last_observed_sim_time=None)]
        with self.assertRaisesRegex(ValueError,'snapshot_required'):cache.apply(self.delta(cache,[packed([[0,0,0]],9.)],changes))
        self.assertEqual(cache.snapshot(),before)
    def test_malformed_count_base64_shape_and_missing_key_are_atomic(self):
        bads=[dict(key_count=-1,keys_i16le_b64=''),dict(key_count=True,keys_i16le_b64=''),dict(key_count=2_000_001,keys_i16le_b64=''),dict(key_count=1,keys_i16le_b64='********'),dict(key_count=1,keys_i16le_b64='AAAA'),dict(key_count=1,keys_i16le_b64='AAAAAA=='),dict(key_count=1,keys_i16le_b64='AAAAAAAA',keys=[]),dict(key_count=1,keys_i16le_b64='AAAAAAAA',key_runs=[]),packed([[1,1,1]],10.)]
        for bad in bads:
            with self.subTest(bad=bad):
                cache=self.cache();before=cache.snapshot();bad=dict(bad,capture_sim_time=10.)
                d=self.delta(cache,[packed([[0,0,0]],9.),bad])
                with self.assertRaises(ValueError):cache.apply(d)
                self.assertEqual(cache.snapshot(),before)
    def test_nonfinite_timestamp_and_unknown_encoding_reject(self):
        for change in ('stamp','encoding'):
            cache=self.cache();before=cache.snapshot();d=self.delta(cache,[packed([[0,0,0]],8.)])
            if change=='stamp':d['refreshes'][0]['capture_sim_time']=float('nan')
            else:d['refresh_encoding']='unknown'
            with self.assertRaises(ValueError):cache.apply(d)
            self.assertEqual(cache.snapshot(),before)
    def client(self):
        c=OctomapClient(BINARY);c.request('init',**IDENTITY,prob_hit=.9,prob_miss=.1,clamp_min=.12,clamp_max=.88);return c
    def scan(self,c,seq,encoding='keys',**extra):
        return c.request('integrate',scene_id=IDENTITY['scene_id'],scene_epoch='e',robot_id='r',sensor_id='lidar',calibration_id='v',frame_seq=seq,capture_sim_time=float(seq),receive_wall_time=0.,origin=[-.125,.125,.375],points=[[2.125,.125,.375]],max_range_m=10.,compact=True,refresh_encoding=encoding,**extra)
    def test_real_service_default_binary_and_duplicate_are_exact(self):
        with self.client() as full,self.client() as binary:
            a=MapCache(full.request('snapshot'));b=MapCache(binary.request('snapshot'))
            for i in range(1,7):
                d1=self.scan(full,i);d2=self.scan(binary,i,'i16le_base64');a.apply(d1);b.apply(d2)
                self.assertNotIn('refresh_encoding',d1);self.assertEqual(d2['refresh_encoding'],'i16le_base64')
                self.assertEqual(unpack(d2['refreshes']),d1['refreshes']);self.assertEqual(a.snapshot(),b.snapshot())
            duplicate=self.scan(binary,6,'i16le_base64');self.assertTrue(duplicate['duplicate']);self.assertEqual(duplicate['refreshes'],[])
            d1=full.request('changes',since_revision=1,compact=True);d2=binary.request('changes',since_revision=1,compact=True,refresh_encoding='i16le_base64')
            self.assertEqual(unpack(d2['refreshes']),d1['refreshes']);self.assertEqual(d2['changed_cells'],d1['changed_cells'])
    def test_invalid_encoding_rejects_before_integrate_mutation_or_dedup(self):
        with self.client() as c:
            before=c.request('snapshot')
            with self.assertRaises(MapError) as error:self.scan(c,1,'invalid')
            self.assertEqual(error.exception.code,'invalid_refresh_encoding')
            after=c.request('snapshot');after['request_id']=before['request_id'];self.assertEqual(after,before)
            good=self.scan(c,1,'i16le_base64');self.assertFalse(good['duplicate'])
            with self.assertRaises(MapError) as error:
                c.request('integrate',scene_id=IDENTITY['scene_id'],scene_epoch='e',refresh_encoding='i16le_base64',compact=False)
            self.assertEqual(error.exception.code,'invalid_refresh_encoding')
