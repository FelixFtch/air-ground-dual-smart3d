"""Opt-in transport timings must not change default map response contracts."""
import os
from pathlib import Path
import threading
import unittest
from mapping.client import OctomapClient,MapError

ROOT=Path(__file__).resolve().parents[1]
BINARY=Path(os.environ.get('OCTOMAP_SERVICE_BIN',ROOT/'mapping/build/octomap_service'))
IDENTITY=dict(scene_id='timing',scene_epoch='test',content_version='v1',frame_id='map',resolution=.25)

class TransportTimingTests(unittest.TestCase):
    def test_default_response_unchanged(self):
        with OctomapClient(BINARY) as client:
            response=client.request('init',**IDENTITY)
            self.assertNotIn('_transport_timing',response)
            self.assertTrue(response['ok'])

    def test_opt_in_records_bytes_and_stages_on_success_and_error(self):
        with OctomapClient(BINARY,profile_transport=True) as client:
            response=client.request('init',**IDENTITY)
            timings=response['_transport_timing']
            self.assertEqual(set(timings),{'wait_lock_s','encode_s','write_s','await_s','decode_s','response_bytes','request_bytes'})
            for key,value in timings.items():self.assertGreaterEqual(value,0,key)
            self.assertGreater(timings['request_bytes'],0);self.assertGreater(timings['response_bytes'],0)
            with self.assertRaises(MapError) as error:client.request('missing_operation')
            self.assertIn('_transport_timing',error.exception.response)

    def test_wait_lock_is_not_charged_to_server(self):
        with OctomapClient(BINARY,profile_transport=True) as client:
            client.request('init',**IDENTITY)
            client._lock.acquire()
            release=threading.Timer(.04,client._lock.release);release.start()
            try:response=client.request('status')
            finally:release.join()
            self.assertGreaterEqual(response['_transport_timing']['wait_lock_s'],.02)
