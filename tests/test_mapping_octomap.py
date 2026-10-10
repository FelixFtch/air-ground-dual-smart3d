"""Behavior checks against the real linked OctoMap process (no mocked occupancy)."""
import json
import math
import os
from pathlib import Path
import tempfile
import unittest

from mapping.client import FREE_RAY_SOURCES, MapCache, MapError, OctomapClient

ROOT = Path(__file__).resolve().parents[1]
BINARY = Path(os.environ.get("OCTOMAP_SERVICE_BIN", ROOT / "mapping/build/octomap_service"))


class MappingTests(unittest.TestCase):
    def setUp(self):
        if not BINARY.is_file():
            self.fail(f"Build real OctoMap service first: {BINARY}")
        self.client = OctomapClient(BINARY)
        self.client.request("init", scene_id="lab", scene_epoch="episode-a",
                            content_version="geometry-1", frame_id="map", resolution=.25,
                            bounds={"min": [-10, -10, -3], "max": [10, 10, 8]},
                            history_limit=3)
        self.sequence = 0

    def tearDown(self):
        self.client.close()

    def scan(self, points, origin=(.125, .125, 1.125), **overrides):
        self.sequence += 1
        data = dict(scene_id="lab", scene_epoch="episode-a", robot_id="go2_01",
                    sensor_id="lidar", frame_seq=self.sequence,
                    capture_sim_time=1 + self.sequence*.05, receive_wall_time=100.,
                    calibration_id="go2-lidar-v1", origin=list(origin), points=points,
                    max_range_m=12.)
        data.update(overrides)
        return self.client.request("integrate", **data)

    def query(self, *points):
        return self.client.request("query", points=points)["cells"]

    def test_hit_free_unknown_and_occlusion(self):
        self.scan([[2.125, .125, 1.125]])
        cells = self.query([1.125, .125, 1.125], [2.125, .125, 1.125], [3.125, .125, 1.125])
        self.assertEqual([c["state"] for c in cells], ["free", "occupied", "unknown"])

    def test_duplicate_frame_does_not_add_weight(self):
        first = self.scan([[2.125, .125, 1.125]])
        before = self.query([2.125, .125, 1.125])[0]
        duplicate = self.scan([[2.125, .125, 1.125]], frame_seq=1, capture_sim_time=1.05)
        self.assertTrue(duplicate["duplicate"])
        self.assertEqual(first["revision"], duplicate["revision"])
        self.assertEqual(before, self.query([2.125, .125, 1.125])[0])

    def test_many_hits_in_one_frame_count_once_and_hit_wins(self):
        self.scan([[2.125, .125, 1.125]]*100 + [[3.125, .125, 1.125]])
        node = self.query([2.125, .125, 1.125])[0]
        self.assertAlmostEqual(node["log_odds"], math.log(.7/.3), places=6)
        self.assertEqual(node["state"], "occupied")

    def test_shared_robot_evidence_and_clear_changes(self):
        first = self.scan([[2.125, .125, 1.125]])
        shared = self.scan([[2.125, .125, 1.125]], robot_id="uav_01", frame_seq=1)
        self.assertGreater(self.query([2.125,.125,1.125])[0]["log_odds"], 1.5)
        for _ in range(5):
            self.scan([[4.125,.125,1.125]], robot_id="uav_01")
        cell = self.query([2.125,.125,1.125])[0]
        self.assertEqual(cell["state"], "free")
        self.assertGreater(shared["revision"], first["revision"])
        delta = self.client.request("changes", since_revision=shared["revision"]+2)
        self.assertIn([8,0,4], [c["key"] for c in delta["changed_cells"] if c["state"]=="free"])

    def test_capture_origin_not_latest_robot_pose(self):
        self.scan([[2.125,2.125,1.125]], origin=(2.125,-1.125,1.125))
        free, untouched = self.query([2.125,.125,1.125], [.125,.125,1.125])
        self.assertEqual(free["state"], "free")
        self.assertEqual(untouched["state"], "unknown")

    def test_old_scene_and_old_time_rejected_without_mutation(self):
        self.scan([[2.125,.125,1.125]], capture_sim_time=20)
        before = self.client.request("snapshot")
        for fields, code in [({"scene_epoch":"old"}, "scene_mismatch"),
                             ({"capture_sim_time":19}, "observation_too_old")]:
            with self.assertRaises(MapError) as caught:
                self.scan([[8.125,.125,1.125]], **fields)
            self.assertEqual(caught.exception.code, code)
        self.assertEqual(before, {**self.client.request("snapshot"), "request_id":before["request_id"]})

    def test_in_window_out_of_order_keeps_newest_freshness(self):
        self.scan([[2.125,.125,1.125]], capture_sim_time=20, frame_seq=100)
        self.scan([[2.125,.125,1.125]], capture_sim_time=19.8, frame_seq=99)
        self.assertEqual(self.query([2.125,.125,1.125])[0]["last_observed_sim_time"],20)

    def test_invalid_frame_transaction_and_valid_retry(self):
        with self.assertRaises(MapError):
            self.scan([[2.125,.125,1.125],[2,3]], frame_seq=40)
        self.assertEqual(self.client.request("status")["revision"],0)
        retry=self.scan([[2.125,.125,1.125]], frame_seq=40)
        self.assertFalse(retry["duplicate"])

    def test_no_return_and_beyond_range_do_not_clear(self):
        self.scan([[2.125,.125,1.125]])
        original=self.query([2.125,.125,1.125])[0]
        self.scan([])
        self.scan([[8.125,.125,1.125]], max_range_m=4)
        self.assertEqual(original,self.query([2.125,.125,1.125])[0])

    def test_explicit_depth_free_ray_and_hit_priority(self):
        self.scan([[2.125,.125,1.125]],free_rays=[
            {"endpoint":[4.125,.125,1.125],"source":"carla_depth_far_plane"}])
        hit,end,behind=self.query([2.125,.125,1.125],[4.125,.125,1.125],[4.375,.125,1.125])
        self.assertEqual([hit["state"],end["state"],behind["state"]],["occupied","free","unknown"])
        before=self.client.request("status")["revision"]
        with self.assertRaises(MapError) as caught:
            self.scan([],free_rays=[{"endpoint":[4.125,.125,1.125],"source":"lidar_no_return"}])
        self.assertEqual(caught.exception.code,"invalid_free_ray_source")
        self.assertEqual(self.client.request("status")["revision"],before)

    def test_explicit_beyond_range_lidar_hit_clears_only_cropped_ray(self):
        self.assertIn("lidar_measured_beyond_range",FREE_RAY_SOURCES)
        origin=(-4.125,.125,1.125)
        self.scan([[2.125,.125,1.125]],origin=origin)
        before=self.query([2.125,.125,1.125])[0]
        # A real return 13 m away is not automatically converted to free evidence.
        self.scan([[8.875,.125,1.125]],origin=origin,max_range_m=12.)
        self.assertEqual(before,self.query([2.125,.125,1.125])[0])
        # The bridge explicitly submits only its observed 12 m free segment.
        for _ in range(3):
            self.scan([],origin=origin,max_range_m=12.,compact=True,free_rays=[
                {"endpoint":[7.875,.125,1.125],"source":"lidar_measured_beyond_range"}])
        previous_obstacle,terminal,beyond=self.query([2.125,.125,1.125],[7.875,.125,1.125],[8.125,.125,1.125])
        self.assertEqual([c["state"] for c in (previous_obstacle,terminal,beyond)],["free","free","unknown"])

    def test_negative_keys_are_floor_coordinates(self):
        self.scan([[-.125,-.125,1.125]],origin=(-2.125,-.125,1.125))
        self.assertEqual(self.query([-.125,-.125,1.125])[0]["key"],[-1,-1,4])

    def test_forget_versions_unknown_and_cache_gap(self):
        initial=self.client.request("snapshot")
        cache=MapCache(initial)
        cache.apply(self.scan([[2.125,.125,1.125]]))
        delta=self.client.request("forget",scene_id="lab",scene_epoch="episode-a",keys=[[8,0,4]],reason="explicit maintenance test")
        cache.apply(delta)
        self.assertEqual(delta["changed_cells"][0]["state"],"unknown")
        self.assertNotIn((8,0,4),cache.cells)
        with self.assertRaisesRegex(ValueError,"snapshot_required"):
            cache.apply(delta)
        for _ in range(4): self.scan([[2.125,.125,1.125]])
        with self.assertRaises(MapError) as caught: self.client.request("changes",since_revision=0)
        self.assertEqual(caught.exception.code,"snapshot_required")

    def test_compact_refresh_clear_and_full_snapshot_equivalence(self):
        cache=MapCache(self.client.request("snapshot"))
        first=self.scan([[2.125,.125,1.125]],compact=True)
        cache.apply(first)
        second=self.scan([[2.125,.125,1.125]],compact=True)
        self.assertEqual(second["changed_cells"],[])
        self.assertTrue(second["refreshes"])
        previous_probability=cache.cells[(8,0,4)]["log_odds"]
        cache.apply(second)
        self.assertGreater(self.query([2.125,.125,1.125])[0]["log_odds"],previous_probability)
        self.assertEqual(cache.cells[(8,0,4)]["last_observed_sim_time"],1.1)
        for _ in range(5): cache.apply(self.scan([[4.125,.125,1.125]],compact=True))
        self.assertEqual(cache.cells[(8,0,4)]["state"],"free")
        full=self.client.request("snapshot")
        self.assertEqual({tuple(c["key"]):(c["state"],c["last_observed_sim_time"]) for c in full["cells"]},
                         {key:(c["state"],c["last_observed_sim_time"]) for key,c in cache.cells.items()})
        cache.apply(self.client.request("forget",scene_id="lab",scene_epoch="episode-a",keys=[[8,0,4]],reason="compact deletion",compact=True))
        self.assertNotIn((8,0,4),cache.cells)

    def test_compact_merged_changes_newest_per_cell_time(self):
        self.scan([[2.125,.125,1.125]],capture_sim_time=20,compact=True)
        cache=MapCache(self.client.request("snapshot"))
        self.scan([[2.125,.125,1.125]],capture_sim_time=20.2,compact=True)
        self.scan([[2.125,.125,1.125]],capture_sim_time=20.1,compact=True)
        merged=self.client.request("changes",since_revision=cache.revision,compact=True)
        self.assertEqual(merged["changed_cells"],[])
        self.assertEqual([g["capture_sim_time"] for g in merged["refreshes"]],[20.2])
        cache.apply(merged)
        self.assertEqual(cache.cells[(8,0,4)]["last_observed_sim_time"],20.2)

    def test_full_probability_ot_restore_identity_and_stale_freshness(self):
        self.scan([[2.125,.125,1.125]])
        self.scan([[2.125,.125,1.125]])
        before=self.query([2.125,.125,1.125])[0]
        with tempfile.TemporaryDirectory() as folder:
            path=str(Path(folder)/"scene.ot")
            self.client.request("save",path=path)
            self.assertTrue(Path(path).read_bytes().startswith(b"# Octomap OcTree file"))
            with OctomapClient(BINARY) as restored:
                restored.request("init",scene_id="lab",scene_epoch="episode-b",content_version="geometry-1",frame_id="map",resolution=.25)
                response=restored.request("load",path=path)
                self.assertEqual(response["revision"],1)
                cell=restored.request("query",points=[[2.125,.125,1.125]])["cells"][0]
                self.assertEqual(cell["log_odds"],before["log_odds"])
                self.assertIsNone(cell["last_observed_sim_time"])
                self.assertEqual(cell["state"],"occupied")
            with OctomapClient(BINARY) as wrong:
                wrong.request("init",scene_id="another",scene_epoch="episode-b",content_version="geometry-1",frame_id="map",resolution=.25)
                with self.assertRaises(MapError) as caught: wrong.request("load",path=path)
                self.assertEqual(caught.exception.code,"restore_mismatch")
            with open(path,"ab") as stream: stream.write(b"tampered")
            with OctomapClient(BINARY) as corrupt:
                corrupt.request("init",scene_id="lab",scene_epoch="episode-b",content_version="geometry-1",frame_id="map",resolution=.25)
                with self.assertRaises(MapError) as caught: corrupt.request("load",path=path)
                self.assertEqual(caught.exception.code,"checksum_mismatch")


if __name__=="__main__":
    unittest.main()
