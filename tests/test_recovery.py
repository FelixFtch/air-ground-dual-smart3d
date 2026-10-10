import copy
import math
import unittest

from execution.guard import check_path_segment
from execution.recovery import check_recovery_segment, find_recovery_target


class RecoveryTests(unittest.TestCase):
    # Body touches x=[0,.25] through its rightmost 2.5 cm only.
    START = (-.075, .125, 1.125)
    HALF = (.1, .06, .06)

    def free(self):
        return {(x,y,z): {"state":"free", "last_observed_sim_time":10.}
                for x in range(-8,9) for y in range(-8,9) for z in range(-2,11)}

    def map(self):
        cells=self.free();cells[(0,0,4)]={"state":"occupied", "last_observed_sim_time":10.}
        return cells

    def check(self,cells,end,**kw):
        return check_recovery_segment(cells,.25,self.START,end,self.HALF,(0,0,0),10.,**kw)

    def find(self,cells,**kw):
        return find_recovery_target(cells,.25,self.START,self.HALF,(0,0,0),10.,**kw)

    def test_valid_shallow_escape_map_unchanged_and_end_fully_clear(self):
        cells=self.map();before=copy.deepcopy(cells)
        end=self.find(cells)
        self.assertIsNotNone(end)
        self.assertLess(end[0],self.START[0]);self.assertEqual(end[1:],list(self.START[1:]))
        self.assertLessEqual(math.dist(self.START,end),.6)
        self.assertTrue(self.check(cells,end)["ok"])
        self.assertTrue(check_path_segment(cells,.25,end,end,self.HALF,(0,0,0),10.)["ok"])
        self.assertEqual(cells,before)

    def test_cannot_cross_thin_wall_despite_clear_endpoint(self):
        result=self.check(self.map(),(.4,.125,1.125))
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"],"occupied_overlap_expands")

    def test_new_obstacle_in_escape_sweep_is_not_exempt(self):
        cells=self.map();cells[(-2,0,4)]={"state":"occupied"}
        result=self.check(cells,(-.45,.125,1.125))
        self.assertFalse(result["ok"]);self.assertEqual(result["reason"],"occupied")
        self.assertEqual(result["blocked_key"],[-2,0,4])

    def test_unknown_new_volume_rejected_even_in_same_initial_voxel(self):
        cells=self.map();del cells[(-1,0,4)]
        self.assertEqual(self.check(cells,(-.11,.125,1.125))["reason"],"unknown")
        self.assertIsNone(self.find(cells))

    def test_diagonal_may_not_expand_old_overlap_along_another_axis(self):
        # X retreats, but Y expands the occupied intersection before X is clear.
        result=self.check(self.map(),(-.15,.20,1.125))
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"],"occupied_overlap_expands")

    def test_partial_slow_prefix_is_monotone_but_not_complete_target(self):
        cells=self.map();prefix=(-.085,.125,1.125)
        self.assertEqual(self.check(cells,prefix)["reason"],"end_still_occupied")
        self.assertTrue(self.check(cells,prefix,require_clear_end=False)["ok"])
        self.assertFalse(self.check(cells,(-.065,.125,1.125),require_clear_end=False)["ok"])

    def test_freshness_policy_bounds_and_six_tenths_limit_preserved(self):
        cells=self.map()
        for cell in cells.values():cell["last_observed_sim_time"]=0.
        self.assertIsNone(self.find(cells,max_free_age=3.))
        self.assertIsNotNone(self.find(cells,max_free_age=None))
        self.assertIsNone(self.find(cells,max_distance_m=.61))
        self.assertEqual(self.check(cells,(-.7,.125,1.125))["reason"],"recovery_distance_out_of_bounds")
        self.assertIsNone(self.find(cells,bounds={"min":[-.18,-1,0],"max":[1,1,2]}))

    def test_vertical_only_egress_available_to_3d_not_2d(self):
        cells=self.free()
        # Broad occupied ceiling; XY moves would deepen contact in adjacent cells.
        for x in range(-5,6):
            for y in range(-5,6):cells[(x,y,5)]={"state":"occupied"}
        start=(.125,.125,1.175);half=(.10,.10,.10)
        self.assertIsNone(find_recovery_target(cells,.25,start,half,(0,0,0),10.,dimension=2))
        end=find_recovery_target(cells,.25,start,half,(0,0,0),10.,dimension=3)
        self.assertIsNotNone(end);self.assertLess(end[2],start[2]);self.assertEqual(end[:2],list(start[:2]))
        self.assertTrue(check_recovery_segment(cells,.25,start,end,half,(0,0,0),10.,dimension=3)["ok"])

    def test_offset_and_no_initial_overlap(self):
        cells=self.map();start=tuple(self.START[i]-(.2 if i==0 else 0) for i in range(3))
        end=find_recovery_target(cells,.25,start,self.HALF,(.2,0,0),10.)
        self.assertIsNotNone(end)
        self.assertTrue(check_recovery_segment(cells,.25,start,end,self.HALF,(.2,0,0),10.)["ok"])
        self.assertIsNone(self.find(self.free()))


if __name__=="__main__":unittest.main()
