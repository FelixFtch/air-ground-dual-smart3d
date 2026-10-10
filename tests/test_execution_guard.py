import math
import unittest
from execution.guard import check_path_segment, bounded_path_target


class GuardTests(unittest.TestCase):
    def free(self, stamp=9.):
        return {(x, y, z): {'state': 'free', 'last_observed_sim_time': stamp}
                for x in range(-8, 9) for y in range(-8, 9) for z in range(-8, 17)}

    def check(self, cells, start=(0.1,0.1,1.1), end=(0.6,0.1,1.1), half=(.1,.1,.1), **kw):
        return check_path_segment(cells,.25,start,end,half,kw.pop('offset',(0,0,0)),10.,**kw)

    def test_fresh_free_and_age_boundary(self):
        self.assertTrue(self.check(self.free())['ok'])
        self.assertTrue(self.check(self.free(7.))['ok'])
        self.assertEqual(self.check(self.free(6.99))['reason'],'stale_free')
        self.assertEqual(self.check(self.free(11.))['reason'],'free_time_in_future')

    def test_retained_free_policy_accepts_old_evidence(self):
        self.assertTrue(self.check(self.free(0.), max_free_age=None)['ok'])
        # The default remains the original explicit 3-second policy.
        self.assertEqual(self.check(self.free(0.))['reason'], 'stale_free')

    def test_retained_free_policy_still_requires_observed_free(self):
        self.assertEqual(self.check({}, max_free_age=None)['reason'], 'unknown')
        for stamp in (None, float('nan'), float('inf')):
            cells = self.free()
            for cell in cells.values(): cell['last_observed_sim_time'] = stamp
            self.assertEqual(self.check(cells, max_free_age=None)['reason'], 'free_time_missing')
        self.assertEqual(self.check(self.free(11.), max_free_age=None)['reason'], 'free_time_in_future')
        cells = self.free(0.); cells[(1,0,4)]['state'] = 'occupied'
        self.assertEqual(self.check(cells, max_free_age=None)['reason'], 'occupied')

    def test_occupied_always_checked_even_in_current_body(self):
        cells = self.free(); cells[(0,0,4)]['state']='occupied'
        self.assertEqual(self.check(cells)['reason'],'occupied')
        self.assertEqual(self.check(cells,end=(.1,.1,1.1))['reason'],'occupied')

    def test_unknown_new_volume_blocked_but_stationary_exempt(self):
        self.assertEqual(self.check({})['reason'],'unknown')
        self.assertTrue(self.check({},end=(.1,.1,1.1))['ok'])
        # Both poses touch the same voxel: its newly entered portion is unknown.
        self.assertEqual(self.check({},start=(.10,.10,1.10),end=(.11,.10,1.10),half=(.01,.01,.01))['reason'],'unknown')

    def test_stale_initial_volume_exempt_only_where_already_occupied(self):
        self.assertTrue(self.check(self.free(0),end=(.1,.1,1.1))['ok'])
        self.assertEqual(self.check(self.free(0))['reason'],'stale_free')

    def test_offset_and_overhead(self):
        cells = self.free(); cells[(1,0,8)]={'state':'occupied','last_observed_sim_time':9.}
        self.assertTrue(self.check(cells)['ok'])
        self.assertEqual(self.check(cells,offset=(0,0,1))['reason'],'occupied')
        cells = self.free(); cells[(1,0,6)]['state']='occupied'
        self.assertTrue(self.check(cells,half=(.1,.1,.1))['ok'])
        self.assertEqual(self.check(cells,half=(.1,.1,.5))['reason'],'occupied')

    def test_diagonal_segment_interior_collision(self):
        cells = self.free(); cells[(1,1,4)]['state']='occupied'
        self.assertEqual(self.check(cells,start=(.1,.1,1.1),end=(1.,1.,1.1),half=(.01,.01,.01))['reason'],'occupied')
        # AABB broadphase alone would wrongly reject this off-diagonal voxel.
        cells = self.free(); cells[(0,3,4)]['state']='occupied'
        self.assertTrue(self.check(cells,start=(.1,.1,1.1),end=(1.,1.,1.1),half=(.01,.01,.01))['ok'])

    def test_go2_floor_and_positive_low_obstacle(self):
        cells = self.free(); cells[(0,0,-1)]['state']='occupied'
        self.assertTrue(self.check(cells,start=(.1,.1,.349),end=(.2,.1,.349),half=(.43,.43,.28))['ok'])
        cells[(0,0,0)]['state']='occupied'
        self.assertEqual(self.check(cells,start=(.1,.1,.349),end=(.2,.1,.349),half=(.43,.43,.28))['reason'],'occupied')

    def test_full_body_bounds(self):
        bounds={'min':[-1,-1,0],'max':[.65,1,2]}
        self.assertEqual(self.check(self.free(),bounds=bounds)['reason'],'body_out_of_bounds')
        self.assertTrue(self.check(self.free(),end=(.5,.1,1.1),bounds=bounds)['ok'])

    def test_near_target_distance_bounded_in_3d(self):
        path=[[0,0,1],[1,0,1],[1,0,3]]
        for current in ([0,0,1],[.8,0,1],[.9,.4,1],[1,0,3]):
            p=bounded_path_target(path,current,.25)
            self.assertLessEqual(math.dist(current,p),.25+1e-9)
        self.assertEqual(bounded_path_target(path,[1,0,1],.25),[1.,0.,1.25])

    def test_invalid_nonfinite_fails_closed(self):
        self.assertEqual(self.check(self.free(),end=(float('nan'),0,0))['reason'],'invalid_input')
        self.assertEqual(self.check(self.free(),half=(0,.1,.1))['reason'],'invalid_input')


if __name__=='__main__':unittest.main()
