import math
import pytest
from execution.path_follow import lookahead,bounded_step


def test_lookahead_tracks_segment_without_returning_to_old_start():
    assert list(lookahead([5,0,0],[[0,0,0],[10,0,0]],.7))==pytest.approx([5.7,0.,0.])


def test_turn_first_and_keep_altitude_for_ground_target():
    p,yaw,s=bounded_step([0,0,.349],math.pi,[10,0,.349],0,2,2,1,.05)
    assert p==[0.,0.,.349] and s==0 and yaw<math.pi


def test_xyz_command_has_measurable_vertical_step():
    p,yaw,s=bounded_step([0,0,2],0,[3,0,4],0,1.5,1.2,.8,.05)
    assert p[2]>2 and math.dist(p,[0,0,2])<=1.5*.05
