import numpy as np
from sim_io.geometry import depth_points


def test_pixel_center_projects_encoded_axial_depth():
    depth=2.;value=round(depth/1000*16777215)
    class Image:
        width=2; height=2
        raw_data=bytes([(value>>16)&255,(value>>8)&255,value&255,255])*4
    points,free,_=depth_points(Image(),90,stride=1)
    assert len(points)==4 and len(free)==0
    actual_depth=value/16777215*1000
    np.testing.assert_allclose(points[0],[actual_depth,-.5*actual_depth,.5*actual_depth])
    np.testing.assert_allclose(points[-1],[actual_depth,.5*actual_depth,-.5*actual_depth])
