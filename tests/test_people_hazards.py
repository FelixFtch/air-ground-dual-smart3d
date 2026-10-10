import copy
import math
import unittest

from perception.hazards import build_hazards, check_hazard_segment, hazard_segment_intersects


def profile(n=3):
    return dict(profile_id="test", dimension=n, body_half_extents_m=[.12,.12,.12],
                body_offset_m=[0.,0.,0.], planar_z=.349)


def document(velocity=(1.,0.,0.)):
    return dict(schema="air_ground.people/v1", scene_epoch="epoch1", scene_id="test",
                reference_sim_time=1., generated_wall_time=100., people=[dict(
                    id="person-1", global_id="person-1", center_xyz=[0.,0.,1.],
                    half_extents_xyz=[.2,.15,.4], velocity_xyz=list(velocity),
                    velocity_xy_valid=True, velocity_z_valid=False, capture_sim_time=1.,
                    position_reference_sim_time=1., valid_until_sim_time=2.5,
                    source_keys=["epoch1:go2:front:session:1"],
                    quality=dict(extent_kind="visible_surface_quantiles", calibration_status="uncalibrated"))])


class PeopleHazards(unittest.TestCase):
    def test_identity_time_expiry_and_velocity_gates(self):
        doc=document()
        self.assertEqual(build_hazards(doc,profile(),2.5,"epoch1","test"),[])
        for edit in [lambda d:d.update(scene_epoch="old"),
                     lambda d:d.update(reference_sim_time=2.),
                     lambda d:d["people"][0].update(velocity_z_valid=True),
                     lambda d:d["people"][0].update(velocity_xyz=[1,0,.1]),
                     lambda d:d["people"][0].update(capture_sim_time=1.1),
                     lambda d:d["people"][0].update(source_keys=[])]:
            bad=copy.deepcopy(doc); edit(bad)
            with self.assertRaises(ValueError): build_hazards(bad,profile(),1.,"epoch1","test")

    def test_speed_orientation_strict_ellipse_and_no_vertical_prediction(self):
        for n in (2,3):
            h=build_hazards(document((.6,.8,0.)),profile(n),1.,"epoch1","test")[0]
            self.assertEqual(h["axes_xyz"][0],[.6,.8,0.])
            self.assertGreater(h["semi_axes_m"][0],h["semi_axes_m"][1])
            self.assertFalse(h["velocity_z_valid"])
            self.assertEqual(h["center_xyz"][2],.349 if n==2 else 1.)
            self.assertTrue(h["prior"]["human_height_prior_applied"])
            self.assertEqual(h["measurement"]["half_extents_xyz"],[.2,.15,.4])
        d=document();d["people"][0]["velocity_xy_valid"]=False
        h=build_hazards(d,profile(),1.,"epoch1","test")[0]
        self.assertEqual(h["center_xyz"],[0.,0.,1.])
        self.assertEqual(h["semi_axes_m"][0],h["semi_axes_m"][1])
        self.assertEqual(h["velocity_xyz"],[0.,0.,0.])

    def test_swept_measured_body_and_robot_envelope_are_enclosed_once(self):
        # Probe Minkowski box corners and horizontal angular support over sweep.
        for n in (2,3):
            p=profile(n);p["body_offset_m"]=[.3,-.2,.1]
            h=build_hazards(document((.6,.8,0)),p,1.,"epoch1","test")[0]
            rho=math.hypot(.2,.15)+math.hypot(.12,.12)+.15
            hz=.4+.85+.12
            for step in range(11):
                for deg in range(0,360,9):
                    angle=math.radians(deg)
                    for z in (-hz,hz):
                        point=[.6*step/10+rho*math.cos(angle)-.3,
                               .8*step/10+rho*math.sin(angle)+.2,
                               1+z-.1]
                        self.assertTrue(hazard_segment_intersects(point,point,h))
            self.assertEqual(h["robot_envelope"]["half_extents_xyz"],[.12,.12,.12])
            self.assertTrue(h["footprint_included"])

    def test_exact_tangent_thin_crossing_and_finite_vertical_extent(self):
        h=build_hazards(document(),profile(),1.,"epoch1","test")[0]
        x,y,z=h["center_xyz"];a,b,c=h["semi_axes_m"]
        self.assertIsNotNone(check_hazard_segment([x-10,y,z],[x+10,y,z],[h],1.))
        self.assertTrue(hazard_segment_intersects([x-10,y+b,z],[x+10,y+b,z],h))
        self.assertFalse(hazard_segment_intersects([x-10,y+b+.001,z],[x+10,y+b+.001,z],h))
        self.assertFalse(hazard_segment_intersects([x-10,y,z+c+.001],[x+10,y,z+c+.001],h))
        self.assertIsNone(check_hazard_segment([x-10,y,z],[x+10,y,z],[h],2.5))

    def test_planar_vertical_gate_and_explicit_disabled_prior(self):
        d=document();d["people"][0]["center_xyz"][2]=4.
        self.assertEqual(build_hazards(d,profile(2),1.,"epoch1","test"),[])
        self.assertEqual(len(build_hazards(d,profile(3),1.,"epoch1","test")),1)
        p=profile();p["people_hazard"]={"vertical_padding_m":0}
        h=build_hazards(document(),p,1.,"epoch1","test")[0]
        self.assertFalse(h["prior"]["human_height_prior_applied"])
        self.assertLess(h["semi_axes_m"][2],build_hazards(document(),profile(),1.,"epoch1","test")[0]["semi_axes_m"][2])

    def test_registry_projection_is_not_applied_twice(self):
        d=document();d["reference_sim_time"]=1.5
        d["people"][0].update(position_reference_sim_time=1.5,center_xyz=[.5,0.,1.])
        h=build_hazards(d,profile(),1.5,"epoch1","test")[0]
        self.assertAlmostEqual(h["center_xyz"][0],1.) # .5 source + .5 horizon midpoint
        h=build_hazards(d,profile(),2.4,"epoch1","test")[0]
        self.assertAlmostEqual(h["center_xyz"][0],1.5) # cap reference propagation at capture+1


if __name__ == '__main__': unittest.main()
