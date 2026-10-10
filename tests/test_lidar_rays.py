import numpy as np
import pytest

from sim_io.lidar_rays import observed_xy_azimuth_span_deg, prepare_lidar_rays


B = [[-50, 50], [-50, 50], [-5, 10]]
O = [.125, .125, 1.125]


def prepare(points, **kwargs):
    return prepare_lidar_rays(points, O, .25, bounds=B, **kwargs)


def test_all_distinct_near_hit_rays_survive_even_with_shared_endpoint_voxels():
    points = np.array([[x+.2, .125, 1.125] for x in np.linspace(1., 8., 400)])
    points = np.concatenate((points, [[-1.20, .125, 1.125], [-1.24, .125, 1.125]]))
    hits, free, stats = prepare(points)
    assert {tuple(k) for k in np.floor(hits/.25)} == {tuple(k) for k in np.floor(points/.25)}
    assert all(any(np.array_equal(h, p) for p in points) for h in hits)
    assert any(np.array_equal(h, [-1.20, .125, 1.125]) for h in hits)
    assert not len(free)
    assert stats['input_points'] == len(points)
    assert stats['ray_geometry_policy'] == 'all_distinct_measured_rays'
    np.testing.assert_array_equal(hits, points)
    assert stats['hit_ray_count'] == stats['total_ray_count'] == len(points)
    assert stats['unique_hit_voxels'] < stats['hit_ray_count']


def test_more_than_1200_distinct_hit_voxels_are_not_uniformly_dropped():
    xy = np.mgrid[-6:6:.25, -6:6:.25].reshape(2, -1).T
    pts = np.column_stack((xy, np.full(len(xy), 1.125)))
    hits, _, _ = prepare(pts)
    assert len(hits) == len(pts) > 1200


def test_mark_and_clear_ranges_are_independent_and_measured_endpoint_is_not_free():
    hits, free, stats = prepare([[9.125, .125, 1.125], [10.125, .125, 1.125], [45.125, .125, 1.125]])
    np.testing.assert_allclose(hits, [[9.125, .125, 1.125]])
    np.testing.assert_allclose(free, [[10.125, .125, 1.125], [40.125, .125, 1.125]])
    assert stats['free_include_endpoint'] is False
    assert stats['range_clipped_rays'] == 1


def test_missing_and_zero_returns_generate_no_free_space():
    for points in ([], [[.125, .125, 1.125]]):
        hits, free, _ = prepare(points)
        assert hits.shape == free.shape == (0, 3)


def test_clip_actual_outside_map_returns_without_leaving_the_ray():
    bounds = [[-2, 2], [-3, 3], [0, 2]]
    origin = np.array([0., 0., 1.])
    points = np.array([[10., 0, 1], [-10., 0, 1], [0., 10., 1], [0, -10, 1], [0, 0, 10], [0, 0, -10], [10, 10, 1]])
    hits, free, stats = prepare_lidar_rays(points, origin, .25, bounds=bounds)
    assert not len(hits)
    assert len(free) == len(points)
    limits = np.asarray(bounds)
    assert ((free > limits[:, 0]) & (free < limits[:, 1])).all()
    for p in free:
        assert any(np.linalg.norm(np.cross(p-origin, q-origin)) < 1e-8 for q in points)
    assert stats['boundary_clipped_rays'] == len(points)


def test_free_rays_share_endpoint_voxel_but_only_exact_duplicates_are_removed():
    points = [[12.22, .125, 1.125], [12.01, .125, 1.125], [12.20, .125, 1.125]]
    _, free, stats = prepare(points + points[:1])
    np.testing.assert_allclose(free, points)
    assert stats['measured_free_rays'] == 4
    assert stats['unique_free_endpoint_voxels'] == 1
    assert stats['free_ray_count'] == stats['total_ray_count'] == 3


def test_near_hit_exact_duplicates_are_removed_without_merging_distinct_rays():
    points = [[6.22, .125, 1.125], [6.01, .125, 1.125], [6.20, .125, 1.125]]
    hits, _, stats = prepare(points + points[:1])
    np.testing.assert_array_equal(hits, points)
    assert stats['in_range_hit_points'] == 4
    assert stats['unique_hit_voxels'] == 1
    assert stats['hit_ray_count'] == stats['total_ray_count'] == 3


def test_upper_bound_is_not_marked_and_negative_voxel_floor_is_correct():
    hits, free, _ = prepare_lidar_rays([[-.01, .1, .1], [-.24, .1, .1], [2, .1, .1]], [0, .1, .1], .25,
                                     bounds=[[-2, 2], [-2, 2], [-2, 2]])
    np.testing.assert_allclose(hits, [[-.01, .1, .1], [-.24, .1, .1]])
    assert len(free) == 1 and free[0, 0] < 2


def test_boundary_origin_and_outward_return_do_not_make_spurious_ray():
    hits, free, _ = prepare_lidar_rays([[-10, 0, 0], [1, 0, 0]], [-2, 0, 0], .25,
                                     bounds=[[-2, 2], [-2, 2], [-2, 2]])
    np.testing.assert_allclose(hits, [[1, 0, 0]])
    assert len(free) == 0


@pytest.mark.parametrize('points', [[[np.nan, 0, 0]], [[np.inf, 0, 0]], [[1, 2]], [1, 2, 3], [[1e308, 1e308, 1e308]]])
def test_bad_points_reject_complete_input(points):
    with pytest.raises(ValueError):
        prepare(points)


@pytest.mark.parametrize('kwargs', [dict(mark_range_m=-1), dict(clear_range_m=-1), dict(mark_range_m=41), dict(mark_range_m=0), dict(clear_range_m=np.nan)])
def test_bad_ranges_rejected(kwargs):
    with pytest.raises(ValueError):
        prepare([[1, 0, 0]], **kwargs)


@pytest.mark.parametrize('origin,resolution,bounds', [([50,0,0],.25,B), ([0,np.nan,0],.25,B), (O,0,B), (O,np.inf,B), (O,.25,[[0,1],[0,1]]), (O,.25,[[0,0],[0,1],[0,1]])])
def test_bad_geometry_rejected(origin,resolution,bounds):
    with pytest.raises(ValueError):
        prepare_lidar_rays([[1,0,0]],origin,resolution,bounds=bounds)


def test_structured_frame_protects_only_original_in_bounds_real_far_hits():
    from sim_io.lidar_rays import prepare_lidar_frame
    frame = prepare_lidar_frame(
        [[9.125,.125,1.125], [12.01,.125,1.125], [12.20,.125,1.125],
         [40.125,.125,1.125], [45.125,.125,1.125]], O, .25, bounds=B)
    np.testing.assert_allclose(frame.hit_points, [[9.125,.125,1.125]])
    np.testing.assert_allclose(frame.protected_hit_points,
                               [[12.01,.125,1.125], [40.125,.125,1.125]])
    assert frame.stats['real_far_hit_points'] == 3
    assert frame.stats['unique_protected_hit_voxels'] == 2
    old = prepare_lidar_rays(
        [[12.01,.125,1.125], [12.20,.125,1.125]], O, .25, bounds=B)
    assert len(old) == 3  # Historical tuple contract remains available.


def test_clipped_endpoints_never_become_protected_hit_points():
    from sim_io.lidar_rays import prepare_lidar_frame
    # Both clipped free endpoints exist, but neither is an actual surface.
    frame = prepare_lidar_frame([[45.125,.125,1.125]], O, .25, bounds=B)
    assert len(frame.free_endpoints) == 1
    assert frame.protected_hit_points.shape == (0, 3)
    frame = prepare_lidar_frame([[15.125,.125,1.125]], O, .25,
                               bounds=[[-10,10],[-10,10],[-5,10]])
    assert len(frame.free_endpoints) == 1
    assert frame.protected_hit_points.shape == (0, 3)


@pytest.mark.parametrize('points', [[], [[1., 0., 2.]], [[1., 0., 0.], [2., 0., 4.]],
                                  [[0., 0., 0.], [0., 0., 3.]]])
def test_observed_azimuth_empty_single_direction_and_zero_xy(points):
    assert observed_xy_azimuth_span_deg(points) == 0.


@pytest.mark.parametrize('degrees', [list(range(-60, 61, 10)), [300, 330, 0, 30, 60]])
def test_observed_azimuth_120_degree_arc_including_wraparound(degrees):
    angles = np.radians(degrees)
    points = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    before = points.copy()
    assert observed_xy_azimuth_span_deg(points) == pytest.approx(120.)
    np.testing.assert_array_equal(points, before)


def test_observed_azimuth_near_full_circle_is_not_claimed_as_emitted_360_degrees():
    angles = np.radians(np.arange(360))
    points = np.column_stack((np.cos(angles), np.sin(angles)))
    assert observed_xy_azimuth_span_deg(points) == pytest.approx(359.)


def test_observed_azimuth_ignores_nonfinite_or_zero_xy_but_not_irrelevant_z():
    points = [[1., 0., np.nan], [0., 1., np.inf], [0., 0., 3.],
              [np.nan, 1., 0.], [1., np.inf, 0.]]
    assert observed_xy_azimuth_span_deg(points) == pytest.approx(90.)
