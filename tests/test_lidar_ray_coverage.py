"""Real OctoMap checks that endpoint sharing cannot discard ray coverage."""
import os
from pathlib import Path

import pytest

from mapping.client import OctomapClient
from sim_io.lidar_rays import prepare_lidar_frame

ROOT = Path(__file__).resolve().parents[1]
BINARY = os.environ.get('OCTOMAP_SERVICE_BIN', ROOT / 'mapping/build/octomap_service')
BOUNDS = [[-50,50],[-50,50],[-1,8]]
GO2 = [.125,.125,1.125]


@pytest.fixture
def client():
    with OctomapClient(BINARY) as c:
        c.request('init', scene_id='ray-coverage', scene_epoch='e', content_version='v',
                  frame_id='map', resolution=.25, prob_hit=.9, prob_miss=.1,
                  clamp_min=.12, clamp_max=.88,
                  bounds={'min': [-50,-50,-1], 'max': [50,50,8]})
        yield c


def scan(client, seq, origin, returns, robot='go2'):
    frame = prepare_lidar_frame(returns, origin, .25, 9, 40, BOUNDS)
    return client.request('integrate', scene_id='ray-coverage', scene_epoch='e',
        robot_id=robot, sensor_id='lidar', frame_seq=seq, capture_sim_time=float(seq),
        receive_wall_time=0., calibration_id='c', origin=origin,
        points=frame.hit_points.tolist(), protected_hit_points=frame.protected_hit_points.tolist(),
        free_rays=[{'endpoint': p.tolist(), 'source': 'lidar_measured_beyond_range',
                    'include_endpoint': False} for p in frame.free_endpoints],
        mark_range_m=9., max_range_m=40., compact=True)


def query(client, *points):
    return client.request('query', points=points)['cells']


@pytest.mark.parametrize('old_obstacle,returns,expected_hit_cells', [
    ([6.125,.30,.625], [[12.05,.26,.01], [12.24,.49,.02]], 0),
    ([4.125,.30,.625], [[8.05,.26,.01], [8.24,.49,.02]], 1),
])
def test_same_endpoint_voxel_distinct_rays_clear_intermediate_occupied_cell(
        client, old_obstacle, returns, expected_hit_cells):
    # Both returns land in one voxel, but only the second ray crosses A's
    # intermediate voxel. Endpoint-voxel dedup discarded that real free evidence.
    seed_origin = [old_obstacle[0], old_obstacle[1], 2.125]
    scan(client, 1, seed_origin, [old_obstacle], robot='uav')
    initial = query(client, old_obstacle)[0]
    assert initial['state'] == 'occupied'
    scan(client, 2, GO2, returns[:1])
    assert query(client, old_obstacle)[0] == initial

    frame = prepare_lidar_frame(returns + returns[:1], GO2, .25, 9, 40, BOUNDS)
    assert frame.stats['total_ray_count'] == 2  # Exact duplicate still removed.
    assert (frame.stats['unique_hit_voxels'] +
            frame.stats['unique_free_endpoint_voxels']) == 1
    update = scan(client, 3, GO2, returns + returns[:1])
    actual = query(client, old_obstacle)[0]
    assert actual['state'] == 'free'
    assert actual['last_observed_sim_time'] == 3.
    assert update['accepted_hits'] == expected_hit_cells
    assert any(c['key'] == actual['key'] and c['state'] == 'free'
               for c in update['changed_cells'])


@pytest.mark.parametrize('returns', [
    [[12.05,.26,.01], [12.24,.49,.02]],
    [[8.05,.26,.01], [8.24,.49,.02]],
])
def test_retained_rays_do_not_override_same_frame_real_hit(client, returns):
    obstacle = [4.125,.30,.625] if returns[0][0] < 9 else [6.125,.30,.625]
    update = scan(client, 1, GO2, returns + [obstacle])
    assert query(client, obstacle)[0]['state'] == 'occupied'
    assert update['accepted_hits'] >= 1
