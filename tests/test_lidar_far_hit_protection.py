"""Real OctoMap regressions for far surfaces crossed by other measured rays."""
import os
import json
from pathlib import Path

import pytest

from mapping.client import MapCache, MapError, OctomapClient
from sim_io.lidar_rays import prepare_lidar_frame

ROOT = Path(__file__).resolve().parents[1]
BINARY = os.environ.get('OCTOMAP_SERVICE_BIN', ROOT / 'mapping/build/octomap_service')
BOUNDS = [[-50,50],[-50,50],[-1,8]]
GO2 = [.125,.125,1.125]
UAV = [10.125,.26,3.125]
A = [12.125,.26,3.125]
B = [13.125,.27,3.30]
AKEY = [48,1,12]


def init(client, epoch='e'):
    return client.request('init', scene_id='r', scene_epoch=epoch,
        content_version='v', frame_id='map', resolution=.25,
        prob_hit=.9, prob_miss=.1, clamp_min=.12, clamp_max=.88,
        bounds={'min': [-50,-50,-1], 'max': [50,50,8]})


@pytest.fixture
def client():
    with OctomapClient(BINARY) as c:
        init(c)
        yield c


def fields(seq, origin, returns, robot='go2', stamp=None, epoch='e'):
    frame = prepare_lidar_frame(returns, origin, .25, 9, 40, BOUNDS)
    return dict(scene_id='r', scene_epoch=epoch, robot_id=robot,
        sensor_id='lidar', frame_seq=seq,
        capture_sim_time=float(seq if stamp is None else stamp),
        receive_wall_time=0., calibration_id='c', origin=origin,
        points=frame.hit_points.tolist(),
        protected_hit_points=frame.protected_hit_points.tolist(),
        free_rays=[{'endpoint': p.tolist(), 'source': 'lidar_measured_beyond_range',
                    'include_endpoint': False} for p in frame.free_endpoints],
        mark_range_m=9., max_range_m=40., compact=True)


def scan(client, seq, origin, returns, **kwargs):
    return client.request('integrate', **fields(seq, origin, returns, **kwargs))


def query(client, *points):
    return client.request('query', points=points)['cells']


def strip_request_id(response):
    return {k: v for k, v in response.items() if k != 'request_id'}


def test_crossing_far_ray_preserves_real_far_surface_and_updates_only_observed_hits(client):
    cache = MapCache(client.request('snapshot'))
    cache.apply(scan(client, 1, UAV, [A], robot='uav'))
    before = query(client, A)[0]
    assert before['key'] == AKEY and before['state'] == 'occupied'
    response = scan(client, 2, GO2, [A, B])
    cache.apply(response)
    actual = query(client, A)[0]
    assert actual['state'] == 'occupied'
    assert actual['log_odds'] == before['log_odds']  # Already at the configured hit clamp.
    assert actual['last_observed_sim_time'] == 2.
    assert response['accepted_hits'] == 0
    assert response['protected_hit_cells'] == 2
    assert response['refreshed_protected_hits'] == 1
    assert AKEY not in [c['key'] for c in response['changed_cells']]
    assert cache.cells[tuple(AKEY)]['last_observed_sim_time'] == 2.
    assert any(AKEY in group['keys'] for group in response['refreshes'])


def test_later_true_crossing_ray_clears_departed_surface_without_permanent_lock(client):
    scan(client, 1, UAV, [A], robot='uav')
    scan(client, 2, GO2, [A, B])
    scan(client, 3, GO2, [B])
    actual = query(client, A)[0]
    assert actual['state'] == 'free'
    assert actual['last_observed_sim_time'] == 3.
    # A known surface gets symmetric real positive evidence at far range.
    recovered = scan(client, 4, GO2, [A, B])
    assert query(client, A)[0]['state'] == 'occupied'
    assert recovered['recovered_far_hit_cells'] == 1
    assert recovered['refreshed_protected_hits'] == 1
    scan(client, 5, GO2, [B])
    assert query(client, A)[0]['state'] == 'free'


def test_protected_far_return_does_not_create_unknown_obstacle_or_certify_it_free(client):
    response = scan(client, 1, GO2, [A, B])
    assert [v['state'] for v in query(client, A, B)] == ['unknown','unknown']
    assert response['accepted_hits'] == response['refreshed_protected_hits'] == 0


def test_low_go2_rays_do_not_clear_or_refresh_high_uav_obstacle(client):
    scan(client, 1, UAV, [A], robot='uav')
    before = query(client, A)[0]
    scan(client, 2, GO2, [[13.125,.26,1.125]])
    assert query(client, A)[0] == before


@pytest.mark.parametrize('stamp', [1.9, 2.])
def test_cross_robot_older_or_equal_free_cannot_erase_new_far_hit(client, stamp):
    scan(client, 1, UAV, [A], robot='uav')
    scan(client, 2, GO2, [A, B], stamp=2.)
    scan(client, 3, GO2, [B], robot='other_lidar', stamp=stamp)
    assert query(client, A)[0]['state'] == 'occupied'
    assert query(client, A)[0]['last_observed_sim_time'] == 2.
    scan(client, 4, GO2, [B], robot='other_lidar', stamp=2.1)
    assert query(client, A)[0]['state'] == 'free'


def test_old_far_hit_does_not_renew_newer_metadata(client):
    scan(client, 1, UAV, [A], robot='uav', stamp=2.)
    scan(client, 2, GO2, [A, B], stamp=1.9)
    assert query(client, A)[0]['last_observed_sim_time'] == 2.


def test_restored_far_hit_first_observation_is_classified(client, tmp_path):
    scan(client, 1, UAV, [A], robot='uav')
    path = str(tmp_path / 'restore.ot')
    client.request('save', path=path)
    with OctomapClient(BINARY) as restored:
        init(restored, epoch='restored')
        restored.request('load', path=path)
        assert query(restored, A)[0]['last_observed_sim_time'] is None
        response = scan(restored, 2, GO2, [A, B], epoch='restored')
        occupied = next(v for v in response['changed_cells'] if v['key'] == AKEY)
        assert occupied['state'] == 'occupied'
        assert occupied['last_observed_sim_time'] == 2.
        assert response['refreshed_protected_hits'] == 1


@pytest.mark.parametrize('bad', [
    'not_an_array', [[1,2]], [['bad',0,0]], [[1e308,1e308,1e308]],
    [[2,.125,1.125]], [[41,.125,1.125]], [[12,.125,9]],
])
def test_malformed_protection_rejects_whole_frame_and_allows_same_sequence_retry(client, bad):
    scan(client, 1, UAV, [A], robot='uav')
    before = strip_request_id(client.request('snapshot'))
    request = fields(2, GO2, [A,B])
    request['protected_hit_points'] = bad
    with pytest.raises(MapError):
        client.request('integrate', **request)
    assert strip_request_id(client.request('snapshot')) == before
    retry = scan(client, 2, GO2, [A,B])
    assert not retry['duplicate']
    assert query(client, A)[0]['state'] == 'occupied'


@pytest.mark.parametrize('mark', [None, -1, 0, 41, 'bad'])
def test_protection_requires_explicit_valid_mark_range(client, mark):
    request = fields(1, GO2, [A])
    if mark is None:
        request.pop('mark_range_m')
    else:
        request['mark_range_m'] = mark
    with pytest.raises(MapError):
        client.request('integrate', **request)
    assert client.request('status')['revision'] == 0


def test_far_hit_refresh_works_with_binary_execution_cache_and_presence_only_planner(client):
    cache = MapCache(client.request('snapshot'))
    cache.apply(scan(client, 1, UAV, [A], robot='uav'))
    baseline_revision = cache.revision
    request = fields(2, GO2, [A,B])
    request['refresh_encoding'] = 'i16le_base64'
    update = client.request('integrate', **request)
    cache.apply(update)
    assert cache.cells[tuple(AKEY)]['state'] == 'occupied'
    assert cache.cells[tuple(AKEY)]['last_observed_sim_time'] == 2.
    planner = client.request('changes', since_revision=baseline_revision,
                             compact=True, refresh_semantics='presence_only')
    assert AKEY not in [v['key'] for v in planner['changed_cells']]
    assert planner['refreshes'] == []



def known_count(client):
    return client.request('status')['known_near_hit_cells']


def test_far_positive_does_not_promote_free_only_observed_cell(client):
    scan(client, 1, GO2, [B])
    before = query(client, A)[0]
    assert before['state'] == 'free'
    assert known_count(client) == 0
    response = scan(client, 2, GO2, [A, B])
    assert query(client, A)[0] == before
    assert response['refreshed_protected_hits'] == 0
    assert response['recovered_far_hit_cells'] == 0
    assert known_count(client) == 0


def test_older_near_hit_does_not_authorize_far_recovery(client):
    scan(client, 1, GO2, [B], stamp=2.)
    before = query(client, A)[0]
    scan(client, 1, UAV, [A], robot='uav', stamp=1.9)
    assert query(client, A)[0] == before
    assert known_count(client) == 0
    scan(client, 2, GO2, [A], stamp=2.1)
    assert query(client, A)[0] == before
    assert known_count(client) == 0


def test_older_far_hit_cannot_restore_newer_free_but_equal_hit_wins(client):
    scan(client, 1, UAV, [A], robot='uav', stamp=1.)
    scan(client, 2, GO2, [B], stamp=2.)
    before = query(client, A)[0]
    assert before['state'] == 'free'
    old = scan(client, 3, GO2, [A], stamp=1.9)
    assert query(client, A)[0] == before
    assert old['refreshed_protected_hits'] == old['recovered_far_hit_cells'] == 0
    equal = scan(client, 4, GO2, [A], stamp=2.)
    assert query(client, A)[0]['state'] == 'occupied'
    assert query(client, A)[0]['last_observed_sim_time'] == 2.
    assert equal['recovered_far_hit_cells'] == 1


def test_invalid_frame_does_not_leak_near_hit_provenance(client):
    request = fields(1, UAV, [A], robot='uav')
    request['protected_hit_points'] = [[1,2]]
    with pytest.raises(MapError):
        client.request('integrate', **request)
    assert client.request('status')['revision'] == known_count(client) == 0
    scan(client, 1, GO2, [A])
    assert query(client, A)[0]['state'] == 'unknown'
    assert known_count(client) == 0
    retry = scan(client, 1, UAV, [A], robot='uav')
    assert not retry['duplicate']
    assert known_count(client) == 1


def test_invalid_compact_is_rejected_before_near_provenance_mutation(client):
    request = fields(1, UAV, [A], robot='uav')
    request['compact'] = 'not_a_boolean'
    with pytest.raises(MapError):
        client.request('integrate', **request)
    assert client.request('status')['revision'] == known_count(client) == 0
    assert query(client, A)[0]['state'] == 'unknown'
    assert not scan(client, 1, UAV, [A], robot='uav')['duplicate']


def test_explicit_mark_range_also_bounds_new_hit_points(client):
    request = fields(1, GO2, [])
    request['points'] = [A]  # Actual far return incorrectly put in the near-hit list.
    result = client.request('integrate', **request)
    assert result['accepted_hits'] == 0
    assert result['discarded_points'] == 1
    assert known_count(client) == 0
    assert query(client, A)[0]['state'] == 'unknown'


def test_forget_removes_provenance_until_new_in_range_hit(client):
    scan(client, 1, UAV, [A], robot='uav')
    assert known_count(client) == 1
    client.request('forget', scene_id='r', scene_epoch='e', keys=[AKEY], reason='operator removal')
    assert known_count(client) == 0
    scan(client, 2, GO2, [A])
    assert query(client, A)[0]['state'] == 'unknown'
    assert known_count(client) == 0
    scan(client, 3, UAV, [A], robot='uav')
    scan(client, 4, GO2, [B])
    assert query(client, A)[0]['state'] == 'free'
    scan(client, 5, GO2, [A])
    assert query(client, A)[0]['state'] == 'occupied'


def test_cleared_surface_provenance_survives_new_snapshot_and_epoch(client, tmp_path):
    scan(client, 1, UAV, [A], robot='uav')
    scan(client, 2, GO2, [B])
    assert query(client, A)[0]['state'] == 'free'
    path = str(tmp_path / 'known-cleared.ot')
    client.request('save', path=path)
    metadata = json.loads(Path(path+'.metadata.json').read_text())
    assert metadata['known_near_hit_keys'] == [AKEY]
    with OctomapClient(BINARY) as restored:
        init(restored, epoch='restored')
        assert known_count(restored) == 0
        restored.request('load', path=path)
        assert known_count(restored) == 1
        assert query(restored, A)[0]['last_observed_sim_time'] is None
        result = scan(restored, 1, GO2, [A], epoch='restored')
        assert query(restored, A)[0]['state'] == 'occupied'
        assert result['recovered_far_hit_cells'] == 1
        scan(restored, 2, GO2, [B], epoch='restored')
        assert query(restored, A)[0]['state'] == 'free'


def test_legacy_snapshot_migrates_only_occupied_cells(client, tmp_path):
    other = [12.125,1.125,3.125]
    scan(client, 1, UAV, [A, other], robot='uav')
    scan(client, 2, GO2, [B])
    assert [c['state'] for c in query(client, A, other)] == ['free', 'occupied']
    path = str(tmp_path / 'legacy.ot')
    client.request('save', path=path)
    meta_path = Path(path+'.metadata.json')
    metadata = json.loads(meta_path.read_text())
    metadata.pop('known_near_hit_keys')
    meta_path.write_text(json.dumps(metadata))
    with OctomapClient(BINARY) as restored:
        init(restored, epoch='restored')
        restored.request('load', path=path)
        assert known_count(restored) == 1
        result = scan(restored, 1, GO2, [A, other], epoch='restored')
        assert [c['state'] for c in query(restored, A, other)] == ['free', 'occupied']
        assert result['refreshed_protected_hits'] == 1
        assert result['recovered_far_hit_cells'] == 0
        assert query(restored, A)[0]['last_observed_sim_time'] is None
        assert query(restored, other)[0]['last_observed_sim_time'] == 1.


@pytest.mark.parametrize('bad', ['not_an_array', [[1,2]], [[48.1,1,12]],
                                  [[48,1,32768]], [[32000,32000,32000]]])
def test_invalid_saved_provenance_rejects_restore_atomically(client, tmp_path, bad):
    scan(client, 1, UAV, [A], robot='uav')
    path = str(tmp_path / 'invalid-provenance.ot')
    client.request('save', path=path)
    meta_path = Path(path+'.metadata.json')
    original = json.loads(meta_path.read_text())
    invalid = dict(original, known_near_hit_keys=bad)
    meta_path.write_text(json.dumps(invalid))
    with OctomapClient(BINARY) as restored:
        init(restored, epoch='restored')
        with pytest.raises(MapError):
            restored.request('load', path=path)
        state = restored.request('status')
        assert state['revision'] == state['leaf_count'] == state['known_near_hit_cells'] == 0
        assert query(restored, A)[0]['state'] == 'unknown'
        meta_path.write_text(json.dumps(original))
        restored.request('load', path=path)
        assert known_count(restored) == 1
        assert query(restored, A)[0]['state'] == 'occupied'


def test_recovered_far_hit_propagates_to_execution_and_planner(client):
    cache = MapCache(client.request('snapshot'))
    cache.apply(scan(client, 1, UAV, [A], robot='uav'))
    cache.apply(scan(client, 2, GO2, [B]))
    before_revision = cache.revision
    request = fields(3, GO2, [A])
    request['refresh_encoding'] = 'i16le_base64'
    result = client.request('integrate', **request)
    cache.apply(result)
    assert cache.cells[tuple(AKEY)]['state'] == 'occupied'
    assert cache.cells[tuple(AKEY)]['last_observed_sim_time'] == 3.
    planner = client.request('changes', since_revision=before_revision,
                             compact=True, refresh_semantics='presence_only')
    assert any(c['key'] == AKEY and c['state'] == 'occupied' for c in planner['changed_cells'])
    assert result['recovered_far_hit_cells'] == 1


def test_fresh_process_does_not_inherit_provenance(client):
    scan(client, 1, UAV, [A], robot='uav')
    with OctomapClient(BINARY) as fresh:
        init(fresh, epoch='new')
        assert known_count(fresh) == 0
        scan(fresh, 1, GO2, [A], epoch='new')
        assert query(fresh, A)[0]['state'] == 'unknown'
        assert known_count(fresh) == 0


def test_far_return_adds_normal_probability_once_without_forcing_occupancy():
    import math
    with OctomapClient(BINARY) as c:
        c.request('init', scene_id='r', scene_epoch='e', content_version='v',
            frame_id='map', resolution=.25, prob_hit=.7, prob_miss=.4,
            clamp_min=.12, clamp_max=.97,
            bounds={'min': [-50,-50,-1], 'max': [50,50,8]})
        scan(c, 1, UAV, [A], robot='uav')
        before = query(c, A)[0]['log_odds']
        result = scan(c, 2, GO2, [A])
        after = query(c, A)[0]['log_odds']
        assert after == pytest.approx(before + math.log(.7/.3), abs=1e-6)
        assert result['refreshed_protected_hits'] == 1
        assert result['recovered_far_hit_cells'] == 0
        duplicate = scan(c, 2, GO2, [A])
        assert duplicate['duplicate']
        assert query(c, A)[0]['log_odds'] == after
        for seq in range(3,13):
            scan(c, seq, GO2, [B])
        before = query(c, A)[0]['log_odds']
        assert query(c, A)[0]['state'] == 'free'
        result = scan(c, 13, GO2, [A])
        assert query(c, A)[0]['log_odds'] == pytest.approx(before + math.log(.7/.3), abs=1e-6)
        assert query(c, A)[0]['state'] == 'free'  # One weak hit need not recover it.
        assert result['refreshed_protected_hits'] == 1
        assert result['recovered_far_hit_cells'] == 0
        scan(c, 14, GO2, [A])
        recovered = scan(c, 15, GO2, [A])
        assert query(c, A)[0]['state'] == 'occupied'
        assert recovered['recovered_far_hit_cells'] == 1
