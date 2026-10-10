import math

import pytest

from sim_io.fusion_schedule import choose_candidate


def item(robot, kind):
    return (0, robot, kind, object(), 0)


def period(robot, kind):
    return .2 if kind == 'lidar' else (.5 if kind == 'depth' else 1.5)


def team():
    return ([item('go', 'lidar'), item('uav', 'lidar'),
             item('go', 'depth'), item('uav', 'depth')]
            + [item('uav', 'depth_aux_' + str(i)) for i in range(6)])


def simulate(duration, cost):
    """All streams have current candidates; cost blocks the single mapper."""
    candidates = team()
    last, starts = {}, {entry[1:3]: [] for entry in candidates}
    now = 0.
    while now < duration:
        chosen = choose_candidate(candidates, last, now, period)
        if chosen is None:
            now += .001
            continue
        key = chosen[1:3]
        starts[key].append(now)
        last[key] = now
        now += cost(key)
    return starts


def test_overdue_auxiliary_can_precede_due_lidar():
    go = item('go', 'lidar')
    uav = item('uav', 'lidar')
    aux = item('uav', 'depth_aux_0')
    assert choose_candidate([aux, go, uav],
                            {('go', 'lidar'): .7, ('uav', 'lidar'): .65,
                             ('uav', 'depth_aux_0'): -20}, 1, period) is aux


def test_more_overdue_relative_deadline_wins_not_absolute_sensor_type():
    lidar = item('go', 'lidar')
    depth = item('go', 'depth')
    # LiDAR is 1 period late; depth is only .4 period late.
    assert choose_candidate([depth, lidar],
                            {('go', 'lidar'): .6, ('go', 'depth'): .3},
                            1, period) is lidar


def test_enough_budget_preserves_five_hz_target_for_each_lidar():
    duration = 120
    starts = simulate(duration, lambda key: .005)
    for rid in ('go', 'uav'):
        assert 4.9 <= len(starts[(rid, 'lidar')]) / duration <= 5.01
        assert 1.95 <= len(starts[(rid, 'depth')]) / duration <= 2.01
    assert all(len(starts[('uav', 'depth_aux_' + str(i))]) / duration >= .65
               for i in range(6))
    for key, times in starts.items():
        assert all(b - a >= period(*key) - 1e-9 for a, b in zip(times, times[1:]))


def test_overloaded_mapper_degrades_all_sources_without_depth_starvation():
    duration = 120
    # Requested utilization is 18 integrations/s * .12 s = 2.16 (> 1).
    # With absolute LiDAR priority the 2 LiDAR alone overload the mapper.
    starts = simulate(duration, lambda key: .12)
    for key, times in starts.items():
        assert len(times) >= 25, (key, len(times))
        gaps = [times[0], *[b - a for a, b in zip(times, times[1:])], duration - times[-1]]
        assert max(gaps) < 6., (key, max(gaps))
    assert len(starts[('go', 'lidar')]) > len(starts[('go', 'depth')])
    assert len(starts[('go', 'depth')]) > len(starts[('uav', 'depth_aux_0')])
    assert abs(len(starts[('go', 'lidar')]) - len(starts[('uav', 'lidar')])) <= 2


def test_variable_cost_overload_keeps_all_six_clearance_views_alive():
    starts = simulate(90, lambda key: .04 if key[1] == 'lidar' else .16)
    assert all(len(times) > 15 for times in starts.values())
    assert all(max(b-a for a,b in zip(times,times[1:])) < 6
               for times in starts.values())


def test_absent_sensors_do_not_reserve_slots_and_new_sensor_is_immediately_due():
    lidar = item('go', 'lidar')
    depth = item('go', 'depth')
    assert choose_candidate([depth], {('go', 'lidar'): 0}, 1, period) is depth
    assert choose_candidate([lidar], {('go', 'lidar'): .9}, 1, period) is None


def test_fresh_depth_is_not_fused_repeatedly_ahead_of_its_period():
    d = item('go', 'depth')
    assert choose_candidate([d], {('go', 'depth'): 1}, 1.1, period) is None
    assert choose_candidate([d], {('go', 'depth'): 1}, 1.5, period) is d


@pytest.mark.parametrize('bad', [0, -1, math.nan, math.inf])
def test_invalid_period_rejected(bad):
    with pytest.raises(ValueError):
        choose_candidate([item('go', 'lidar')], {}, 1, lambda *args: bad)


def test_invalid_times_rejected():
    with pytest.raises(ValueError):
        choose_candidate([item('go', 'lidar')], {}, math.nan, period)
    with pytest.raises(ValueError):
        choose_candidate([item('go', 'lidar')], {('go', 'lidar'): math.inf}, 1, period)
