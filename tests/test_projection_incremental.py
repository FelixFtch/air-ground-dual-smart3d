"""Delta equivalence tests for the fixed uninflated display height band."""
import random

import pytest

from observability.projection import BodyHeightProjection, body_height_projection


def comparable(projection):
    return tuple(set(map(tuple, group)) for group in projection)


def apply(cells, projection, changes):
    # Mimic MapCache.apply's final state, including absent unknown keys.
    for change in changes:
        key = tuple(change["key"])
        if change["state"] == "unknown":
            cells.pop(key, None)
        else:
            cells[key] = dict(change)
    projection.apply(cells, changes)


def test_column_unknown_free_occupied_unknown_with_multiple_blockers():
    cells = {}
    projection = BodyHeightProjection(cells, .25, .33, .28)
    assert projection.snapshot() == ([], [])
    for z in (0, 1):
        apply(cells, projection, [{"key": [1, -2, z], "state": "free"}])
        assert projection.snapshot() == ([], [])
    apply(cells, projection, [{"key": [1, -2, 2], "state": "free"}])
    assert projection.snapshot() == ([[1, -2]], [])
    apply(cells, projection, [{"key": [1, -2, 0], "state": "occupied"},
                              {"key": [1, -2, 1], "state": "occupied"}])
    assert projection.snapshot() == ([], [[1, -2]])
    apply(cells, projection, [{"key": [1, -2, 0], "state": "unknown"}])
    assert projection.snapshot() == ([], [[1, -2]])
    apply(cells, projection, [{"key": [1, -2, 1], "state": "free"}])
    assert projection.snapshot() == ([], [])  # z=0 remains unknown.
    apply(cells, projection, [{"key": [1, -2, 0], "state": "free"}])
    assert projection.snapshot() == ([[1, -2]], [])
    apply(cells, projection, [{"key": [1, -2, z], "state": "unknown"} for z in (0, 1, 2)])
    assert projection.snapshot() == ([], [])
    assert not projection._states and not projection._columns


def test_restored_initialization_ignores_timestamps_and_floor_outside_band():
    cells = {(0, 0, -1): {"state": "occupied", "last_observed_sim_time": None},
             (0, 0, 0): {"state": "free", "last_observed_sim_time": None},
             (0, 0, 1): {"state": "free", "last_observed_sim_time": None},
             (0, 0, 2): {"state": "free", "last_observed_sim_time": None},
             (1, 0, 1): {"state": "occupied", "last_observed_sim_time": None},
             (2, 0, 1): {"state": "unknown", "last_observed_sim_time": None}}
    projection = BodyHeightProjection(cells, .25, .33, .28)
    assert comparable(projection.snapshot()) == comparable(body_height_projection(cells, .25, .33, .28))
    for cell in cells.values():
        cell["last_observed_sim_time"] = 10.
    projection.apply(cells, [])  # Compact refreshes do not alter geometry.
    assert projection.snapshot() == ([[0, 0]], [[1, 0]])
    apply(cells, projection, [{"key": [0, 0, -1], "state": "free"},
                              {"key": [0, 0, 3], "state": "occupied"}])
    assert projection.snapshot() == ([[0, 0]], [[1, 0]])


def test_offset_and_inclusive_negative_z_boundaries_match_baseline():
    cells = {(0, 0, z): {"state": "free"} for z in range(-3, 4)}
    cells[(0, 0, 0)]["state"] = "occupied"
    for settings in ((.25, -.5, .25, 0.), (.25, .33, .28, -.75), (.25, -.25, 0., 0.)):
        projection = BodyHeightProjection(cells, *settings)
        assert comparable(projection.snapshot()) == comparable(body_height_projection(cells, *settings))


def test_apply_reads_only_changed_in_band_keys_and_is_idempotent():
    projection = BodyHeightProjection({}, .25, .33, .28)
    class GetOnly(dict):
        def items(self):
            raise AssertionError("Full map scan during incremental update")
        def get(self, key, default=None):
            assert 0 <= key[2] <= 2
            return super().get(key, default)
    cells = GetOnly({(0, 0, 1): {"state": "occupied"}})
    changes = [{"key": [0, 0, 1], "state": "unknown"},
               {"key": [0, 0, 1], "state": "occupied"},
               {"key": [0, 0, 20], "state": "occupied"}]
    projection.apply(cells, changes)
    projection.apply(cells, changes)
    assert projection.snapshot() == ([], [[0, 0]])
    assert projection._columns[(0, 0)] == [0, 1]


def test_snapshot_mutation_does_not_change_incremental_state():
    projection = BodyHeightProjection({(0, 0, 1): {"state": "occupied"}}, .25, .33, .28)
    free, blocked = projection.snapshot()
    blocked[0][0] = 99
    free.append([99, 99])
    assert projection.snapshot() == ([], [[0, 0]])


@pytest.mark.parametrize("seed", [2, 11, 91])
def test_random_delta_sequences_equal_full_projection_after_every_revision(seed):
    rng = random.Random(seed)
    keys = [(x, y, z) for x in range(-6, 7) for y in range(-6, 7) for z in range(-4, 10)]
    cells = {key: {"state": rng.choice(("free", "occupied"))} for key in rng.sample(keys, 700)}
    settings = (.25, .33, .28, -.25)
    projection = BodyHeightProjection(cells, *settings)
    assert comparable(projection.snapshot()) == comparable(body_height_projection(cells, *settings))
    for _ in range(100):
        changes = [{"key": list(rng.choice(keys)), "state": rng.choice(("unknown", "free", "occupied"))}
                   for _ in range(rng.randrange(1, 80))]
        apply(cells, projection, changes)
        assert comparable(projection.snapshot()) == comparable(body_height_projection(cells, *settings))


@pytest.mark.parametrize("settings", [(0, 0, .2), (-1, 0, .2), (.25, 0, -.1), (.25, float("nan"), .1)])
def test_invalid_band_rejected(settings):
    with pytest.raises(ValueError, match="invalid_body_projection_band"):
        BodyHeightProjection({}, *settings)


def test_immutable_projection_invalidates_only_changed_membership():
    cells={(1,2,0):{'state':'occupied'}}
    projection=BodyHeightProjection(cells,.25,.33,.28)
    initial=projection.immutable_snapshot()
    assert initial==((),((1,2),))
    with pytest.raises(TypeError):initial[1][0][0]=9
    apply(cells,projection,[{'key':[1,2,1],'state':'occupied'}])
    second=projection.immutable_snapshot()
    assert all(a is b for a,b in zip(initial,second))  # Another blocker, same XY.
    apply(cells,projection,[{'key':[1,2,0],'state':'free'}])
    assert projection.immutable_snapshot()[1] is initial[1]
    apply(cells,projection,[{'key':[1,2,1],'state':'free'}, {'key':[1,2,2],'state':'free'}])
    assert projection.immutable_snapshot()==(((1,2),),())
    assert initial==((),((1,2),))
    # Legacy caller still receives independently mutable coordinate lists.
    legacy=projection.snapshot();legacy[0][0][0]=99
    assert projection.immutable_snapshot()==(((1,2),),())
