"""Map-frame identities for per-frame detections: continuity, re-identification, expiry."""
import numpy as np

from perception.map_tracker import MapTracker, appearance, similarity


def colour(index, size=154):
    histogram = np.zeros(size)
    histogram[index] = 1.
    return histogram


RED, BLUE = colour(10), colour(90)


def test_continuous_walk_keeps_one_identity():
    tracker = MapTracker()
    ids = {tracker.update(.35 * i, [([.35 * i, 0.], RED)])[0] for i in range(12)}
    assert ids == {0}


def test_far_detection_is_a_new_person():
    tracker = MapTracker()
    first = tracker.update(0., [([0., 0.], RED)])[0]
    assert tracker.update(.35, [([3., 0.], RED)])[0] != first


def test_returning_person_recovers_identity_with_the_same_clothing():
    tracker = MapTracker()
    for i in range(6):
        tracker.update(.35 * i, [([.35 * i, 0.], RED)])
    # Out of view for six seconds, back near the extrapolated walk.
    assert tracker.update(1.75 + 6., [([1.75 + 3. + .4, .2], RED)]) == [0]


def test_long_gap_needs_matching_clothing():
    tracker = MapTracker()
    for i in range(6):
        tracker.update(.35 * i, [([.35 * i, 0.], RED)])
    assert tracker.update(1.75 + 6., [([1.75 + 3., 0.], BLUE)]) != [0]


def test_short_gap_follows_position_even_without_appearance():
    tracker = MapTracker()
    tracker.update(0., [([0., 0.], None)])
    assert tracker.update(.35, [([.3, 0.], None)]) == [0]


def test_lost_track_expires():
    tracker = MapTracker(lost_ttl_s=20.)
    tracker.update(0., [([0., 0.], RED)])
    assert tracker.update(20.5, [([0., 0.], RED)]) != [0]
    assert 0 not in tracker.tracks


def test_two_people_crossing_keep_their_identities():
    tracker = MapTracker()
    seen = []
    for i in range(10):
        t = .35 * i
        a, b = [-1.5 + .35 * i, .25], [1.5 - .35 * i, -.25]
        seen.append(tracker.update(t, [(a, RED), (b, BLUE)]))
    assert all(ids == seen[0] for ids in seen)


def test_detection_without_map_position_gets_a_fresh_untracked_id():
    tracker = MapTracker()
    ids = tracker.update(0., [(None, RED), ([0., 0.], RED)])
    assert len(set(ids)) == 2 and ids[0] not in tracker.tracks and ids[1] in tracker.tracks


def test_appearance_histogram_is_normalised_and_needs_enough_pixels():
    rgb = np.zeros((40, 40, 3), np.uint8)
    rgb[:20] = (200, 30, 30)
    rgb[20:] = (128, 128, 128)
    mask = np.ones((40, 40), bool)
    histogram = appearance(rgb, mask)
    assert abs(histogram.sum() - 1.) < 1e-9
    assert similarity(histogram, histogram) > .999
    assert appearance(rgb, np.zeros((40, 40), bool)) is None
    red_only = appearance(rgb[:20], np.ones((20, 40), bool))
    grey_only = appearance(rgb[20:], np.ones((20, 40), bool))
    assert similarity(red_only, grey_only) < .01
