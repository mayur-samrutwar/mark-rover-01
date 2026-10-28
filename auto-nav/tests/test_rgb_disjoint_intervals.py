"""Disjoint image-floor evidence never clears a gap or an occluder shadow."""
from dataclasses import replace
import math
from pathlib import Path
import json
import hashlib

import numpy as np
import pytest

from rover_nav.navigation import FREE, OCCUPIED, UNKNOWN, Navigator, Observation, Pose


def frame(segments, *, angles=None, hits=None, contacts=None):
    angles = np.linspace(-.8, .8, 101) if angles is None else np.asarray(angles, float)
    intervals = tuple(segments for _ in angles)
    return Observation(Pose(0., 0., 0.), angles, np.full(len(angles), .3), 2.5, 1.,
        min_ranges=np.full(len(angles), .2),
        hits=np.zeros(len(angles), bool) if hits is None else hits,
        hit_ranges=contacts, free_intervals=intervals)


def navigator():
    nav = Navigator(bounds=(-2., -2., 4., 4.), resolution=.05,
                    robot_radius=.25, safety_margin=.08, account_cell_area=True)
    nav.grid.fill(FREE)
    nav._observed.fill(True)
    nav._evidence.fill(-4)
    nav.pose = Pose(0., 0., 0.)
    nav.status, nav.reason = "following", "frontier_goal"
    nav.goal, nav.path = (1., 0.), [(0., 0.), (.2, 0.), (.5, 0.), (1., 0.)]
    return nav


def reason(nav, observation, speed=.25):
    return nav._rgb_swept_stop_reason(observation, speed, 0., nav.traversable())


def test_integrates_each_segment_and_leaves_the_gap_and_prefix_unknown():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = frame(((.2, .35), (.6, .9)), angles=[-.05, 0., .05])
    nav.integrate(observation)
    for distance in (.25, .7):
        assert nav.grid[nav.world_to_cell(distance, 0.)] == FREE
    for distance in (.1, .45, 1.):
        assert nav.grid[nav.world_to_cell(distance, 0.)] == UNKNOWN
    assert not np.any(nav.grid == OCCUPIED)


def test_explicit_empty_union_is_authoritative_over_legacy_first_pair():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = frame((), angles=[-.05, 0., .05])
    nav.integrate(observation)
    assert np.all(nav.grid == UNKNOWN)
    assert reason(navigator(), observation) == "insufficient_current_forward_floor"


def test_none_retains_the_single_interval_compatibility_contract():
    observation = replace(frame(()), ranges=np.full(101, 2.), free_intervals=None)
    assert reason(navigator(), observation) is None


def test_later_segment_corroborates_without_extending_the_first_segment():
    nav = navigator()
    observation = frame(((.225, .275), (.375, .7)))
    assert reason(nav, replace(observation, free_intervals=None)) == "insufficient_current_forward_floor"
    assert reason(nav, observation) is None


def test_convex_hull_of_segments_cannot_corroborate_unobserved_gap():
    observation = frame(((.2, .3), (.7, 1.)))
    assert reason(navigator(), observation) == "insufficient_current_forward_floor"


def test_adjacent_bearings_need_an_actual_common_segment():
    observation = frame(((.2, .3), (.375, .44)), angles=[-.1, 0., .1])
    observation = replace(observation, free_intervals=(
        ((.2, .3), (.375, .44)), ((.2, .3), (.55, .7)), ((.2, .3), (.375, .44))))
    assert reason(navigator(), observation) == "insufficient_current_forward_floor"


def test_union_intersection_can_pair_segments_with_different_indices():
    observation = frame((), angles=[-.1, 0., .1])
    observation = replace(observation, free_intervals=(
        ((.2, .3), (.375, .7)), ((.37, .7),), ((.2, .3), (.375, .7))))
    assert reason(navigator(), observation) is None


def test_empty_original_bearing_cannot_be_skipped_to_find_later_support():
    observation = frame(((.2, .3), (.375, .7)), angles=[-.1, 0., .1])
    observation = replace(observation, free_intervals=(observation.free_intervals[0], (), observation.free_intervals[2]))
    assert reason(navigator(), observation) == "insufficient_current_forward_floor"


def test_all_segments_clip_at_positive_contact_and_leave_its_shadow_unknown():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = frame(((.2, .3), (.5, .7), (1.1, 1.5)), angles=[0.],
                        hits=np.ones(1, bool), contacts=np.array([.6]))
    nav.integrate(observation)
    for distance in (.25, .55):
        assert nav.grid[nav.world_to_cell(distance, 0.)] == FREE
    assert nav.grid[nav.world_to_cell(.6, 0.)] == OCCUPIED
    for distance in (.4, .7, 1.2):
        assert nav.grid[nav.world_to_cell(distance, 0.)] == UNKNOWN


def test_positive_contact_before_all_segments_blocks_every_segment():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = frame(((.2, .3), (.5, .7)), angles=[0.],
                        hits=np.ones(1, bool), contacts=np.array([.15]))
    nav.integrate(observation)
    assert nav.grid[nav.world_to_cell(.15, 0.)] == OCCUPIED
    assert not np.any(nav.grid == FREE)


def test_witness_does_not_use_a_segment_behind_a_positive_contact():
    nav = navigator()
    observation = frame(((.2, .3), (.375, .7)), hits=np.ones(101, bool),
                        contacts=np.full(101, .35))
    assert not nav._rgb_forward_floor_witness(observation, nav._rgb_braking_paths(.25, 0.), np.zeros(2))


INVALID_BEARINGS = [
    [(.2, .4)], ((.2, .4), (.3, .5)), ((.5, .7), (.2, .4)),
    ((.4, .4),), ((.4, .2),), ((-.1, .4),), ((np.nan, .4),),
    ((.2, np.inf),), ((.2, 3.),), ((".2", .4),), ((False, .4),),
    ([.2, .4],), ((.2, .4, .5),),
]


@pytest.mark.parametrize("bearing", INVALID_BEARINGS)
def test_invalid_union_pauses_before_changing_map_or_accepted_clock(bearing):
    nav = navigator()
    nav._last_timestamp = .5
    nav._last_command = (.2, 0.)
    observation = replace(frame(()), free_intervals=(bearing,) * 101)
    before = (nav.grid.copy(), nav._evidence.copy())
    assert nav.update(observation) == (0., 0.)
    assert nav.reason == "invalid_range_intervals"
    assert nav._last_timestamp == .5
    np.testing.assert_array_equal(nav.grid, before[0])
    np.testing.assert_array_equal(nav._evidence, before[1])


@pytest.mark.parametrize("intervals", [[], (), ((),), [()] * 101])
def test_invalid_outer_union_shape_is_rejected(intervals):
    nav = navigator()
    assert nav.update(replace(frame(()), free_intervals=intervals)) == (0., 0.)
    assert nav.reason == "invalid_range_intervals"


def test_explicit_floor_union_requires_independent_hit_flags():
    nav = navigator()
    assert nav.update(replace(frame(((.2, .7),)), hits=None)) == (0., 0.)
    assert nav.reason == "invalid_range_intervals"


def test_malformed_union_cannot_clear_but_independent_contact_survives_direct_integration():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = frame(((.2, .3),), angles=[0.], hits=np.ones(1, bool), contacts=np.array([.4]))
    nav.integrate(replace(observation, free_intervals=(((.5, .2),),)))
    assert nav.grid[nav.world_to_cell(.4, 0.)] == OCCUPIED
    assert not np.any(nav.grid == FREE)


def test_low_speed_confirmation_starts_at_physical_stop_before_extra_buffer():
    nav = navigator()
    observation = frame(((.225, .275),))
    assert reason(nav, observation, .035) is None
    assert reason(nav, observation, .25) == "insufficient_current_forward_floor"


def test_actual_doorway_rgb_allows_verified_slow_motion_without_filling_texture_gap():
    fixtures = Path(__file__).parent / "fixtures"
    saved = json.loads((fixtures / "rgb_doorway_disjoint_intervals.json").read_text())
    assert hashlib.sha256((fixtures / saved["fixture_image"]).read_bytes()).hexdigest() == saved["source_image_sha256"]
    samples = saved["intervals"]
    segments = tuple(tuple(tuple(pair) for pair in sample["segments_m"]) for sample in samples)
    near = np.array([pairs[0][0] if pairs else np.nan for pairs in segments])
    far = np.array([pairs[0][1] if pairs else np.nan for pairs in segments])
    contacts = np.array([sample["positive_hit_range_m"]
                         if sample["positive_hit_range_m"] is not None else np.nan
                         for sample in samples])
    offset = tuple(np.asarray(saved["calibration"]["T_base_camera"])[:2, 3])
    observation = Observation(Pose(0., 0., 0.), np.array([s["angle_rad"] for s in samples]),
        far, 2.5, saved["source_time"], sensor_origin=offset, min_ranges=near,
        hits=np.isfinite(contacts), hit_ranges=contacts, free_intervals=segments)
    nav = navigator()
    assert nav._valid_free_intervals(observation)
    assert reason(nav, observation, .035) is None
    assert reason(nav, observation, .25) == "insufficient_current_forward_floor"
    linear, angular = nav._follow(observation, nav.traversable())
    assert 0. < linear < .25
    assert reason(nav, observation, linear) is None
    assert angular == pytest.approx(0.)
    # Isolate the original centre bearing's two real intervals for mapping.
    middle = len(samples) // 2
    assert segments[middle] == ((.225, .325), (.55, 2.5))
    mapping = Navigator(bounds=(-1., -1., 3., 2.), resolution=.025)
    centre = replace(observation, angles=observation.angles[middle:middle + 1],
        ranges=far[middle:middle + 1], min_ranges=near[middle:middle + 1],
        hits=observation.hits[middle:middle + 1], hit_ranges=contacts[middle:middle + 1],
        free_intervals=(segments[middle],))
    mapping.integrate(centre)
    for distance in (.275, .7):
        assert mapping.grid[mapping.world_to_cell(offset[0] + distance, offset[1])] == FREE
    assert mapping.grid[mapping.world_to_cell(offset[0] + .45, offset[1])] == UNKNOWN
