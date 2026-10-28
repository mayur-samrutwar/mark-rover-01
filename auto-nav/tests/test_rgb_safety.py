"""RGB stopping geometry and recovery progress, without simulator truth."""
import math

import numpy as np
import pytest

from rover_nav.navigation import FREE, OCCUPIED, UNKNOWN, Navigator, Observation, Pose


def clear_nav(pose=Pose(0., 0., 0.)):
    nav = Navigator(bounds=(-2., -2., 4., 4.), resolution=.025,
                    robot_radius=.25, safety_margin=.08)
    nav.grid.fill(FREE)
    nav._observed.fill(True)
    nav._evidence.fill(-12)
    nav.pose = pose
    nav.status = "following"
    nav.reason = "frontier_goal"
    nav.goal = (pose.x + math.cos(pose.yaw), pose.y + math.sin(pose.yaw))
    nav.path = [(pose.x + step * math.cos(pose.yaw),
                 pose.y + step * math.sin(pose.yaw)) for step in (0., .2, .5, 1.)]
    return nav


def endpoints(points=(), *, pose=Pose(0., 0., 0.), offset=(0., 0.), hits=True,
              timestamp=1., floor_proof=True):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    relative = points - offset
    angles = np.arctan2(relative[:, 1], relative[:, 0])
    ranges = np.linalg.norm(relative, axis=1)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    world_offset = np.array([[c, -s], [s, c]]) @ offset
    observation = Observation(pose, angles, ranges, 3., timestamp,
                       sensor_origin=(pose.x + world_offset[0], pose.y + world_offset[1]),
                       hits=np.full(len(points), hits, bool))
    return with_floor_proof(observation) if floor_proof else observation


def with_floor_proof(observation):
    """Separate broad current floor evidence for positive-contact geometry tests."""
    from dataclasses import replace
    angles = np.linspace(-.8, .8, 101)
    minimum = (np.zeros(len(observation.ranges)) if observation.min_ranges is None
               else observation.min_ranges)
    contacts = (observation.ranges if observation.hit_ranges is None
                else observation.hit_ranges)
    return replace(observation,
        angles=np.concatenate((observation.angles, angles)),
        ranges=np.concatenate((observation.ranges, np.full(len(angles), 3.))),
        min_ranges=np.concatenate((minimum, np.zeros(len(angles)))),
        hits=np.concatenate((observation.hits, np.zeros(len(angles), bool))),
        hit_ranges=np.concatenate((contacts, np.full(len(angles), np.nan))))


def check(nav, observation, linear=.25, angular=0., **kwargs):
    return nav._rgb_swept_stop_reason(observation, linear, angular,
                                       nav.traversable(), **kwargs)


def test_reaction_distance_stops_before_the_shorter_old_envelope():
    nav = clear_nav()
    # Clearance .175 m exceeds .06 + v²/(2a), but not the 0.35 s reaction bound.
    assert check(nav, endpoints([[.47, 0.]])) == "local_obstacle_stop"
    assert check(nav, endpoints([[.60, 0.]])) is None


def test_rgb_follow_can_choose_a_slower_verified_stopping_sweep():
    nav = clear_nav()
    observation = endpoints([[.47, 0.]])
    linear, angular = nav._follow(observation, nav.traversable())
    assert 0. < linear < .25
    assert nav.reason == "clearance_limited_speed"
    assert check(nav, observation, linear, angular) is None


def test_lower_speed_candidate_cannot_erase_previous_motion_bound():
    nav = clear_nav()
    nav._last_command = (.25, 0.)
    linear, _ = nav._follow(endpoints([[.47, 0.]]), nav.traversable())
    assert linear == 0.
    assert nav.reason == "local_obstacle_stop"


def test_close_positive_obstacle_rejects_every_slow_speed_candidate():
    nav = clear_nav()
    linear, _ = nav._follow(endpoints([[.29, 0.]]), nav.traversable())
    assert linear == 0.
    assert nav.reason == "local_obstacle_stop"


def test_texture_unknown_boundary_is_not_a_positive_obstacle():
    nav = clear_nav()
    assert check(nav, endpoints([[.30, 0.]], hits=False)) is None
    assert check(nav, endpoints([[.30, 0.]], hits=True)) == "local_obstacle_stop"


def test_unknown_endpoint_does_not_create_an_occupied_map_cell():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = endpoints([[.5, 0.]], hits=False, floor_proof=False)
    nav.integrate(observation)
    assert nav.grid[nav.world_to_cell(.6, 0.)] == UNKNOWN
    assert not np.any(nav.grid == OCCUPIED)


def test_lateral_object_outside_the_sweep_does_not_stop_straight_drive():
    nav = clear_nav()
    assert check(nav, endpoints([[.2, .4]])) is None


def test_close_side_hit_outside_central_28_degrees_still_stops():
    nav = clear_nav()
    observation = endpoints([[.13, .24]])
    assert abs(observation.angles[0]) > math.radians(28)
    assert check(nav, observation) == "local_obstacle_stop"


@pytest.mark.parametrize("yaw", [0., math.pi / 2, -1.2])
def test_calibrated_sensor_offset_and_world_yaw_preserve_base_geometry(yaw):
    pose = Pose(.4, -.1, yaw)
    nav = clear_nav(pose)
    observation = endpoints([[.45, .01]], pose=pose, offset=(-.2, .06))
    assert observation.ranges[0] > .6
    assert check(nav, observation) == "local_obstacle_stop"


def test_current_overlap_stops_even_while_rotating_in_place():
    nav = clear_nav()
    assert check(nav, endpoints([[-.10, .12]]), linear=0., angular=.8) == "local_obstacle_stop"


def test_wholly_behind_obstacle_does_not_block_forward_stopping_path():
    nav = clear_nav()
    assert check(nav, endpoints([[-.6, 0.]])) is None


def test_inside_turn_obstacle_is_detected_outside_straight_corridor():
    nav = clear_nav()
    paths = nav._rgb_braking_paths(.25, .8)
    end = max((path[-1] for path in paths), key=lambda point: point[1])
    point = end + [0., .29]
    assert point[1] > .295
    observation = endpoints([point])
    assert check(nav, observation, angular=0.) is None
    assert check(nav, observation, angular=.8) == "local_obstacle_stop"


def test_previous_translation_is_still_braked_after_a_zero_speed_command():
    nav = clear_nav()
    nav._last_command = (.25, 0.)
    assert check(nav, endpoints([[.47, 0.]]), linear=0., angular=.5) == "local_obstacle_stop"


def test_older_exposure_increases_required_stopping_distance():
    nav = clear_nav()
    observation = endpoints([[.54, 0.]])
    assert check(nav, observation) is None
    assert check(nav, observation, observation_age=.25) == "local_obstacle_stop"


def test_entire_braking_path_must_remain_in_verified_free_map():
    nav = clear_nav()
    # The selected waypoint at 4 cm is safe, but the full 21 cm stopping
    # motion reaches the inflation halo of this unseen region.
    for row, col in np.ndindex(nav.shape):
        if nav.cell_to_world((row, col))[0] >= .5:
            nav.grid[row, col] = UNKNOWN
    safe = nav.traversable()
    assert safe[nav.world_to_cell(.04, 0.)]
    assert not safe[nav.world_to_cell(.20, 0.)]
    assert nav._rgb_swept_stop_reason(endpoints(), .25, 0., safe) == "braking_path_unknown"


def test_invalid_and_nonfinite_hit_values_cannot_create_false_obstacles():
    nav = clear_nav()
    observation = Observation(Pose(0., 0., 0.), np.array([0., np.nan, .3]),
                              np.array([np.nan, .2, -1.]), 3., 1., hits=np.ones(3, bool))
    assert check(nav, with_floor_proof(observation)) is None
    bad_origin = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([1.]),
        3., 1., sensor_origin=(math.nan, 0.), hits=np.ones(1, bool))
    assert check(nav, bad_origin) == "invalid_rgb_sensor_origin"


def test_rgb_follow_caps_speed_before_evaluating_braking_motion():
    nav = clear_nav()
    nav.goal = (.30, 0.)
    nav.path = [(0., 0.), (.15, 0.), (.30, 0.)]
    # Safe at 0.13 m/s (sweep ends .4174 m), unsafe at 0.25 m/s (.505 m).
    observation = endpoints([[.47, 0.], [2., -.3], [2., .3]])
    observation.hits[1:] = False
    linear, angular = nav._follow(observation, nav.traversable())
    assert linear == pytest.approx(.13)
    assert angular == pytest.approx(0.)


def test_missing_central_rgb_evidence_does_not_authorize_forward_motion():
    nav = clear_nav()
    observation = endpoints([[1., -1.], [1., 1.]], hits=False, floor_proof=False)
    assert nav._follow(observation, nav.traversable())[0] == 0.


def test_baseline_without_explicit_hits_keeps_existing_stop_rule():
    nav = clear_nav()
    observation = Observation(Pose(0., 0., 0.), np.array([-.2, 0., .2]),
                              np.full(3, .35), 3., 1.)
    assert nav._follow(observation, nav.traversable()) == (0., 0.)
    assert nav.reason == "local_obstacle_stop"


def test_jitter_and_spin_do_not_renew_recovery_but_new_region_does():
    nav = clear_nav()
    nav._fail_goal("test_failure", 1.)
    assert nav._failed_goals == 1
    original_blacklist = nav._blacklist.copy()
    assert not nav._reset_failures_after_progress(Pose(.45, 0., 1.), 2.)
    assert not nav._reset_failures_after_progress(Pose(.45, .45, 2.), 3.)
    assert not nav._reset_failures_after_progress(Pose(0., 0., 3.), 4.)
    assert nav._failed_goals == 1
    assert nav._reset_failures_after_progress(Pose(.81, 0., 3.), 5.)
    assert nav._failed_goals == 0
    assert nav._failure_anchor is None
    assert nav._blacklist == original_blacklist


def test_four_failures_in_the_same_region_still_exhaust_recovery():
    nav = clear_nav()
    for index in range(4):
        nav.pose = Pose(.02 * index, 0., float(index))
        nav.goal = (1., float(index))
        nav._fail_goal("test_failure", float(index))
    assert nav._failed_goals == 4
    assert nav.status == "stalled"
    assert nav.reason == "recovery_budget_exhausted"


def test_failure_in_new_region_starts_a_new_consecutive_budget():
    nav = clear_nav()
    for index in range(3):
        nav.goal = (1., float(index))
        nav._fail_goal("test_failure", float(index))
    nav.pose = Pose(.9, .1, 0.)
    nav.goal = (2., 1.)
    nav._fail_goal("unrelated_new_region_failure", 4.)
    assert nav._failed_goals == 1
    assert nav.status == "capture_sweep"
    assert len(nav._blacklist) == 4


def test_swept_collision_uses_contact_range_independently_of_free_endpoint():
    nav = clear_nav()
    observation = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([.25]),
        3., 1., min_ranges=np.array([.2]), hits=np.array([True]), hit_ranges=np.array([.70]))
    # The short verified-free interval is not an obstacle at the robot's nose.
    assert check(nav, with_floor_proof(observation)) is None
    close_contact = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([np.nan]),
        3., 1., min_ranges=np.array([np.nan]), hits=np.array([True]), hit_ranges=np.array([.40]))
    assert check(nav, close_contact) == "local_obstacle_stop"


@pytest.mark.parametrize("free_end", [np.nan, np.inf, -1., 0.])
def test_valid_positive_contact_survives_an_unusable_free_interval(free_end):
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([free_end]),
        3., 1., min_ranges=np.array([.2]), hits=np.array([True]), hit_ranges=np.array([.40]))
    nav.integrate(observation)
    assert nav.grid[nav.world_to_cell(.40, 0.)] == OCCUPIED
    assert nav.grid[nav.world_to_cell(.25, 0.)] == UNKNOWN
    assert nav.grid[nav.world_to_cell(.55, 0.)] == UNKNOWN
    assert not np.any(nav.grid == FREE)


@pytest.mark.parametrize("contact", [np.nan, np.inf, -1., 0.])
def test_invalid_contact_distance_never_invents_an_occupied_endpoint(contact):
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([.4]),
        3., 1., min_ranges=np.array([.2]), hits=np.array([True]), hit_ranges=np.array([contact]))
    nav.integrate(observation)
    assert not np.any(nav.grid == OCCUPIED)
    # Independently valid free evidence remains bounded by its own interval.
    assert nav.grid[nav.world_to_cell(.3, 0.)] == FREE
    assert nav.grid[nav.world_to_cell(.1, 0.)] == UNKNOWN
    assert nav.grid[nav.world_to_cell(.5, 0.)] == UNKNOWN


def test_zero_length_free_interval_can_still_place_a_distinct_contact():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([.3]),
        3., 1., min_ranges=np.array([.3]), hits=np.array([True]), hit_ranges=np.array([.4]))
    nav.integrate(observation)
    assert nav.grid[nav.world_to_cell(.4, 0.)] == OCCUPIED
    assert not np.any(nav.grid == FREE)


def test_malformed_free_interval_cannot_clear_through_a_closer_positive_contact():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([.8]),
        3., 1., min_ranges=np.array([.2]), hits=np.array([True]), hit_ranges=np.array([.4]))
    nav.integrate(observation)
    assert nav.grid[nav.world_to_cell(.3, 0.)] == FREE
    assert nav.grid[nav.world_to_cell(.4, 0.)] == OCCUPIED
    assert nav.grid[nav.world_to_cell(.5, 0.)] == UNKNOWN


@pytest.mark.parametrize("minimum", [np.nan, np.inf, -.2, .8])
def test_invalid_free_interval_start_does_not_erase_valid_contact(minimum):
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.025)
    observation = Observation(Pose(0., 0., 0.), np.array([0.]), np.array([.3]),
        3., 1., min_ranges=np.array([minimum]), hits=np.array([True]), hit_ranges=np.array([.4]))
    nav.integrate(observation)
    assert nav.grid[nav.world_to_cell(.4, 0.)] == OCCUPIED
    assert not np.any(nav.grid == FREE)


@pytest.mark.parametrize("origin", [(np.nan, 0.), (0., np.inf), (), (0.,),
                                   (0., 0., 0.), ((0., 0.),)])
def test_invalid_sensor_origin_pauses_before_map_or_frame_clock_changes(origin):
    nav = clear_nav()
    nav._last_timestamp = .5
    nav._last_pose = nav.pose
    nav._last_command = (.2, 0.)
    grid, evidence = nav.grid.copy(), nav._evidence.copy()
    observation = Observation(nav.pose, np.array([-.2, 0., .2]), np.full(3, 2.),
        3., 1., sensor_origin=origin, hits=np.zeros(3, bool))
    assert nav.update(observation) == (0., 0.)
    assert nav.status == "paused"
    assert nav.reason == "invalid_sensor_origin"
    assert nav._last_timestamp == .5
    np.testing.assert_array_equal(nav.grid, grid)
    np.testing.assert_array_equal(nav._evidence, evidence)


def sparse_observation(valid_count, *, rgb):
    angles = np.linspace(-.6, .6, 81)
    ranges = np.full(81, np.nan)
    near = np.full(81, np.nan)
    selected = np.arange(40, 40 + valid_count)
    ranges[selected] = 1. if rgb else 3.
    near[selected] = .25
    return Observation(Pose(0., 0., 0.), angles, ranges, 3., 1.,
        min_ranges=near if rgb else None,
        hits=np.zeros(81, bool) if rgb else None,
        hit_ranges=np.full(81, np.nan) if rgb else None)


def test_three_sparse_rgb_intervals_only_clear_their_visible_floor():
    nav = Navigator(bounds=(-1., -1., 4., 3.), resolution=.025)
    observation = sparse_observation(3, rgb=True)
    nav.update(observation)
    assert nav.status != "paused"
    assert nav.grid[nav.world_to_cell(.5, 0.)] == FREE
    assert nav.grid[nav.world_to_cell(.1, 0.)] == UNKNOWN  # camera blind strip
    assert nav.grid[nav.world_to_cell(.5, .5)] == UNKNOWN  # unmeasured bearing
    assert nav.grid[nav.world_to_cell(1.2, 0.)] == UNKNOWN  # behind visible interval
    assert not np.any(nav.grid == OCCUPIED)
    assert not nav.traversable()[nav.world_to_cell(.5, .5)]


def test_visually_tracked_zero_floor_intervals_can_turn_on_known_footprint():
    nav = clear_nav()
    nav.goal = (0., 1.)
    nav.path = [(0., 0.), (0., .2), (0., .5), (0., 1.)]
    unknown = nav.world_to_cell(2., 2.)
    nav.grid[unknown] = UNKNOWN
    nav._observed[unknown] = False
    nav._evidence[unknown] = 0
    before_grid, before_evidence = nav.grid.copy(), nav._evidence.copy()
    linear, angular = nav.update(sparse_observation(0, rgb=True))
    assert linear == 0.
    assert angular > 0.
    assert nav.status == "following"
    assert nav._last_timestamp == 1.
    np.testing.assert_array_equal(nav.grid, before_grid)
    np.testing.assert_array_equal(nav._evidence, before_evidence)


def test_zero_floor_intervals_never_authorize_forward_motion_even_on_known_map():
    nav = clear_nav()
    # This path points straight ahead and has fully known historical clearance.
    # Current central floor evidence is still required for forward movement.
    assert nav.update(sparse_observation(0, rgb=True))[0] == 0.
    assert nav.status == "following"


def test_lost_visual_tracking_with_zero_floor_intervals_stops_rotation():
    from dataclasses import replace
    nav = clear_nav()
    nav.goal = (0., 1.)
    nav.path = [(0., 0.), (0., .2), (0., .5), (0., 1.)]
    nav._last_command = (0., .65)
    before = nav.grid.copy()
    assert nav.update(replace(sparse_observation(0, rgb=True), tracking=False)) == (0., 0.)
    assert nav.status == "paused"
    assert nav.reason == "tracking_lost"
    assert nav._last_timestamp is None
    np.testing.assert_array_equal(nav.grid, before)


@pytest.mark.parametrize("valid_count, accepted", [(3, False), (19, False), (20, True)])
def test_ideal_depth_mode_preserves_25_percent_validity_requirement(valid_count, accepted):
    nav = Navigator(bounds=(-1., -1., 4., 3.), resolution=.025)
    nav.update(sparse_observation(valid_count, rgb=False))
    if accepted:
        assert nav.status != "paused"
        assert np.any(nav.grid == FREE)
    else:
        assert nav.reason == "insufficient_depth"
        assert nav.status == "paused"
        assert np.all(nav.grid == UNKNOWN)
