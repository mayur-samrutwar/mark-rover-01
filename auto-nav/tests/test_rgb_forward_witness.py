"""Current floor corroboration of historically mapped stopping directions.

No simulator state enters these observations. The saved collision regression
uses the real RGB mask's calibrated floor projection and the historical map.
"""
from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest

from rover_nav.navigation import FREE, UNKNOWN, Navigator, Observation, Pose


FIXTURES = Path(__file__).parent / "fixtures"
OFFSET = np.array([.007710730721508102, .02])


def navigator(pose=Pose(0., 0., 0.)):
    nav = Navigator(bounds=(-2., -2., 4., 4.), resolution=.05,
                    robot_radius=.25, safety_margin=.08, account_cell_area=True)
    nav.grid.fill(FREE)
    nav._observed.fill(True)
    nav._evidence.fill(-12)
    nav.pose = pose
    nav.status, nav.reason = "following", "frontier_goal"
    nav.goal = (pose.x + math.cos(pose.yaw), pose.y + math.sin(pose.yaw))
    nav.path = [(pose.x + distance * math.cos(pose.yaw),
                 pose.y + distance * math.sin(pose.yaw)) for distance in (0., .2, .5, 1.)]
    return nav


def floor_frame(*, near=.375, far=2.5, angles=None, offset=OFFSET,
                pose=Pose(0., 0., 0.), minimum=True):
    angles = np.linspace(-.8, .8, 101) if angles is None else np.asarray(angles, float)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    sensor = np.array([pose.x, pose.y]) + np.array([[c, -s], [s, c]]) @ offset
    return Observation(pose, angles, np.full(len(angles), far), 2.5, 1.,
        sensor_origin=tuple(sensor),
        min_ranges=np.full(len(angles), near) if minimum else None,
        hits=np.zeros(len(angles), bool))


def reason(nav, frame, linear=.25, angular=0.):
    return nav._rgb_swept_stop_reason(frame, linear, angular, nav.traversable())


@pytest.mark.parametrize("speed", [.035, .13, .25])
def test_seed7_uncertain_near_prefix_has_a_real_forward_corroboration_point(speed):
    nav = navigator()
    frame = floor_frame(near=.375, far=2.5)
    assert reason(nav, frame, speed) is None
    # The prefix is still unmeasured: this confirmation must not integrate it.
    unknown_nav = Navigator(bounds=(-1., -1., 3., 2.), resolution=.05)
    unknown_nav.integrate(frame)
    assert unknown_nav.grid[unknown_nav.world_to_cell(.20, .02)] == UNKNOWN


def test_goal_approach_does_not_stall_on_seed7_near_prefix():
    nav = navigator()
    nav.goal = (.30, 0.)
    nav.path = [(0., 0.), (.15, 0.), (.30, 0.)]
    linear, angular = nav._follow(floor_frame(), nav.traversable())
    assert linear == pytest.approx(.13)
    assert angular == pytest.approx(0.)


def test_slow_probe_cannot_extend_beyond_one_grid_cell_diagonal():
    nav = navigator()
    path = nav._rgb_braking_paths(.035, 0.)[0]
    leading = path[-1] + [.25, 0.] - OFFSET
    furthest = np.linalg.norm(leading + [math.sqrt(2.) * nav.resolution, 0.])
    assert .375 < furthest < .40
    assert reason(nav, floor_frame(near=furthest + .001), .035) == "insufficient_current_forward_floor"


@pytest.mark.parametrize("speed", [.035, .13, .25])
def test_far_disjoint_floor_does_not_corroborate_the_stop_direction(speed):
    nav = navigator()
    assert reason(nav, floor_frame(near=2., far=2.5), speed) == "insufficient_current_forward_floor"


@pytest.mark.parametrize("near, far", [(.3, .3), (np.nan, 2.), (.2, np.nan),
                                        (.2, np.inf), (-.1, 2.), (.8, .3)])
def test_no_genuine_nonzero_interval_fails_closed(near, far):
    assert reason(navigator(), floor_frame(near=near, far=far)) == "insufficient_current_forward_floor"


def test_omitted_minimum_uses_zero_but_does_not_bypass_the_gate():
    nav = navigator()
    assert reason(nav, floor_frame(far=2., minimum=False)) is None
    assert reason(nav, floor_frame(far=np.nan, minimum=False)) == "insufficient_current_forward_floor"
    assert reason(nav, floor_frame(far=.20, minimum=False)) == "insufficient_current_forward_floor"


def test_isolated_ray_cannot_authorize_the_surrounding_sector():
    nav = navigator()
    frame = floor_frame(near=.2, far=np.nan)
    frame.ranges[48] = 2.5
    assert reason(nav, frame) == "insufficient_current_forward_floor"


def test_missing_angular_samples_cannot_be_bridged():
    frame = floor_frame(near=.2, angles=[-.3, -.2, .2, .3])
    assert reason(navigator(), frame) == "insufficient_current_forward_floor"


def test_both_adjacent_radial_endpoints_must_contain_the_actual_probe():
    nav = navigator()
    frame = floor_frame(near=.2)
    # Every forward probe lies at a small negative bearing because the camera
    # is mounted 2 cm left of centre. Its lower bracketing beam ends too soon.
    frame.ranges[frame.angles < 0.] = .38
    assert reason(nav, frame) == "insufficient_current_forward_floor"
    frame.ranges[:] = 2.5
    frame.min_ranges[frame.angles < 0.] = .9
    assert reason(nav, frame) == "insufficient_current_forward_floor"


def test_no_angular_extrapolation_outside_current_camera_bearings():
    frame = floor_frame(near=.2, angles=np.linspace(.1, .8, 101))
    assert reason(navigator(), frame) == "insufficient_current_forward_floor"


@pytest.mark.parametrize("yaw", [0., 1.1, -2.2])
def test_sensor_offset_is_transformed_from_world_before_checking_bearing(yaw):
    pose = Pose(.4, -.1, yaw)
    nav = navigator(pose)
    angles = np.linspace(-.05, .05, 101)
    centered = floor_frame(near=.2, pose=pose, angles=angles, offset=np.zeros(2))
    assert reason(nav, centered) is None
    shifted = floor_frame(near=.2, pose=pose, angles=angles, offset=np.array([0., .20]))
    assert reason(nav, shifted) == "insufficient_current_forward_floor"


def test_all_curved_braking_models_require_current_support():
    nav = navigator()
    narrow = floor_frame(near=.2, angles=np.linspace(-.06, .06, 101))
    assert reason(nav, narrow, angular=0.) is None
    assert reason(nav, narrow, angular=.8) == "insufficient_current_forward_floor"
    assert reason(nav, floor_frame(near=.2), angular=.8) is None
    nav._last_command = (.25, .8)
    assert reason(nav, narrow, linear=.035) == "insufficient_current_forward_floor"


def test_slow_candidate_and_zero_command_keep_previous_translation_bound():
    nav = navigator()
    frame = floor_frame(near=.375, far=.38)
    assert reason(nav, frame, linear=.035) is None
    nav._last_command = (.25, 0.)
    assert reason(nav, frame, linear=.035) == "insufficient_current_forward_floor"
    assert reason(nav, frame, linear=0., angular=.3) == "insufficient_current_forward_floor"


def test_rotation_without_previous_translation_needs_no_forward_floor_probe():
    nav = navigator()
    frame = floor_frame(far=np.nan)
    assert reason(nav, frame, linear=0., angular=.3) is None
    nav._last_command = (.25, 0.)
    assert reason(nav, frame, linear=0., angular=.3) == "insufficient_current_forward_floor"


def test_current_floor_does_not_override_unknown_historical_braking_tube():
    nav = navigator()
    nav.grid[:, 50:] = UNKNOWN
    assert reason(nav, floor_frame(near=.2)) == "braking_path_unknown"


def test_saved_seed42_wall_image_stops_despite_its_false_free_historical_map(monkeypatch):
    with np.load(FIXTURES / "rgb_forward_collision_seed42.npz") as saved:
        pose = Pose(*saved["pose"])
        nav = Navigator(bounds=(-3., -3., 13., 11.), resolution=.05,
                        robot_radius=.25, safety_margin=.08, account_cell_area=True)
        nav.grid[:] = saved["grid"]
        nav.pose = pose
        nav.status, nav.reason = "following", "frontier_goal"
        nav.goal, nav.path = tuple(saved["goal"]), saved["path"].tolist()
        nav._last_command = tuple(saved["previous_command"])
        offset = saved["sensor_offset"]
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        sensor = np.array([pose.x, pose.y]) + np.array([[c, -s], [s, c]]) @ offset
        frame = Observation(pose, saved["angles"], saved["ranges"], 2.5, 984.5,
            sensor_origin=tuple(sensor), min_ranges=saved["min_ranges"],
            hits=np.zeros(len(saved["angles"]), bool))
    central = np.abs(frame.angles) < math.radians(28)
    assert np.nanmax(frame.ranges[central]) <= .30 + 1e-12
    assert np.any(np.isfinite(frame.ranges[central]))  # old finite-ray check passed
    with monkeypatch.context() as old_gate:
        old_gate.setattr(nav, "_rgb_forward_floor_witness", lambda *args: True)
        assert reason(nav, frame, *nav._last_command) is None
    assert reason(nav, frame, *nav._last_command) == "insufficient_current_forward_floor"
    assert nav._follow(frame, nav.traversable()) == (0., 0.)
    assert nav.reason == "insufficient_current_forward_floor"
