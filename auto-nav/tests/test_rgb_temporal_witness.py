"""Recent raw RGB can corroborate the same world point, never a grid fiction."""
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from rover_nav.navigation import FREE, UNKNOWN, Navigator, Observation, Pose


OFFSET = np.array([.007710730721508102, .02])


def nav():
    result = Navigator(bounds=(-2., -2., 4., 4.), resolution=.05,
                       robot_radius=.25, safety_margin=.08, account_cell_area=True)
    result.grid.fill(FREE)
    result._observed.fill(True)
    result._evidence.fill(-4)
    result.pose = Pose(0., 0., 0.)
    result.status, result.reason = "following", "frontier_goal"
    result.goal, result.path = (1., 0.), [(0., 0.), (.2, 0.), (.5, 0.), (1., 0.)]
    return result


def image(near=.7, far=1.5, *, timestamp=2., pose=Pose(0., 0., 0.),
          angles=None, offset=OFFSET):
    angles = np.linspace(-.8, .8, 101) if angles is None else np.asarray(angles, float)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    origin = np.array([pose.x, pose.y]) + np.array([[c, -s], [s, c]]) @ offset
    return Observation(pose, angles, np.full(len(angles), far), 2.5, timestamp,
        sensor_origin=tuple(origin), min_ranges=np.full(len(angles), near),
        hits=np.zeros(len(angles), bool),
        free_intervals=tuple(((near, far),) for _ in angles))


def reason(navigator, observation, speed=.25, angular=0.):
    return navigator._rgb_swept_stop_reason(observation, speed, angular, navigator.traversable())


def test_real_recent_intervals_enable_a_current_gap_but_far_floor_alone_does_not():
    navigator = nav()
    current = image()
    assert reason(navigator, current) == "insufficient_current_forward_floor"
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1.))
    assert reason(navigator, current) is None
    assert navigator.snapshot()["temporal_rgb_corroboration_checks"] == 1


@pytest.mark.parametrize("age, permitted", [(5., True), (5.00001, False), (0., False), (-.1, False)])
def test_original_exposure_age_is_bounded_and_must_be_in_the_past(age, permitted):
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=10. - age))
    assert (reason(navigator, image(timestamp=10.)) is None) is permitted


@pytest.mark.parametrize("distance, permitted", [(.4, True), (.40001, False)])
def test_relative_translation_has_a_hard_bound(distance, permitted):
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1.))
    current = image(timestamp=2., pose=Pose(distance, 0., 0.))
    assert (reason(navigator, current) is None) is permitted


def test_current_wall_without_far_floor_stops_even_with_recent_clear_image():
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1.))
    assert reason(navigator, image(.225, .3)) == "insufficient_current_forward_floor"
    empty = replace(image(), free_intervals=((),) * 101)
    assert reason(navigator, empty) == "insufficient_current_forward_floor"


def test_current_positive_contact_still_vetoes_recent_floor():
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1.))
    current = replace(image(), hits=np.ones(101, bool), hit_ranges=np.full(101, .35))
    assert reason(navigator, current) == "local_obstacle_stop"
    assert navigator.snapshot()["temporal_rgb_corroboration_checks"] == 0


def test_recent_floor_cannot_override_the_full_historical_braking_tube():
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1.))
    navigator.grid[:, 50:] = UNKNOWN
    assert reason(navigator, image()) == "braking_path_unknown"


def test_current_and_old_support_at_different_probe_parameters_cannot_be_combined():
    navigator = nav()
    old = image(.26, .295, timestamp=1.8, pose=Pose(0., -.02, 0.),
                angles=[-.01, 0., .01], offset=np.array([0., .02]))
    current = image(.6, 1., angles=[-.07, -.06, -.05], offset=np.array([0., .02]))
    current = replace(current, free_intervals=((), ((.6, 1.),), ((.6, 1.),)))
    navigator._remember_rgb_observation(old)
    path = navigator._rgb_braking_paths(.035, 0.)[0]
    leading = path[-5] + [.25, 0.] - [0., .02]
    band = .06 + math.sqrt(2.) * navigator.resolution
    current_t = [value for sector in navigator._rgb_floor_sectors(current)
                 for value in navigator._rgb_probe_intervals(leading, np.array([1., 0.]), band, sector, beyond_near=True)]
    old_t = [value for sector in navigator._rgb_floor_sectors(old)
             for value in navigator._rgb_probe_intervals(leading + [0., .02], np.array([1., 0.]), band, sector)]
    assert current_t and old_t
    assert min(a for a, b in current_t) > max(b for a, b in old_t)
    assert reason(navigator, current, .035) == "insufficient_current_forward_floor"


def test_original_sensor_origin_and_yaw_define_the_same_world_point():
    navigator = nav()
    current_pose = Pose(.1, .05, .2)
    old_pose = Pose(0., 0., -.4)
    old_offset = np.array([-.04, .12])
    path = navigator._rgb_braking_paths(.25, 0.)[0]
    point = path[-5] + [.25 + .04, 0.]
    rotation = lambda yaw: np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    world = rotation(current_pose.yaw) @ point + [current_pose.x, current_pose.y]
    origin = rotation(old_pose.yaw) @ old_offset + [old_pose.x, old_pose.y]
    relative = rotation(old_pose.yaw).T @ (world - origin)
    radius, bearing = np.linalg.norm(relative), math.atan2(relative[1], relative[0])
    old = image(radius - .004, radius + .004, timestamp=1., pose=old_pose,
                angles=np.linspace(bearing - .015, bearing + .015, 101), offset=old_offset)
    current = image(pose=current_pose)
    navigator._remember_rgb_observation(old)
    assert reason(navigator, current) is None
    wrong = nav()
    wrong._remember_rgb_observation(replace(old, sensor_origin=(old_pose.x, old_pose.y)))
    assert reason(wrong, current) == "insufficient_current_forward_floor"


def test_disjoint_old_unions_never_become_a_radial_hull():
    navigator = nav()
    old = image(timestamp=1.)
    old = replace(old, free_intervals=tuple(((.2, .3), (.7, 1.5)) for _ in old.angles))
    navigator._remember_rgb_observation(old)
    assert reason(navigator, image()) == "insufficient_current_forward_floor"


def test_history_keeps_original_angular_holes():
    navigator = nav()
    old = image(.2, 1.5, timestamp=1., angles=[-.3, -.2, .2, .3])
    navigator._remember_rgb_observation(old)
    assert reason(navigator, image()) == "insufficient_current_forward_floor"


def test_history_owns_readonly_arrays_and_scalar_geometry():
    navigator = nav()
    source = image(.2, 1.5, timestamp=1.)
    x, y, yaw, ox, oy = (np.array(v) for v in (0., 0., 0., OFFSET[0], OFFSET[1]))
    source = replace(source, pose=Pose(x, y, yaw), sensor_origin=(ox, oy))
    navigator._remember_rgb_observation(source)
    stored = navigator._rgb_recent_observations[0]
    for array in (source.angles, source.ranges, source.min_ranges):
        array[:] = np.nan
    source.hits[:] = True
    for scalar in (x, y, yaw, ox, oy):
        scalar.fill(99.)
    assert stored.pose == Pose(0., 0., 0.)
    assert stored.sensor_origin == tuple(OFFSET)
    for array in (stored.angles, stored.ranges, stored.min_ranges, stored.hits):
        assert not array.flags.writeable
    assert reason(navigator, image()) is None


@pytest.mark.parametrize("fault", ["tracking", "stale_check", "translation_jump", "yaw_jump"])
def test_tracking_continuity_faults_clear_history(fault):
    navigator = nav()
    navigator.update(image(.2, 1.5, timestamp=1.))
    assert navigator._rgb_recent_observations
    if fault == "stale_check":
        navigator.check(1.50001)
    elif fault == "tracking":
        navigator.update(replace(image(timestamp=1.1), tracking=False))
    else:
        pose = Pose(.6, 0., 0.) if fault == "translation_jump" else Pose(0., 0., 1.)
        navigator.update(image(timestamp=1.1, pose=pose))
    assert not navigator._rgb_recent_observations


def test_accepted_frame_gap_clears_history_even_without_independent_watchdog():
    navigator = nav()
    navigator.update(image(.2, 1.5, timestamp=1.))
    assert navigator.update(image(timestamp=1.50001)) == (0., 0.)
    assert navigator.reason == "insufficient_current_forward_floor"
    assert all(past.timestamp > 1. for past in navigator._rgb_recent_observations)


def test_legacy_sensing_switch_clears_rgb_history():
    navigator = nav()
    navigator.update(image(.2, 1.5, timestamp=1.))
    navigator.update(replace(image(.2, 2.5, timestamp=1.1), hits=None, free_intervals=None))
    assert not navigator._rgb_recent_observations


def test_history_is_bounded_and_never_retimes_recent_support():
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1.))
    for timestamp in np.arange(1.1, 6.2, .1):
        current = image(timestamp=float(timestamp))
        reason(navigator, current)
        navigator._remember_rgb_observation(current)
    assert reason(navigator, image(timestamp=6.2)) == "insufficient_current_forward_floor"
    assert all(past.min_ranges[0] == .7 for past in navigator._rgb_recent_observations)
    for timestamp in np.arange(6.21, 7.21, .01):
        navigator._remember_rgb_observation(image(timestamp=float(timestamp)))
    assert len(navigator._rgb_recent_observations) == 64


def test_current_adjacent_rays_require_a_common_real_far_floor_patch():
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1.))
    current = image(angles=[-.1, 0., .1])
    current = replace(current, free_intervals=(((.6, .8),), ((.9, 1.2),), ((.6, .8),)))
    assert reason(navigator, current) == "insufficient_current_forward_floor"


def test_different_old_observations_cannot_supply_opposite_radial_endpoints():
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, .3, timestamp=1.))
    navigator._remember_rgb_observation(image(.7, 1.5, timestamp=1.2))
    assert reason(navigator, image()) == "insufficient_current_forward_floor"


def test_annulus_clip_preserves_both_components_and_their_gap():
    angle = .8
    sector = (np.array([math.cos(angle), -math.sin(angle)]),
              np.array([math.cos(angle), math.sin(angle)]), .12, .2)
    intervals = Navigator._rgb_probe_intervals(np.array([.1, -.1]), np.array([0., 1.]), .2, sector)
    assert len(intervals) == 2
    assert intervals[0][0] == pytest.approx(0.)
    assert intervals[1][1] == pytest.approx(.2)
    assert intervals[0][1] < .1 < intervals[1][0]


def test_counter_requires_all_braking_models_to_pass():
    navigator = nav()
    navigator._remember_rgb_observation(image(.2, 1.5, timestamp=1., angles=np.linspace(-.08, .08, 101)))
    assert reason(navigator, image(), angular=.8) == "insufficient_current_forward_floor"
    assert navigator.snapshot()["temporal_rgb_corroboration_checks"] == 0


def test_saved_actual_doorway_has_recent_same_world_point_support():
    saved = json.loads((Path(__file__).parent / "fixtures" / "rgb_temporal_doorway_observations.json").read_text())

    def decode(raw):
        values = dict(raw)
        values["pose"] = Pose(**values["pose"])
        for key in ("angles", "ranges", "min_ranges", "hit_ranges"):
            values[key] = np.array([np.nan if value is None else value for value in values[key]], float)
        values["hits"] = np.array(values["hits"], bool)
        values["sensor_origin"] = tuple(values["sensor_origin"])
        values["free_intervals"] = tuple(tuple(tuple(pair) for pair in union) for union in values["free_intervals"])
        return Observation(**{key: value for key, value in values.items() if key in Observation.__dataclass_fields__})

    current = decode(saved["current"])
    history = [decode(raw) for raw in saved["history"]]
    navigator = Navigator(bounds=(-3., -3., 13., 11.), resolution=.05,
                          robot_radius=.25, safety_margin=.08, account_cell_area=True)
    navigator.grid.fill(FREE)
    assert reason(navigator, current, .035) == "insufficient_current_forward_floor"
    for observation in history:
        navigator._remember_rgb_observation(observation)
    assert reason(navigator, current, .035) is None
    assert navigator.snapshot()["temporal_rgb_corroboration_checks"] == 1
    assert current.timestamp - history[0].timestamp == pytest.approx(3.7)
    assert math.hypot(current.pose.x - history[0].pose.x, current.pose.y - history[0].pose.y) < .02
