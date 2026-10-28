"""Geometry, safety and planning tests for the sensor-only controller."""
import math
import unittest

import numpy as np

from .navigation import FREE, OCCUPIED, UNKNOWN, Navigator, Observation, Pose


class NavigationTests(unittest.TestCase):
    def observation(self, timestamp=0., tracking=True, ranges=None, yaw=0.):
        angles = np.linspace(-.6, .6, 61)
        return Observation(Pose(2., 2., yaw), angles,
                           np.full(len(angles), 3.) if ranges is None else ranges,
                           3., timestamp, tracking)

    def test_finite_ray_preserves_occluded_and_unseen_cells(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.integrate(Observation(Pose(1., 3., 0.), np.array([0.]), np.array([1.]), 3., 0.))
        self.assertEqual(navigator.grid[navigator.world_to_cell(1.5, 3.)], FREE)
        self.assertEqual(navigator.grid[navigator.world_to_cell(2., 3.)], OCCUPIED)
        self.assertEqual(navigator.grid[navigator.world_to_cell(2.5, 3.)], UNKNOWN)
        self.assertEqual(navigator.grid[navigator.world_to_cell(1.5, 3.5)], UNKNOWN)

    def test_missing_depth_never_clears_unknown(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.integrate(Observation(Pose(1., 3., 0.), np.array([0., .2, -.2]),
                                         np.array([np.nan, np.inf, 0.]), 3., 0.))
        self.assertTrue(np.all(navigator.grid == UNKNOWN))

    def test_sensor_extrinsic_origin_is_used_for_mapping(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.integrate(Observation(Pose(1., 3., 0.), np.array([0.]), np.array([1.]),
                                         3., 0., sensor_origin=(1.2, 3.)))
        self.assertEqual(navigator.grid[navigator.world_to_cell(2.2, 3.)], OCCUPIED)
        self.assertEqual(navigator.grid[navigator.world_to_cell(1., 3.)], UNKNOWN)

    def test_footprint_inflates_both_unknown_and_obstacles(self):
        navigator = Navigator(bounds=(0, 0, 4, 4), resolution=.1)
        navigator.grid[5:35, 5:35] = FREE
        navigator.grid[20, 20] = OCCUPIED
        safe = navigator.traversable()
        self.assertFalse(safe[6, 10])
        self.assertFalse(safe[20, 23])
        self.assertTrue(safe[12, 12])

    def test_astar_forbids_diagonal_corner_cut(self):
        navigator = Navigator(bounds=(0, 0, .3, .3), resolution=.1, robot_radius=.01, safety_margin=0.)
        safe = np.zeros((3, 3), dtype=bool)
        safe[0, 0] = safe[1, 1] = True
        self.assertEqual(navigator.astar((0, 0), (1, 1), safe), [])
        safe[0, 1] = safe[1, 0] = True
        self.assertEqual(navigator.astar((0, 0), (1, 1), safe), [(0, 0), (1, 1)])

    def test_tracking_loss_and_stale_frames_stop(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.update(self.observation())
        self.assertEqual(navigator.check(.6), (0., 0.))
        self.assertEqual(navigator.reason, "stale_observation")
        navigator.update(self.observation(timestamp=.7))
        self.assertNotEqual(navigator.status, "paused")
        self.assertEqual(navigator.update(self.observation(timestamp=.8, tracking=False)), (0., 0.))
        self.assertEqual(navigator.reason, "tracking_lost")

    def test_invalid_ranges_and_reordered_timestamps_stop(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.update(self.observation(timestamp=1.))
        self.assertEqual(navigator.update(self.observation(timestamp=.5)), (0., 0.))
        self.assertEqual(navigator.reason, "nonmonotonic_timestamp")
        self.assertEqual(navigator.update(self.observation(timestamp=2., ranges=np.full(61, np.nan))), (0., 0.))
        self.assertEqual(navigator.reason, "insufficient_depth")

    def test_duplicate_frames_cannot_renew_motion_or_watchdog(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.update(self.observation(timestamp=1.))
        navigator.update(self.observation(timestamp=1.1))
        self.assertNotEqual(navigator.check(1.1), (0., 0.))
        self.assertEqual(navigator.update(self.observation(timestamp=1.1)), (0., 0.))
        self.assertEqual(navigator.reason, "nonmonotonic_timestamp")
        self.assertEqual(navigator.check(1.2), (0., 0.))
        self.assertEqual(navigator.check(1.7), (0., 0.))
        self.assertEqual(navigator.reason, "stale_observation")

    def test_invalid_and_future_controller_clock_stop(self):
        for now, reason in ((float("nan"), "invalid_controller_clock"),
                            (float("inf"), "invalid_controller_clock"),
                            (.9, "future_observation")):
            navigator = Navigator(bounds=(0, 0, 6, 6))
            navigator.update(self.observation(timestamp=1.))
            self.assertEqual(navigator.check(now), (0., 0.))
            self.assertEqual(navigator.reason, reason)

    def test_pose_discontinuity_stops_before_map_corruption(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.update(self.observation(timestamp=1.))
        old_grid = navigator.grid.copy()
        frame = self.observation(timestamp=1.1)
        jumped = Observation(Pose(4., 2., 0.), frame.angles, frame.ranges, 3., 1.1)
        self.assertEqual(navigator.update(jumped), (0., 0.))
        self.assertEqual(navigator.reason, "pose_discontinuity")
        np.testing.assert_array_equal(navigator.grid, old_grid)

    def test_yaw_discontinuity_stops_before_map_corruption(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.update(self.observation(timestamp=1.))
        old_grid = navigator.grid.copy()
        self.assertEqual(navigator.update(self.observation(timestamp=1.1, yaw=math.pi)), (0., 0.))
        self.assertEqual(navigator.reason, "yaw_discontinuity")
        np.testing.assert_array_equal(navigator.grid, old_grid)

    def test_repeated_obstructed_replans_have_bounded_recovery(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.pose = Pose(2., 2., 0.)
        safe = np.zeros(navigator.shape, dtype=bool)
        for failure in range(4):
            for retry in range(4):
                navigator.status = "following"
                navigator.goal = (3., 2.)
                navigator.path = [(2.05, 2.05), (2.15, 2.05), (3., 2.)]
                self.assertEqual(navigator._follow(self.observation(timestamp=failure + retry * .1), safe), (0., 0.))
            self.assertEqual(navigator._failed_goals, failure + 1)
        self.assertEqual(navigator.status, "stalled")
        self.assertEqual(navigator.reason, "recovery_budget_exhausted")

    def test_stationary_survey_time_is_excluded_from_drive_stall_timeout(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.pose = Pose(2., 2., 0.)
        navigator.grid.fill(FREE)
        navigator._last_progress_xy = (2., 2.)
        navigator._last_progress_time = 1.
        navigator._begin_survey("station", 2.)
        # A physically slow full revolution can take far longer than the
        # translation watchdog. The first command afterward needs a new timer.
        navigator.status = "following"
        navigator.goal = (3., 2.)
        navigator.path = [(2.05, 2.05), (2.35, 2.05), (3., 2.)]
        linear, _ = navigator._follow(self.observation(timestamp=62.), navigator.traversable())
        self.assertGreater(linear, 0.)
        self.assertEqual(navigator.status, "following")
        self.assertEqual(navigator._failed_goals, 0)
        self.assertEqual(navigator._last_progress_time, 62.)

    def test_local_emergency_stop_zeroes_both_command_axes(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.pose = Pose(2., 2., 0.)
        navigator.grid.fill(FREE)
        navigator.status = "following"
        navigator.goal = (3., 2.1)
        navigator.path = [(2.05, 2.05), (2.35, 2.1), (3., 2.1)]
        stopped = navigator._follow(self.observation(timestamp=1., ranges=np.full(61, .2)), navigator.traversable())
        self.assertEqual(stopped, (0., 0.))
        self.assertEqual(navigator.reason, "local_obstacle_stop")

    def test_nearby_forward_obstacle_allows_verified_clear_turn_away(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        navigator.pose = Pose(2., 2., 0.)
        navigator.grid.fill(FREE)
        # The phone is forward of the base, so a short measured range need
        # not intersect the circular footprint during an in-place rotation.
        navigator.grid[18:23, 24] = OCCUPIED
        navigator.status = "following"
        navigator.goal = (2., 3.)
        navigator.path = [(2.05, 2.05), (2.05, 2.35), (2., 3.)]
        frame = self.observation(timestamp=1., ranges=np.full(61, .20))
        linear, angular = navigator._follow(frame, navigator.traversable())
        self.assertEqual(linear, 0.)
        self.assertGreater(angular, 0.)
        self.assertEqual(navigator.status, "following")
        self.assertEqual(navigator._blocked_replans, 0)

    def test_quantized_margin_can_return_to_safe_corridor(self):
        navigator = Navigator(bounds=(0, 0, 3, 3), robot_radius=.25, safety_margin=.10)
        navigator.grid.fill(FREE)
        navigator.grid[5:10, 5:8] = OCCUPIED
        safe = navigator.traversable()
        start = (1.00, 1.00)
        target = (1.15, 1.15)
        self.assertFalse(safe[navigator.world_to_cell(*start)])
        self.assertTrue(navigator._line_safe(start, target, safe))

    def test_frontier_target_is_reachable_and_conservative(self):
        navigator = Navigator(bounds=(0, 0, 8, 8))
        navigator.pose = Pose(2., 2., 0.)
        navigator.grid[5:50, 5:50] = FREE
        navigator.grid[60:75, 60:75] = FREE  # Disconnected island must not be a goal.
        goal = navigator.select_frontier()
        self.assertIsNotNone(goal)
        safe = navigator.traversable()
        self.assertTrue(safe[goal])
        self.assertLess(goal[0], 50)
        self.assertLess(goal[1], 50)
        self.assertTrue(navigator.astar(navigator.world_to_cell(2., 2.), goal, safe))

    def test_full_survey_requires_measured_yaw(self):
        navigator = Navigator(bounds=(0, 0, 6, 6))
        for index in range(100):
            linear, angular = navigator.update(self.observation(timestamp=index * .1))
            self.assertEqual(linear, 0.)
            self.assertLessEqual(abs(angular), .8)
        self.assertEqual(navigator.status, "initial_survey")
        for index in range(1, 100):
            navigator.update(self.observation(timestamp=10 + index * .1, yaw=index * .07))
            if navigator.status == "planning":
                break
        self.assertEqual(navigator.status, "planning")
        self.assertEqual(len(navigator.stations), 1)
        self.assertGreaterEqual(navigator.snapshot()["capture_count"], 12)


if __name__ == "__main__":
    unittest.main()
