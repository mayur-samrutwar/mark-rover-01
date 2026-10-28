"""Independent checks of the authored rover, sensor visibility and stop paths.

Only IsaacProtocol's in-process MuJoCo simulation is used. The two tiny walls
in ``occlusion_world`` are a deliberately obstructed test fixture; they are never
used for mission acceptance or substituted for the furnished three-room world.
"""
from dataclasses import replace
import math

import mujoco
import numpy as np
import pytest

from rover_nav.isaac import IsaacRover
from rover_nav.navigation import Navigator, UNKNOWN, wrap_angle
from rover_nav.world import generate_world


@pytest.fixture(scope="module")
def exact_rover():
    rover = IsaacRover(generate_world(7))
    yield rover
    rover.close()


@pytest.fixture(scope="module")
def occlusion_rover():
    world = {"kind": "isaac.world", "version": 1,
             "name": "Test fixture: intentional wall impact and occlusion",
             "units": "m", "upAxis": "Z", "ground_z": 0.,
             "boxes": [
                 {"center": [1.8, 1., .6], "size": [.16, 1.4, 1.2], "color": [.6, .4, .2, 1.]},
                 {"center": [2.8, 1., .8], "size": [.16, 1.6, 1.6], "color": [.2, .4, .6, 1.]},
             ],
             "source": {"spawn": [1., 1., 0.]}}
    rover = IsaacRover(world)
    yield rover
    rover.close()


def reset_rover(rover):
    """Reset dynamic state only; keep original mass, drives and contacts."""
    mujoco.mj_resetDataKeyframe(rover.model, rover.data, 0)
    rover.sim["_desired_ctrl"][:] = 0.
    rover.collision_count = 0
    rover.max_collision_force = 0.
    rover.distance = 0.
    rover.drive(0., 0.)
    rover.settle()
    rover.command_time = float(rover.data.time)
    return rover


@pytest.fixture
def rover(exact_rover):
    return reset_rover(exact_rover)


@pytest.fixture
def obstacle_rover(occlusion_rover):
    return reset_rover(occlusion_rover)


def run_command(rover, linear, angular, duration):
    for _ in range(round(duration / .1)):
        rover.drive(linear, angular)
        rover.advance(.1)


def moving_navigator(rover):
    navigator = Navigator(bounds=(-.25, -.25, 10.25, 8.25))
    navigator.update(rover.observe())
    rover.advance(.1)
    command = navigator.update(rover.observe())
    assert command != (0., 0.), "The test must first establish a live motion command"
    rover.drive(*command)
    return navigator


def test_original_rover_drives_forward_with_closed_loop_actuators(rover):
    before = rover.pose()
    run_command(rover, .18, 0., 1.0)
    after = rover.pose()
    assert .12 < after.x - before.x < .21
    assert abs(after.y - before.y) < .025
    assert abs(wrap_angle(after.yaw - before.yaw)) < .04
    assert rover.model.nu == 4 and rover.model.ncam == 1
    assert np.isfinite(rover.data.qvel).all()
    assert max(abs(rover.data.ctrl)) <= 8.
    # Ground support and normal wheel-ground contact are not collisions.
    assert rover.collision_count == 0


def test_both_turn_commands_have_correct_measured_yaw_direction(rover):
    start = rover.pose()
    run_command(rover, 0., .7, 1.2)
    positive = rover.pose()
    assert wrap_angle(positive.yaw - start.yaw) > .15
    assert math.hypot(positive.x - start.x, positive.y - start.y) < .06
    reset_rover(rover)
    start = rover.pose()
    run_command(rover, 0., -.7, 1.2)
    negative = rover.pose()
    assert wrap_angle(negative.yaw - start.yaw) < -.15
    assert math.hypot(negative.x - start.x, negative.y - start.y) < .06
    assert rover.collision_count == 0


def test_phone_sensor_is_mounted_forward_and_excludes_rover_self_hits(rover):
    observation = rover.observe()
    camera = mujoco.mj_id2name(rover.model, mujoco.mjtObj.mjOBJ_CAMERA, 0)
    assert camera == "cam_mark_phone_rear"
    assert rover.model.cam_bodyid[0] != 0
    assert .15 < rover.data.cam_xpos[0, 2] < .4
    camera_forward = rover.data.cam_xmat[0].reshape(3, 3) @ [0., 0., -1.]
    base_forward = rover.data.xmat[rover.root].reshape(3, 3) @ [0., 0., -1.]
    assert np.dot(camera_forward, base_forward) > .999
    assert len(observation.ranges) == len(observation.angles) == 81
    assert np.max(np.abs(observation.angles)) < math.radians(39)
    assert np.all(np.isfinite(observation.ranges))
    assert np.all((observation.ranges > .1) & (observation.ranges <= observation.max_range))
    hit_ids = rover.ray_ids[rover.ray_dist >= 0]
    assert hit_ids.size > 0
    assert np.all(rover.model.geom_bodyid[hit_ids] == 0), "A depth ray hit the robot itself"
    assert np.any(observation.ranges < observation.max_range)


def test_front_wall_occludes_rear_geometry_and_mapping_shadow(obstacle_rover):
    observation = obstacle_rover.observe()
    central = np.abs(observation.angles) < .1
    assert np.any(central)
    front_id = mujoco.mj_name2id(obstacle_rover.model, mujoco.mjtObj.mjOBJ_GEOM, "nav_world_0")
    rear_id = mujoco.mj_name2id(obstacle_rover.model, mujoco.mjtObj.mjOBJ_GEOM, "nav_world_1")
    assert front_id >= 0 and rear_id >= 0
    hit_rows = obstacle_rover.ray_ids.reshape(-1, obstacle_rover.ray_rows)[central]
    assert np.any(hit_rows == front_id)
    assert not np.any(hit_rows == rear_id), "A central ray saw through a solid front wall"
    expected = 1.8 - .16 / 2 - observation.sensor_origin[0]
    np.testing.assert_allclose(observation.ranges[central], expected, atol=.015)
    navigator = Navigator(bounds=(0., 0., 4., 3.))
    navigator.integrate(observation)
    assert navigator.grid[navigator.world_to_cell(2.5, 1.)] == UNKNOWN
    assert navigator.grid[navigator.world_to_cell(.5, 1.)] == UNKNOWN


def test_command_age_watchdog_removes_drive_after_350_ms(rover):
    rover.drive(.2, 0.)
    issued = rover.command_time
    rover.advance(.30)
    assert np.max(abs(rover.data.ctrl)) > 1.
    rover.advance(.08)
    assert rover.data.time - issued > .35
    np.testing.assert_array_equal(rover.data.ctrl, np.zeros(4))
    stopped_at = rover.pose()
    rover.advance(.7)
    stopped = rover.pose()
    assert math.hypot(stopped.x - stopped_at.x, stopped.y - stopped_at.y) < .025
    assert np.max(abs(rover.data.qvel[-4:])) < .15


def test_tracking_loss_zeroes_navigation_and_wheel_commands(rover):
    navigator = moving_navigator(rover)
    old_grid = navigator.grid.copy()
    command = navigator.update(rover.observe(tracking=False))
    rover.drive(*command)
    assert command == (0., 0.)
    assert navigator.status == "paused" and navigator.reason == "tracking_lost"
    np.testing.assert_array_equal(rover.data.ctrl, np.zeros(4))
    np.testing.assert_array_equal(navigator.grid, old_grid)


def test_independent_sensor_age_check_stops_without_another_frame(rover):
    navigator = moving_navigator(rover)
    rover.advance(.6)  # Deliberately deliver no additional observation.
    command = navigator.check(float(rover.data.time))
    rover.drive(*command)
    assert command == (0., 0.)
    assert navigator.reason == "stale_observation"
    np.testing.assert_array_equal(rover.data.ctrl, np.zeros(4))


@pytest.mark.parametrize("fault", ["backward", "duplicate", "nan", "infinity"])
def test_timestamp_fault_cannot_refresh_command_or_change_map(rover, fault):
    navigator = moving_navigator(rover)
    observation = rover.observe()
    value = {"backward": observation.timestamp - .1,
             "duplicate": observation.timestamp,
             "nan": math.nan, "infinity": math.inf}[fault]
    old_grid = navigator.grid.copy()
    command = navigator.update(replace(observation, timestamp=value))
    rover.drive(*command)
    assert command == (0., 0.), f"Unsafe command after {fault} sensor timestamp"
    assert navigator.status == "paused"
    np.testing.assert_array_equal(navigator.grid, old_grid)
    np.testing.assert_array_equal(rover.data.ctrl, np.zeros(4))


def test_collision_evaluator_detects_deliberate_wall_impact(obstacle_rover):
    assert obstacle_rover.collision_count == 0
    for _ in range(30):
        obstacle_rover.drive(.24, 0.)
        obstacle_rover.advance(.1)
        if obstacle_rover.collision_count:
            break
    obstacle_rover.drive(0., 0.)
    assert obstacle_rover.collision_count > 0, "Intentional impact escaped collision evaluation"
    assert obstacle_rover.max_collision_force > .05
    assert obstacle_rover.pose().x < 1.8


@pytest.mark.parametrize("heading", [math.pi - .025, -math.pi + .025])
def test_forward_sensor_angles_stay_local_across_world_yaw_wrap(rover, heading):
    # Rotate only the test fixture's free base. Neither sensor extrinsics nor
    # contact/motor parameters change; the phone must retain both front halves.
    delta = heading - rover.pose().yaw
    rotation = np.array([math.cos(delta / 2), 0., 0., math.sin(delta / 2)])
    quaternion = np.zeros(4)
    mujoco.mju_mulQuat(quaternion, rotation, rover.data.qpos[3:7].copy())
    rover.data.qpos[3:7] = quaternion
    rover.data.qvel[:] = 0.
    mujoco.mj_forward(rover.model, rover.data)
    observation = rover.observe()
    assert np.max(np.abs(observation.angles)) < math.radians(39)
    left = np.count_nonzero((observation.angles > 0) & (observation.angles < math.radians(28)))
    right = np.count_nonzero((observation.angles < 0) & (observation.angles > -math.radians(28)))
    assert min(left, right) > 20
    assert abs(left - right) <= 1
