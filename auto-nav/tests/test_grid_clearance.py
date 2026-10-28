"""Clearance is measured to grid squares rather than their centre points."""
import math

import numpy as np

from rover_nav.navigation import FREE, UNKNOWN, Navigator
from rover_nav.navigation import Pose
from scipy.ndimage import distance_transform_edt


def test_rgb_clearance_excludes_unknown_cell_corners():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.05,
                    robot_radius=.25, safety_margin=.08, account_cell_area=True)
    # A diagonal unseen boundary exercises square corners and grid axes.
    for row, col in np.ndindex(nav.shape):
        x, y = nav.cell_to_world((row, col))
        nav.grid[row, col] = FREE if x > -.6 and y > -.6 and x + .7 * y < 1.5 else UNKNOWN
    safe = nav.traversable()
    assert np.any(safe)
    unseen = np.array([nav.cell_to_world(tuple(cell)) for cell in np.argwhere(nav.grid == UNKNOWN)])
    for cell in np.argwhere(safe):
        center = nav.cell_to_world(tuple(cell))
        # Exact point-to-square distance, independently of the planner's EDT.
        delta = np.maximum(np.abs(unseen - center) - nav.resolution / 2, 0.)
        clearance = np.min(np.linalg.norm(delta, axis=1))
        assert clearance >= nav.inflation_radius + .02 - 1e-9


def test_finer_rgb_grid_still_has_a_route_through_the_doorway():
    nav = Navigator(bounds=(-1., -1., 2., 2.), resolution=.05,
                    robot_radius=.25, safety_margin=.08, account_cell_area=True)
    # A 1.08 m physical opening with a 10 cm conservative floor strip on
    # each side still admits a circular rover through guaranteed free floor.
    for row, col in np.ndindex(nav.shape):
        x, y = nav.cell_to_world((row, col))
        if abs(y - .5) < .30 and abs(x - .5) > .44:
            nav.grid[row, col] = UNKNOWN
        else:
            nav.grid[row, col] = FREE
    start, goal = nav.world_to_cell(.5, -.5), nav.world_to_cell(.5, 1.5)
    path = nav.astar(start, goal)
    assert path
    assert math.dist(nav.cell_to_world(path[0]), (.5, -.5)) < .04
    assert math.dist(nav.cell_to_world(path[-1]), (.5, 1.5)) < .04


def test_camera_survey_prefers_floor_visible_from_an_open_viewpoint():
    nav = Navigator(bounds=(-3., -3., 3., 3.), resolution=.05,
                    survey_clearance=.6, account_cell_area=True)
    for cell in np.ndindex(nav.shape):
        x, y = nav.cell_to_world(cell)
        if math.hypot(x, y) < 2.:
            nav.grid[cell] = FREE
    nav.pose = Pose(0., 0., 0.)
    goal = nav.select_frontier()
    assert goal is not None
    clearance = distance_transform_edt(nav.grid == FREE) * nav.resolution
    assert clearance[goal] >= .6
    assert nav.astar(nav.world_to_cell(0., 0.), goal)


def test_open_scan_goal_does_not_require_the_whole_path_to_be_wide():
    nav = Navigator(bounds=(-2., -2., 5., 2.), resolution=.05,
                    robot_radius=.25, safety_margin=.08,
                    survey_clearance=.6, account_cell_area=True)
    for cell in np.ndindex(nav.shape):
        x, y = nav.cell_to_world(cell)
        if (math.hypot(x, y) < 1.05 or math.hypot(x - 3., y) < 1.2
                or .5 < x < 3. and abs(y) < .44):
            nav.grid[cell] = FREE
    nav.pose = Pose(0., 0., 0.)
    nav.stations = [{"x":0., "y":0.}]
    goal = nav.select_frontier()
    assert goal is not None
    assert nav.cell_to_world(goal)[0] > 1.5
    path = nav.astar(nav.world_to_cell(0., 0.), goal)
    assert path
    clearance = distance_transform_edt(nav.grid == FREE) * nav.resolution
    assert clearance[goal] >= .6
    assert min(clearance[cell] for cell in path) < .6
