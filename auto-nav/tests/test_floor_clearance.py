"""Prove bounded EDT filtering exactly preserves nearest-point floor evidence."""
from dataclasses import fields
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.spatial import cKDTree

from rover_nav.floor_vision import FloorVision


FIXTURES = Path(__file__).parent / "fixtures"
CALIBRATION = json.loads((FIXTURES / "floor_rgb_calibration.json").read_text())


def sensor(*, exact=False):
    result = FloorVision(CALIBRATION["K"], CALIBRATION["T_base_camera"], 320, 240)
    if exact:
        result._clearance_exclusions = exact_exclusions
    return result


def exact_exclusions(unknown, points, required):
    if not len(points):
        return np.empty(0, bool)
    if not len(unknown):
        return np.zeros(len(points), bool)
    distance, _ = cKDTree(unknown).query(points, workers=1)
    return distance < required


def read(name):
    return cv2.cvtColor(cv2.imread(str(FIXTURES / name)), cv2.COLOR_BGR2RGB)


def assert_same_result(left, right):
    for field in fields(left):
        first, second = getattr(left, field.name), getattr(right, field.name)
        if isinstance(first, np.ndarray):
            np.testing.assert_array_equal(first, second, err_msg=field.name, strict=True)
        else:
            assert first == second, field.name


@pytest.mark.parametrize("fixture", sorted(path.name for path in FIXTURES.glob("*.png")))
def test_every_rgb_fixture_matches_full_exact_kd_output_bit_for_bit(fixture):
    optimized, reference = sensor(), sensor(exact=True)
    start = read("floor_rgb_bright_start.png")
    assert_same_result(optimized.process(start), reference.process(start))
    rgb = read(fixture)
    assert_same_result(optimized.process(rgb), reference.process(rgb))


def test_random_image_estimated_transforms_match_full_exact_kd_output():
    rng = np.random.default_rng(761284)
    start = read("floor_rgb_bright_start.png")
    images = sorted(path.name for path in FIXTURES.glob("*.png"))
    for index in range(16):
        optimized, reference = sensor(), sensor(exact=True)
        optimized.process(start)
        reference.process(start)
        transform = np.asarray(CALIBRATION["T_base_camera"]).copy()
        roll, pitch = rng.uniform(-math.radians(4), math.radians(4), 2)
        rx = np.array([[1., 0., 0.], [0., math.cos(roll), -math.sin(roll)],
                       [0., math.sin(roll), math.cos(roll)]])
        ry = np.array([[math.cos(pitch), 0., math.sin(pitch)], [0., 1., 0.],
                       [-math.sin(pitch), 0., math.cos(pitch)]])
        transform[:3, :3] = rx @ ry @ transform[:3, :3]
        transform[:2, 3] += rng.uniform(-.008, .008, 2)
        transform[2, 3] *= rng.uniform(.9, 1.1)
        optimized.set_camera_transform(transform)
        reference.set_camera_transform(transform)
        rgb = read(images[index % len(images)])
        assert_same_result(optimized.process(rgb), reference.process(rgb))


def test_random_clouds_and_exact_threshold_equalities_match_full_kd():
    rng = np.random.default_rng(54037)
    instance = sensor()
    for _ in range(16):
        unknown = instance.origin + rng.uniform(-2., 2., (250, 2))
        points = instance.origin + rng.uniform(-2., 2., (2400, 2))
        distance, _ = cKDTree(unknown).query(points)
        # Half the thresholds are strict/equal floating-point neighbours of the
        # actual point distance; these must take the exact ambiguous route.
        required = rng.uniform(0., .5, len(points))
        required[:400] = distance[:400]
        required[400:800] = np.nextafter(distance[400:800], np.inf)
        required[800:1200] = np.nextafter(distance[800:1200], -np.inf)
        np.testing.assert_array_equal(instance._clearance_exclusions(unknown, points, required),
                                      exact_exclusions(unknown, points, required))


def test_grid_edges_empty_clouds_and_outside_grid_fallback_remain_exact():
    instance = sensor()
    resolution = instance._texture_resolution
    base = instance.origin - (instance.max_range + .15)
    # Points approach opposite corners of containing cells, exercising the
    # full two-half-diagonals error bound, including nextafter cell boundaries.
    cell = base + np.array([100., 120.]) * resolution
    unknown = np.array([np.nextafter(cell, np.inf),
                        np.nextafter(cell + [resolution, 0.], -np.inf)])
    points = np.array([np.nextafter(cell + resolution, -np.inf), cell,
                       cell + [2 * resolution, resolution], [100., -100.]])
    for group in (points[:3], points):
        distance, _ = cKDTree(unknown).query(group)
        for required in (distance, np.nextafter(distance, np.inf), np.zeros(len(group))):
            np.testing.assert_array_equal(instance._clearance_exclusions(unknown, group, required),
                                          exact_exclusions(unknown, group, required))
    assert not instance._clearance_exclusions(np.empty((0, 2)), points, np.ones(len(points))).any()
    assert instance._clearance_exclusions(unknown, np.empty((0, 2)), np.empty(0)).shape == (0,)


def test_exact_queries_are_limited_to_the_ambiguous_band(monkeypatch):
    import rover_nav.floor_vision as module
    queried = []

    class TracedTree:
        def __init__(self, points):
            self.tree = cKDTree(points)

        def query(self, points, *, workers):
            queried.append(len(points))
            return self.tree.query(points, workers=workers)

    monkeypatch.setattr(module, "cKDTree", TracedTree)
    instance = sensor()
    rng = np.random.default_rng(861)
    unknown = instance.origin + np.column_stack((np.zeros(1000), np.linspace(-2., 2., 1000)))
    points = instance.origin + rng.uniform(-1.8, 1.8, (20000, 2))
    required = np.full(len(points), .25)
    result = instance._clearance_exclusions(unknown, points, required)
    np.testing.assert_array_equal(result, exact_exclusions(unknown, points, required))
    assert sum(queried) < .10 * len(points)
