"""Regression guards for the image-only perception/transport boundary.

The fixture contains one ordinary RGB exposure and fixed mount calibration.
No renderer, scene file, pose label or depth buffer is needed to run these
tests. Simulator truth access is actively poisoned at the adapter boundary.
"""
import ast
from dataclasses import fields, replace
import importlib
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from rover_nav.floor_vision import FloorVision
from rover_nav.navigation import FREE, OCCUPIED, UNKNOWN, Navigator, Observation, Pose
from rover_nav.vision import CameraPerception, RGBPacket


FIXTURES = Path(__file__).parent / "fixtures"


@pytest.fixture
def fixture_frame():
    calibration = json.loads((FIXTURES / "floor_rgb_calibration.json").read_text())
    pixels = cv2.cvtColor(cv2.imread(str(FIXTURES / "floor_rgb_initial.png")), cv2.COLOR_BGR2RGB)
    return pixels, calibration


class ForbiddenTruth:
    def __getattribute__(self, name):
        raise AssertionError(f"Perception attempted to read external truth: {name}")


def forbidden_measurement(*args, **kwargs):
    raise AssertionError("Perception attempted a simulator pose, range or render read")


class PacketProvider:
    """An external transport envelope with deliberately inaccessible metadata."""
    def __init__(self, pixels, poisoned):
        self.pixels = pixels
        self.reads = 0
        self.metadata = ForbiddenTruth() if poisoned else None

    def read(self):
        packet = RGBPacket(self.pixels.copy(), self.reads * .1)
        self.reads += 1
        # Truth belongs to the renderer's outer envelope. Only the explicit
        # RGBPacket crosses the transport interface into CameraPerception.
        envelope = SimpleNamespace(packet=packet, metadata=self.metadata)
        return envelope.packet


def make_stub_rover(pixels, calibration, poisoned):
    from rover_nav.vision_isaac import VisionRover
    rover = VisionRover.__new__(VisionRover)
    rover.rgb = PacketProvider(pixels, poisoned)
    rover.perception = CameraPerception(calibration)
    rover.pose = forbidden_measurement
    rover.data = ForbiddenTruth()
    rover.model = ForbiddenTruth()
    rover.sim = ForbiddenTruth()
    rover.world = ForbiddenTruth()
    return rover


def assert_same_observation(left, right):
    assert left.pose == right.pose
    assert left.timestamp == right.timestamp
    assert left.tracking == right.tracking
    assert left.sensor_origin == right.sensor_origin
    assert left.max_range == right.max_range
    assert left.free_intervals == right.free_intervals
    for name in ("angles", "ranges", "min_ranges", "hits", "hit_ranges"):
        np.testing.assert_array_equal(getattr(left, name), getattr(right, name))


def test_rgb_packet_contract_excludes_pose_depth_and_other_measurements():
    assert [field.name for field in fields(RGBPacket)] == ["pixels", "timestamp"]
    assert list(inspect.signature(CameraPerception.process).parameters) == ["self", "packet"]
    with pytest.raises(TypeError):
        RGBPacket(np.zeros((1, 1, 3), np.uint8), 0., depth=np.ones((1, 1)))


def test_real_adapter_observes_rgb_when_all_simulator_measurements_raise(monkeypatch, fixture_frame):
    import mujoco
    pixels, calibration = fixture_frame
    baseline = make_stub_rover(pixels, calibration, False)
    poisoned = make_stub_rover(pixels, calibration, True)
    # These are the privileged calls used by the previous ideal sensor. Their
    # availability must have no effect when RGB is supplied by the transport.
    for name in ("mj_multiRay", "mj_ray", "mj_forward", "Renderer"):
        monkeypatch.setattr(mujoco, name, forbidden_measurement)
    for _ in range(4):
        expected = baseline.observe()
        observed = poisoned.observe()
        assert observed.tracking
        assert observed.free_intervals is not None
        assert observed.free_intervals == poisoned.perception.last_floor.free_intervals
        assert_same_observation(observed, expected)
    assert poisoned.rgb.reads == 4
    assert observed.pose.x == pytest.approx(0., abs=.002)
    assert observed.pose.y == pytest.approx(0., abs=.002)


def test_fault_injection_can_only_remove_tracking(monkeypatch, fixture_frame):
    pixels, calibration = fixture_frame
    normal = make_stub_rover(pixels, calibration, True)
    faulted = make_stub_rover(pixels, calibration, True)
    expected = normal.observe()
    observed = faulted.observe(tracking=False)
    assert not observed.tracking
    assert_same_observation(replace(observed, tracking=expected.tracking), expected)


def test_pure_vision_import_closure_has_no_simulator_or_scene_dependency():
    prohibited = {"mujoco", "simulation", "isaac", "isaacprotocol", "world", "vision_isaac"}
    pending = ["vision"]
    visited = set()
    while pending:
        name = pending.pop()
        if name in visited:
            continue
        visited.add(name)
        module = importlib.import_module(f"rover_nav.{name}")
        tree = ast.parse(Path(module.__file__).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                imported = [node.module or ""]
                if node.level and node.module:
                    pending.append(node.module.split(".")[0])
            else:
                continue
            for imported_name in imported:
                assert not (set(imported_name.split(".")) & prohibited), (name, imported_name)
    assert {"vision", "visual_odometry", "floor_vision", "navigation"} <= visited


def test_actual_floor_intervals_leave_blind_strip_and_occluded_shadow_unknown(fixture_frame):
    pixels, calibration = fixture_frame
    perception = CameraPerception(calibration)
    observation = perception.process(RGBPacket(pixels, 0.))
    assert observation.tracking
    valid = np.isfinite(observation.ranges) & np.isfinite(observation.min_ranges)
    assert valid.sum() > 30
    assert np.all(observation.min_ranges[valid] > .05)
    assert np.all(observation.ranges[valid] >= observation.min_ranges[valid])
    assert np.all(observation.ranges[valid] <= observation.max_range)

    navigator = Navigator(bounds=(-1., -3., 4., 3.), resolution=.025)
    navigator.integrate(observation)
    assert navigator.grid[navigator.world_to_cell(0., 0.)] == UNKNOWN
    assert navigator.grid[navigator.world_to_cell(.10, .02)] == UNKNOWN
    assert navigator.grid[navigator.world_to_cell(.50, .02)] == FREE

    # Isolate a positive furniture boundary from the RGB fixture so adjacent
    # rays cannot mask a mistaken clearing operation behind the endpoint.
    last_free = np.array([segments[-1][1] if segments else np.nan
                         for segments in observation.free_intervals])
    selected = np.flatnonzero(valid & observation.hits & (last_free < 2.)
                             & (observation.ranges > observation.min_ranges + .20)
                             & (observation.hit_ranges > last_free + .10))
    assert len(selected)
    index = selected[np.argmin(np.abs(observation.angles[selected]))]
    single = replace(observation, angles=observation.angles[[index]],
        ranges=observation.ranges[[index]], min_ranges=observation.min_ranges[[index]],
        hits=observation.hits[[index]], hit_ranges=observation.hit_ranges[[index]],
        free_intervals=(observation.free_intervals[index],))
    navigator = Navigator(bounds=(-1., -3., 4., 3.), resolution=.025)
    navigator.integrate(single)
    angle = float(single.angles[0])
    free_distance = single.free_intervals[0][-1][1]
    hit_distance = float(single.hit_ranges[0])
    origin = np.asarray(single.sensor_origin)
    direction = np.array([math.cos(angle), math.sin(angle)])
    free_endpoint = origin + direction * free_distance
    gap = origin + direction * ((free_distance + hit_distance) / 2)
    endpoint, shadow = origin + direction * hit_distance, origin + direction * (hit_distance + .15)
    assert navigator.grid[navigator.world_to_cell(*free_endpoint)] == FREE
    assert navigator.grid[navigator.world_to_cell(*gap)] == UNKNOWN
    assert navigator.grid[navigator.world_to_cell(*endpoint)] == OCCUPIED
    assert navigator.grid[navigator.world_to_cell(*shadow)] == UNKNOWN
    assert navigator.grid[navigator.world_to_cell(*(origin + direction * .10))] == UNKNOWN


def test_texture_uncertainty_never_becomes_an_occupied_endpoint(fixture_frame):
    pixels, calibration = fixture_frame
    observation = CameraPerception(calibration).process(RGBPacket(pixels, 0.))
    finite = np.isfinite(observation.ranges) & np.isfinite(observation.min_ranges)
    candidates = np.flatnonzero(finite & ~observation.hits
        & (observation.ranges < observation.max_range - .05)
        & (observation.ranges > observation.min_ranges + .04))
    assert len(candidates), "Fixture must include finite image-edge/uncertainty intervals"
    index = int(candidates[0])
    single = replace(observation, angles=observation.angles[[index]],
        ranges=observation.ranges[[index]], min_ranges=observation.min_ranges[[index]],
        hits=observation.hits[[index]], hit_ranges=observation.hit_ranges[[index]],
        free_intervals=(observation.free_intervals[index],))
    navigator = Navigator(bounds=(-1., -3., 4., 3.), resolution=.025)
    navigator.integrate(single)
    assert not np.any(navigator.grid == OCCUPIED)


def test_actual_rgb_doorway_preserves_disjoint_evidence_through_perception():
    calibration = json.loads((FIXTURES / "floor_rgb_calibration.json").read_text())
    pixels = cv2.cvtColor(cv2.imread(str(FIXTURES / "floor_rgb_doorway_texture_gap.png")),
                          cv2.COLOR_BGR2RGB)
    perception = CameraPerception(calibration)
    observation = perception.process(RGBPacket(pixels, 0.))
    assert observation.tracking
    assert observation.free_intervals == perception.last_floor.free_intervals
    assert sum(len(segments) > 1 for segments in observation.free_intervals) >= 30
    # Isolate one bearing so adjacent rays cannot conceal a filled texture gap.
    index = next(index for index, segments in enumerate(observation.free_intervals)
                 if len(segments) >= 2 and segments[1][0] - segments[0][1] > .15)
    segments = observation.free_intervals[index]
    single = replace(observation, angles=observation.angles[[index]],
        ranges=observation.ranges[[index]], min_ranges=observation.min_ranges[[index]],
        hits=observation.hits[[index]], hit_ranges=observation.hit_ranges[[index]],
        free_intervals=(segments,))
    navigator = Navigator(bounds=(-3., -3., 4., 3.), resolution=.01)
    navigator.integrate(single)
    origin = np.asarray(single.sensor_origin)
    direction = np.array([math.cos(single.angles[0]), math.sin(single.angles[0])])
    point = lambda distance: navigator.world_to_cell(*(origin + distance * direction))
    assert navigator.grid[point(sum(segments[0]) / 2)] == FREE
    assert navigator.grid[point((segments[0][1] + segments[1][0]) / 2)] == UNKNOWN
    assert navigator.grid[point(sum(segments[1]) / 2)] == FREE


def test_actual_open_floor_after_near_texture_gap_maps_only_supported_interval():
    calibration = json.loads((FIXTURES / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(FIXTURES / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], 320, 240)
    assert sensor.process(read("floor_rgb_bright_start.png")).valid
    # Final image from a zero-contact 22.44 m three-room run. Its smooth near
    # carpet patch previously hid all central interval evidence despite an
    # extensive, independently classified visible floor beyond that patch.
    floor = sensor.process(read("floor_rgb_near_texture_gap.png"))
    assert floor.valid, floor.reason
    central = np.abs(floor.angles) < math.radians(28.)
    assert np.count_nonzero(central & np.isfinite(floor.ranges)) >= 50
    middle = int(np.argmin(np.abs(floor.angles)))
    assert .35 <= floor.min_ranges[middle] <= .4
    assert floor.ranges[middle] >= 2.45
    assert not floor.hits[middle]
    observation = Observation(Pose(0., 0., 0.), floor.angles[[middle]],
        floor.ranges[[middle]], sensor.max_range, 0., sensor_origin=tuple(sensor.origin),
        min_ranges=floor.min_ranges[[middle]], hits=floor.hits[[middle]],
        hit_ranges=floor.hit_ranges[[middle]])
    navigator = Navigator(bounds=(-1., -3., 4., 3.), resolution=.025)
    navigator.integrate(observation)
    direction = np.array([math.cos(floor.angles[middle]), math.sin(floor.angles[middle])])
    # This sample is in the camera image but lacks texture support. Returning a
    # farther interval must not interpolate free evidence across this prefix.
    gap = sensor.origin + direction * .30
    established = sensor.origin + direction * .65
    assert navigator.grid[navigator.world_to_cell(*gap)] == UNKNOWN
    assert navigator.grid[navigator.world_to_cell(*established)] == FREE
    assert navigator.grid[navigator.world_to_cell(*sensor.origin)] == UNKNOWN
    assert not np.any(navigator.grid == OCCUPIED)
