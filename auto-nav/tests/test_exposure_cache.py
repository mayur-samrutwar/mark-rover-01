"""Exercise capture/control exposure sharing without starting a renderer."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from rover_nav.navigation import Observation, Pose
from rover_nav.vision import CameraPerception, RGBPacket
from rover_nav.vision_isaac import VisionRover


class FakeRGB:
    width, height = 32, 24

    def __init__(self):
        self.timestamp = 7.
        self.pixels = np.full((self.height, self.width, 3), 120, np.uint8)
        self.reads = 0

    def read(self):
        self.reads += 1
        # Separate transport requests for the same physical exposure return
        # separate envelopes and arrays with one exposure timestamp.
        return RGBPacket(self.pixels.copy(), self.timestamp)


class FakePerception:
    def __init__(self):
        self.calls = []

    def process(self, packet):
        self.calls.append(packet.timestamp)
        valid = bool(np.any(packet.pixels))
        return Observation(Pose(.01 * len(self.calls), 0., .2),
            np.array([-.1, 0., .1]), np.full(3, 2.), 3., packet.timestamp, tracking=valid)


def fake_rover():
    rover = VisionRover.__new__(VisionRover)
    rover.rgb, rover.perception = FakeRGB(), FakePerception()
    rover._exposure_timestamp = None
    rover._exposure_observation = None
    rover.capture_count = 0
    rover.capture_metadata = []
    rover.calibration = {"K": np.eye(3).tolist(), "T_base_camera": np.eye(4).tolist()}
    return rover


@pytest.mark.parametrize("capture_first", [True, False])
def test_capture_and_control_share_one_exposure_estimation(tmp_path, capture_first):
    rover = fake_rover()
    if capture_first:
        metadata = rover.capture(tmp_path)
        observation = rover.observe()
    else:
        observation = rover.observe()
        metadata = rover.capture(tmp_path)
    repeated = rover.observe()
    assert rover.rgb.reads == 3
    assert rover.perception.calls == [7.]
    assert repeated is observation
    assert observation.tracking
    assert metadata["estimated_base_pose"] == vars(observation.pose)
    assert metadata["simulation_time"] == observation.timestamp
    assert metadata["tracking"]
    assert metadata["depth_source"] is None
    assert metadata["image_sha256"] == hashlib.sha256((tmp_path / metadata["image"]).read_bytes()).hexdigest()
    saved = json.loads((tmp_path / "capture_0000.json").read_text())
    assert saved["estimated_base_pose"] == vars(observation.pose)

    # A new exposure must be consumed even when its pixels have not changed.
    rover.rgb.timestamp = 7.1
    advanced = rover.observe()
    assert rover.perception.calls == [7., 7.1]
    assert advanced is not observation
    assert advanced.pose != observation.pose


def test_fault_override_does_not_poison_cached_capture_tracking(tmp_path):
    rover = fake_rover()
    denied = rover.observe(tracking=False)
    metadata = rover.capture(tmp_path)
    normal = rover.observe()
    assert not denied.tracking
    assert normal.tracking
    assert metadata["tracking"]
    assert denied.pose == normal.pose
    assert rover.perception.calls == [7.]


def test_new_black_exposure_cannot_reuse_a_cached_valid_observation():
    rover = fake_rover()
    assert rover.observe().tracking
    rover.rgb.timestamp = 7.1
    rover.rgb.pixels.fill(0)
    failed = rover.observe()
    assert not failed.tracking
    assert failed.timestamp == 7.1
    assert rover.perception.calls == [7., 7.1]


@pytest.mark.parametrize("repeated_time", [3., 2.9])
def test_external_perception_still_rejects_duplicate_or_older_timestamps(repeated_time):
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    pixels = cv2.cvtColor(cv2.imread(str(fixtures / "floor_rgb_initial.png")), cv2.COLOR_BGR2RGB)
    perception = CameraPerception(calibration)
    initial = perception.process(RGBPacket(pixels, 3.))
    assert initial.tracking
    rejected = perception.process(RGBPacket(pixels.copy(), repeated_time))
    assert not rejected.tracking
    assert perception.last_vo.reason == "nonmonotonic_timestamp"
    assert rejected.pose == initial.pose
    assert perception.frames == 2
    resumed = perception.process(RGBPacket(pixels.copy(), 3.1))
    assert resumed.tracking
    assert perception.frames == 3


def test_rgb_localization_can_remain_valid_when_conservative_floor_mapping_fails(monkeypatch):
    from dataclasses import replace
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    pixels = cv2.cvtColor(cv2.imread(str(fixtures / "floor_rgb_initial.png")), cv2.COLOR_BGR2RGB)
    perception = CameraPerception(calibration)
    initial = perception.process(RGBPacket(pixels, 3.))
    assert initial.tracking
    floor = perception.last_floor
    unavailable_clearance = replace(floor, valid=False, reason="insufficient_visible_bearings",
        mask=np.zeros_like(floor.mask), ranges=np.full_like(floor.ranges, np.nan),
        min_ranges=np.full_like(floor.min_ranges, np.nan), hits=np.zeros_like(floor.hits),
        hit_ranges=np.full_like(floor.hit_ranges, np.nan))
    assert np.any(unavailable_clearance.tracking_mask)
    monkeypatch.setattr(perception.floor, "process", lambda rgb: unavailable_clearance)
    observation = perception.process(RGBPacket(pixels, 3.1))
    assert observation.tracking
    assert perception.last_vo.tracking
    assert not perception.last_floor.valid
    assert not np.any(np.isfinite(observation.ranges))
    assert not np.any(observation.hits)
    # Removing the raw feature evidence must still lose actual visual tracking;
    # mapping failure by itself can neither invent pose nor authorize motion.
    no_features = replace(unavailable_clearance, tracking_mask=np.zeros_like(floor.mask))
    monkeypatch.setattr(perception.floor, "process", lambda rgb: no_features)
    assert not perception.process(RGBPacket(pixels, 3.2)).tracking
