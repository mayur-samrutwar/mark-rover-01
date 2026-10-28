"""Offline end-to-end hardware authority tests; no phone, battery, or network.

Measured-looking values below are synthetic test fixtures, never a deployable
rover profile. Fake motors record method calls without opening a connection.
"""
from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest

import rover_nav.hardware as hardware
from rover_nav.hardware import CommandProof, HardwareRuntime, MotorAuthority
from rover_nav.hardware_camera import CameraCalibration, CameraFrameError
from rover_nav.hardware_config import HardwareConfig
from rover_nav.navigation import Observation, Pose
from rover_nav.vision import RGBPacket


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


def config():
    # Explicitly artificial calibration, isolated from all shipped config files.
    return HardwareConfig.from_dict({
        "physical": {"verified": True, "measurement_record": "offline-synthetic",
                     "footprint_radius_m": .27, "safety_margin_m": .12},
        "motor": {"verified": True, "measurement_record": "offline-synthetic",
                  "effective_track_width_m": .3, "max_motor_pwm": 200,
                  **{name: [[.001, 40], [.3, 200]] for name in
                     ("left_forward", "left_reverse", "right_forward", "right_reverse")}},
        "dynamics": {"verified": True, "measurement_record": "offline-synthetic",
                     "max_linear_m_s": .15, "max_angular_rad_s": .5,
                     "reaction_time_s": .6, "minimum_deceleration_m_s2": .2,
                     "velocity_error_bound_m_s": .02, "angular_error_bound_rad_s": .05,
                     "braking_distance_margin_m": .08},
        "timing": {"verified": True, "measurement_record": "offline-synthetic",
                   "max_observation_age_s": .2, "max_command_age_s": .15,
                   "control_period_s": .05},
        "measurements": {"date": "2026-10-28", "surface": "synthetic-test-only",
                         "battery": "disconnected", "load": "no-real-rover"}})


class FakeTransport:
    def __init__(self):
        self.calls = []
        self.armed = False
        self.fail_send = False

    def arm(self):
        self.calls.append(("arm",))
        self.armed = True

    def send_pwm(self, left, right, *, issued_at=None):
        self.calls.append(("pwm", left, right, issued_at))
        if self.fail_send:
            raise OSError("offline synthetic transport fault")
        assert self.armed

    def stop(self, reason="operator_stop"):
        self.calls.append(("stop", reason))
        self.armed = False
        return True


class FakeCamera:
    def __init__(self, clock, *, calibrated=True):
        self.clock = clock
        self.calibration = (SimpleNamespace(verified=True, perception_calibration={})
                            if calibrated else None)
        self.frames = deque()
        self.latest_timestamp = None
        self.reset_required = False
        self.reset_calls = 0

    def offer(self, *, timestamp=None):
        pixels = np.zeros((240, 320, 3), dtype=np.uint8)
        pixels.setflags(write=False)
        packet = RGBPacket(pixels, self.clock() if timestamp is None else timestamp)
        self.frames.append(packet)
        self.latest_timestamp = packet.timestamp
        return packet

    def status(self):
        age = None if self.latest_timestamp is None else self.clock() - self.latest_timestamp
        return {"live_ready": bool(self.calibration and age is not None and 0 <= age <= .2
                                    and not self.reset_required),
                "reset_required": self.reset_required}

    def read(self):
        if not self.frames:
            raise CameraFrameError("no_new_camera_frame")
        return self.frames.popleft()

    def reset(self):
        self.reset_calls += 1
        self.frames.clear()
        self.latest_timestamp = None
        self.reset_required = False


class FakePerception:
    def __init__(self, _):
        self.frames = 0
        self.last_floor = SimpleNamespace(valid=True)
        self.tracking = True
        self.pose = Pose(0., 0., 0.)
        self.block_entered = self.block_release = None

    def process(self, packet):
        if self.block_entered is not None:
            self.block_entered.set()
            assert self.block_release.wait(1.)
        self.frames += 1
        return Observation(self.pose, np.array([-.5, 0., .5]), np.array([2., 2., 2.]),
                           3., packet.timestamp, tracking=self.tracking,
                           hits=np.zeros(3, dtype=bool))

    def status(self):
        return {"source": "offline_fake_perception", "frames": self.frames}


class FakeNavigator:
    def __init__(self):
        self.status, self.reason = "following", "offline_synthetic_frontier"
        self.command = (.1, 0.)
        self.measured_speed = 0.
        self.measured_angular_rate = self.previous_measured_angular_rate = 0.
        self.events = []
        self.shape, self.resolution, self.origin = (2, 2), .05, (0., 0.)
        self.grid = np.full(self.shape, -1, dtype=np.int8)

    def update(self, _):
        return self.command

    def snapshot(self):
        return {"status": self.status, "reason": self.reason}


def runtime(monkeypatch, *, calibrated=True, verified_config=True):
    monkeypatch.setattr(hardware, "CameraPerception", FakePerception)
    clock, motors = Clock(), FakeTransport()
    camera = FakeCamera(clock, calibrated=calibrated)
    instance = HardwareRuntime(camera, config() if verified_config else HardwareConfig(),
                               motors, clock=clock)
    instance.nav = FakeNavigator()
    return instance, camera, motors, clock


def ready_and_arm(instance, camera):
    camera.offer()
    instance.process_once()
    assert instance.blockers() == []
    instance.arm()
    # Arming intentionally replaces the map and estimator. Keep a deterministic
    # controller fixture for authority integration after that real reset.
    instance.nav = FakeNavigator()


def drive_once(instance, camera):
    camera.offer()
    instance.process_once()
    instance.authority.tick()
    assert instance.authority.armed
    assert any(instance.authority.last_pwm)


def test_default_monitor_status_reset_and_shutdown_never_acquire_or_post(monkeypatch):
    instance, camera, motors, _ = runtime(monkeypatch, calibrated=False, verified_config=False)
    assert instance.status()["mode"] == "monitor"
    assert not instance.status()["motor_output_enabled"]
    assert instance.status()["known_layout"] is False
    instance.process_once()
    instance.reset()
    instance.status()
    instance.close()
    assert camera.reset_calls == 1
    assert motors.calls == []


def test_calibrated_shadow_processes_rgb_and_proposes_path_without_motor_write(monkeypatch):
    instance, camera, motors, _ = runtime(monkeypatch)
    packet = camera.offer()
    instance.process_once()
    status = instance.status()
    assert status["mode"] == "monitor" and status["proposed_command"] == [.1, 0.]
    assert status["physical_pwm"] == [0, 0]
    assert status["pose"] == dict(x=0., y=0., yaw=0.)
    assert instance.last_observation.timestamp == packet.timestamp
    assert motors.calls == []


@pytest.mark.parametrize("camera_verified,physical_verified", [(False, True), (True, False), (False, False)])
def test_arm_rejects_unready_or_unmeasured_system_before_transport(monkeypatch, camera_verified, physical_verified):
    instance, camera, motors, _ = runtime(monkeypatch, calibrated=camera_verified,
                                        verified_config=physical_verified)
    camera.offer()
    instance.process_once()
    with pytest.raises(ValueError):
        instance.arm()
    assert not instance.authority.armed
    assert motors.calls == []


def test_explicit_arm_starts_zero_and_next_frame_alone_may_authorize_pwm(monkeypatch):
    instance, camera, motors, _ = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    assert motors.calls == [("arm",)]
    assert instance.last_observation is None and instance.last_command == (0., 0.)
    instance.authority.tick()
    assert motors.calls[-1][1:3] == (0, 0)
    drive_once(instance, camera)
    assert motors.calls[-1][1:3] == config().pwm_for_velocity(.1, 0.)


def test_no_frame_after_arming_expires_initial_zero_proof_and_requires_rearm(monkeypatch):
    instance, camera, motors, clock = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    clock.now += .151
    instance.authority.tick()
    assert not instance.authority.armed
    assert instance.authority.fault == "navigation_processing_stale"
    assert motors.calls[-1][0] == "stop"
    before = list(motors.calls)
    instance.authority.heartbeat()
    camera.offer()
    instance.process_once()
    instance.authority.tick()
    assert motors.calls == before
    assert not instance.authority.armed
    instance.arm()
    assert motors.calls[-1] == ("arm",)
    instance.authority.stop()


def test_camera_dropout_stops_without_any_more_perception_calls(monkeypatch):
    instance, camera, motors, clock = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    drive_once(instance, camera)
    frames = instance.perception.frames
    clock.now += .201
    instance.authority.tick()
    assert instance.authority.fault == "camera_stale"
    assert not motors.armed and instance.perception.frames == frames
    assert motors.calls[-1] == ("stop", "camera_stale")


def test_command_processing_deadline_expires_before_camera_age_bound(monkeypatch):
    instance, camera, motors, clock = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    drive_once(instance, camera)
    clock.now += .151
    instance.authority.tick()
    assert instance.authority.fault == "navigation_processing_stale"
    assert not motors.armed


def test_operator_presence_is_independent_of_fresh_camera_and_commands():
    clock, motors = Clock(), FakeTransport()
    authority = MotorAuthority(motors, config(), clock=clock)
    authority.arm()
    clock.now += .5
    authority.heartbeat()
    clock.now += .6
    authority.publish(CommandProof(.1, 0., clock(), clock(), authority.generation))
    authority.tick()
    assert authority.armed
    clock.now += .151
    authority.publish(CommandProof(.1, 0., clock(), clock(), authority.generation))
    authority.tick()
    assert not authority.armed and authority.fault == "operator_heartbeat_lost"
    assert motors.calls[-1] == ("stop", "operator_heartbeat_lost")


def test_motor_authority_expires_while_vision_holds_runtime_lock(monkeypatch):
    instance, camera, motors, clock = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    drive_once(instance, camera)
    entered, release = threading.Event(), threading.Event()
    instance.perception.block_entered, instance.perception.block_release = entered, release
    camera.offer()
    worker = threading.Thread(target=instance.process_once)
    worker.start()
    assert entered.wait(1.)
    clock.now += .201
    started = time.monotonic()
    instance.authority.tick()
    assert time.monotonic() - started < .1
    assert not instance.authority.armed and not motors.armed
    release.set()
    worker.join(1.)
    assert not worker.is_alive()
    assert instance.authority.proof is None
    assert not instance.authority.armed


def test_stop_discards_inflight_vision_result_and_old_generation_cannot_restart(monkeypatch):
    instance, camera, motors, clock = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    old_generation = instance.authority.generation
    entered, release = threading.Event(), threading.Event()
    instance.perception.block_entered, instance.perception.block_release = entered, release
    camera.offer()
    worker = threading.Thread(target=instance.process_once)
    worker.start()
    assert entered.wait(1.)
    instance.authority.stop("operator_stop")
    assert not instance.authority.armed and not motors.armed
    release.set()
    worker.join(1.)
    assert not worker.is_alive()
    instance.authority.publish(CommandProof(.1, 0., clock(), clock(), old_generation))
    instance.authority.tick()
    assert instance.authority.proof is None
    assert motors.calls == [("arm",), ("stop", "operator_stop")]


@pytest.mark.parametrize("defect", ["tracking", "floor", "paused", "overspeed", "yaw_overspeed"])
def test_runtime_observed_faults_stop_instead_of_publishing_motion(monkeypatch, defect):
    instance, camera, motors, clock = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    drive_once(instance, camera)
    clock.now += .1
    if defect == "tracking":
        instance.perception.tracking = False
    elif defect == "floor":
        instance.perception.last_floor.valid = False
    elif defect == "paused":
        instance.nav.status = "paused"
    elif defect == "overspeed":
        instance.perception.pose = Pose(.1, 0., 0.)
    elif defect == "yaw_overspeed":
        instance.perception.pose = Pose(0., 0., .2)
    camera.offer()
    instance.process_once()
    assert not instance.authority.armed and not motors.armed
    assert instance.error
    assert instance.authority.proof is None
    assert motors.calls[-1][0] == "stop"


def test_camera_settings_generation_change_stops_and_reset_keeps_motors_disarmed(monkeypatch):
    instance, camera, motors, _ = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    camera.reset_required = True
    instance.process_once()
    assert not instance.authority.armed and instance.error == "camera_stream_changed_reset_required"
    stopped = list(motors.calls)
    instance.reset()
    assert camera.reset_calls == 1 and instance.last_observation is None
    assert motors.calls == stopped
    assert instance.status()["preflight_blockers"]


def test_reset_refuses_an_active_motor_session(monkeypatch):
    instance, camera, motors, _ = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    with pytest.raises(ValueError, match="Stop"):
        instance.reset()
    assert instance.authority.armed and camera.reset_calls == 0
    instance.authority.stop()


def test_completed_exploration_releases_owned_session(monkeypatch):
    instance, camera, motors, _ = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    instance.nav.status = "complete"
    camera.offer()
    instance.process_once()
    assert instance.authority.fault == "exploration_complete"
    assert not instance.authority.armed and not motors.armed


def test_transport_failure_latches_and_fresh_proofs_cannot_rearm(monkeypatch):
    instance, camera, motors, clock = runtime(monkeypatch)
    ready_and_arm(instance, camera)
    motors.fail_send = True
    camera.offer()
    instance.process_once()
    instance.authority.tick()
    assert instance.authority.fault == "motor_transport_or_calibration_fault"
    assert not instance.authority.armed and not motors.armed
    stopped = list(motors.calls)
    instance.authority.publish(CommandProof(.1, 0., clock(), clock(), instance.authority.generation))
    instance.authority.tick()
    assert motors.calls == stopped


def test_actual_camera_perception_can_run_through_shadow_runtime_without_simulator(monkeypatch):
    # Optional existing recorded RGB fixture: this tests real perception plumbing,
    # not hardware calibration. It imports no scene, simulator or evaluator pose.
    directory = Path(__file__).resolve().parents[1] / "artifacts/rgb-playable-seed-7-v6/captures"
    image_path, record_path = directory / "capture_0000.png", directory / "capture_0000.json"
    if not image_path.exists() or not record_path.exists():
        pytest.skip("Optional recorded RGB fixture is not present")
    record = json.loads(record_path.read_text())
    calibration = CameraCalibration.from_dict({
        "version": 1, "camera_id": "OFFLINE-RECORDED-FIXTURE-NOT-HARDWARE",
        "source_width": record["width"], "source_height": record["height"],
        "width": record["width"], "height": record["height"],
        "K": record["K"], "T_base_camera": record["T_base_camera"],
        "distortion": [0., 0., 0., 0., 0.],
        "settings": {"device_id": "offline-fixture", "width": record["width"], "height": record["height"]},
        "verified": True, "timing": {"max_pipeline_age_s": .05, "validated": False}})
    clock, motors = Clock(), FakeTransport()
    camera = FakeCamera(clock)
    camera.calibration = calibration
    pixels = np.asarray(Image.open(image_path).convert("RGB"))
    camera.frames.append(RGBPacket(pixels, clock()))
    camera.latest_timestamp = clock()
    instance = HardwareRuntime(camera, HardwareConfig(), motors, clock=clock)
    instance.process_once()
    assert instance.error is None
    assert instance.perception.frames == 1
    assert instance.last_observation is not None
    assert instance.status()["perception"]["source"] == "rgb_only"
    assert instance.status()["mode"] == "monitor"
    assert motors.calls == []
