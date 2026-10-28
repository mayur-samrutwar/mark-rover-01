"""Independent offline edge cases: no real motor, camera, or network access."""
import math
import subprocess
import sys
import threading

import numpy as np
import pytest

from rover_nav.hardware import CommandProof, HardwareNavigator, MotorAuthority, stopping_paths
from test_hardware_runtime import Clock, FakeTransport, config


def test_hardware_import_does_not_import_simulation():
    script = "import sys; import rover_nav.hardware; assert not any(x in sys.modules for x in ['mujoco', 'rover_nav.mission', 'rover_nav.isaac', 'rover_nav.vision_isaac', 'rover_nav.world'])"
    subprocess.run([sys.executable, "-c", script], check=True, capture_output=True, timeout=10)


def test_stop_envelope_contains_opposite_measured_and_previous_yaw():
    parameters = dict(reaction=.6, deceleration=.2, observation_age=.2, yaw_error=.02, buffer=.08)
    base = stopping_paths(.15, 0., 0., **parameters)
    actual = stopping_paths(.15, 0., 0., measured_angular=-.5, previous_measured_angular=.4, **parameters)
    base_min = min(path[:, 1].min() for path in base)
    assert min(path[:, 1].min() for path in actual) < base_min - .03
    assert max(path[:, 1].max() for path in actual) > .03
    assert all(np.all(np.isfinite(path)) and np.allclose(path[0], [0, 0]) for path in actual)


def test_hardware_stop_uses_observation_age_speed_error_and_measured_speed():
    cfg = config()
    nav = HardwareNavigator(cfg)
    nav.measured_speed = .17
    paths = nav._rgb_braking_paths(.1, 0.)
    speed = .17 + cfg.velocity_error_bound_m_s
    expected = speed * (cfg.reaction_time_s + cfg.max_observation_age_s) + speed ** 2 / (2 * cfg.minimum_deceleration_m_s2) + cfg.braking_distance_margin_m
    assert max(path[-1, 0] for path in paths) >= expected - .001


def test_authority_arm_exposes_atomic_zero_proof():
    clock, motor = Clock(), FakeTransport()
    authority = MotorAuthority(motor, config(), clock=clock)
    authority.arm(observation_time=clock() - .01)
    assert authority.armed
    assert authority.proof == CommandProof(0., 0., clock(), clock() - .01, authority.generation)
    authority.tick()
    assert motor.calls[-1][0] == "pwm" and motor.calls[-1][1:3] == (0, 0)


@pytest.mark.parametrize("field,value", [("issued_at", math.nan), ("issued_at", math.inf),
    ("observation_time", math.nan), ("observation_time", -math.inf), ("linear", math.nan),
    ("angular", math.inf), ("generation", True), ("linear", -.05)])
def test_invalid_proof_never_reaches_pwm_and_latches_stop(field, value):
    clock, motor = Clock(), FakeTransport()
    authority = MotorAuthority(motor, config(), clock=clock)
    authority.arm()
    values = dict(linear=.1, angular=0., issued_at=clock(), observation_time=clock(), generation=authority.generation)
    values[field] = value
    assert authority.publish(CommandProof(**values)) is False
    authority.tick()
    assert not authority.armed and authority.proof is None
    assert not any(call[0] == "pwm" for call in motor.calls)


@pytest.mark.parametrize("timestamp", [math.nan, math.inf, 101., 99.])
def test_bad_initial_exposure_blocks_even_session_acquisition(timestamp):
    clock, motor = Clock(), FakeTransport()
    authority = MotorAuthority(motor, config(), clock=clock)
    with pytest.raises(ValueError, match="observation"):
        authority.arm(observation_time=timestamp)
    assert motor.calls == []


def test_stop_cancels_blocked_send_without_waiting_for_its_network_lock():
    clock = Clock()
    entered, release = threading.Event(), threading.Event()
    class BlockedMotor(FakeTransport):
        def send_pwm(self, *args, **kwargs):
            entered.set()
            assert release.wait(1.)
    motor = BlockedMotor()
    authority = MotorAuthority(motor, config(), clock=clock)
    authority.arm()
    authority.publish(CommandProof(.1, 0., clock(), clock(), authority.generation))
    worker = threading.Thread(target=authority.tick)
    worker.start()
    assert entered.wait(.5)
    stopper = threading.Thread(target=lambda: authority.stop("operator_stop"))
    stopper.start()
    try:
        stopper.join(.15)
        assert not stopper.is_alive(), "STOP waited for an in-flight movement request"
        assert not authority.armed and authority.last_pwm == (0, 0)
        with pytest.raises(ValueError, match="pending"):
            authority.arm()
    finally:
        release.set()
        worker.join(1.)
        stopper.join(1.)
    assert authority.last_pwm == (0, 0)


def test_stop_cancels_inflight_arm_and_prevents_reacquisition_until_it_finishes():
    clock = Clock()
    entered, release = threading.Event(), threading.Event()
    errors = []
    class BlockedMotor(FakeTransport):
        def arm(self):
            super().arm()
            entered.set()
            assert release.wait(1.)
    motor = BlockedMotor()
    authority = MotorAuthority(motor, config(), clock=clock)
    def acquire():
        try:
            authority.arm()
        except Exception as error:
            errors.append(error)
    worker = threading.Thread(target=acquire)
    worker.start()
    assert entered.wait(.5)
    authority.stop("operator_stop")
    assert not authority.armed
    with pytest.raises(ValueError, match="pending"):
        authority.arm()
    release.set()
    worker.join(1.)
    assert errors and "cancelled" in str(errors[0])
    assert not authority.armed and authority.proof is None and not motor.armed


def test_camera_age_rechecked_after_slow_arm_and_owned_zero_session_released():
    clock = Clock()
    class SlowMotor(FakeTransport):
        def arm(self):
            super().arm()
            clock.now += .21
    motor = SlowMotor()
    authority = MotorAuthority(motor, config(), clock=clock)
    with pytest.raises(ValueError, match="observation"):
        authority.arm(observation_time=clock())
    assert not authority.armed and not motor.armed
    assert motor.calls == [("arm",), ("stop", "arming_failed")]


def test_proof_expiring_during_send_is_stopped_after_ack():
    clock = Clock()
    class SlowMotor(FakeTransport):
        def send_pwm(self, *args, **kwargs):
            super().send_pwm(*args, **kwargs)
            clock.now += .16
    motor = SlowMotor()
    authority = MotorAuthority(motor, config(), clock=clock)
    authority.arm()
    authority.publish(CommandProof(.1, 0., clock(), clock(), authority.generation))
    authority.tick()
    assert not authority.armed and not motor.armed
    assert authority.fault == "navigation_processing_stale"
    assert authority.last_pwm == (0, 0)


def test_cancelled_operation_never_acquires_remote_session():
    motor = FakeTransport()
    authority = MotorAuthority(motor, config(), clock=Clock())
    with pytest.raises(ValueError, match="cancelled"):
        authority.arm(cancelled=lambda: True)
    assert motor.calls == [] and not authority.armed


def test_cancellation_during_zero_session_acquisition_releases_it_before_arming():
    cancellation = threading.Event()
    class CancellingMotor(FakeTransport):
        def arm(self):
            super().arm()
            cancellation.set()
    motor = CancellingMotor()
    authority = MotorAuthority(motor, config(), clock=Clock())
    with pytest.raises(ValueError, match="cancelled"):
        authority.arm(cancelled=cancellation.is_set)
    assert motor.calls == [("arm",), ("stop", "arming_failed")]
    assert not authority.armed and not motor.armed and authority.proof is None


def test_broken_cancellation_predicate_fails_closed_without_motor_calls():
    def broken_gate():
        raise RuntimeError("cancel state unavailable")
    motor = FakeTransport()
    authority = MotorAuthority(motor, config(), clock=Clock())
    with pytest.raises(ValueError, match="cancelled"):
        authority.arm(cancelled=broken_gate)
    assert motor.calls == []


def test_runtime_cancelled_before_preflight_has_no_motor_calls(monkeypatch):
    from test_hardware_runtime import runtime
    instance, _, motor, _ = runtime(monkeypatch)
    with pytest.raises(ValueError, match="cancelled"):
        instance.arm(cancelled=lambda: True)
    assert motor.calls == [] and not instance.authority.armed


def test_runtime_cancellation_after_reset_cannot_acquire_motor_session(monkeypatch):
    from test_hardware_runtime import runtime
    instance, camera, motor, _ = runtime(monkeypatch)
    camera.offer()
    instance.process_once()
    cancellation = threading.Event()
    original_reset = instance._reset_pipeline
    def resetting():
        original_reset()
        cancellation.set()
    monkeypatch.setattr(instance, "_reset_pipeline", resetting)
    with pytest.raises(ValueError, match="cancelled"):
        instance.arm(cancelled=cancellation.is_set)
    assert motor.calls == [] and not instance.authority.armed
