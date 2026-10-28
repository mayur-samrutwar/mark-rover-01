"""Real-camera navigation with an independent, expiring motor authority.

Importing this module never imports a simulator, contacts a rover, or arms it.
The frozen RGB planner is reused through a hardware-specific stopping model.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import math
import threading
import time

import numpy as np

from .navigation import Navigator, wrap_angle
from .vision import CameraPerception


def _arming_cancelled(cancelled):
    """Cancellation predicates are nonblocking; an unreadable gate fails closed."""
    if cancelled is None:
        return False
    try:
        return bool(cancelled())
    except Exception:
        return True


def stopping_paths(speed, angular, previous_angular, *, reaction, deceleration,
                   observation_age, yaw_error, buffer, measured_angular=0., previous_measured_angular=0.):
    """Swept centre paths using measured conservative hardware parameters."""
    speed = max(0., float(speed))
    if speed <= 1e-9:
        return [np.zeros((1, 2))]
    delay = reaction + max(0., observation_age)
    duration = delay + speed / deceleration
    steps = max(1, math.ceil(duration / .01))
    dt = duration / steps
    turns = {0., angular, previous_angular, measured_angular, previous_measured_angular}
    turns |= {value + error for value in tuple(turns) for error in (-yaw_error, yaw_error)}
    result = []
    for turn in sorted(turns):
        for keep_curvature in (False, True):
            points, heading = [np.zeros(2)], 0.
            for index in range(steps):
                t = (index + .5) * dt
                velocity = speed if t <= delay else max(0., speed - deceleration * (t - delay))
                rate = turn * (velocity / speed if keep_curvature else 1.)
                mid = heading + rate * dt / 2
                points.append(points[-1] + velocity * dt * np.array([math.cos(mid), math.sin(mid)]))
                heading += rate * dt
            end = points[-1].copy()
            tangent = np.array([math.cos(heading), math.sin(heading)])
            for distance in np.linspace(0., buffer, max(2, math.ceil(buffer / .01) + 1))[1:]:
                points.append(end + distance * tangent)
            result.append(np.asarray(points))
    return result


class HardwareNavigator(Navigator):
    """Reuse mapping/planning, replacing simulated stopping assumptions."""
    def __init__(self, config):
        self.hardware_config = config
        self.measured_speed = 0.
        self.measured_angular_rate = 0.
        self.previous_measured_angular_rate = 0.
        # Fallbacks allow a clearly labelled preview; they never authorize drive.
        super().__init__(bounds=(-10., -10., 10., 10.), resolution=.05,
                         robot_radius=config.footprint_radius_m or .25,
                         safety_margin=config.safety_margin_m if config.safety_margin_m is not None else .1,
                         account_cell_area=True, survey_clearance=.6,
                         survey_angular_speed=min(config.max_angular_rad_s or .15, .3),
                         turn_limit=min(config.max_angular_rad_s or .2, .8))
        self.initialize_own_footprint()

    def _rgb_braking_paths(self, speed, angular, previous_angular=0., *, observation_age=0.):
        cfg = self.hardware_config
        if cfg.live_blockers():
            # Simulation parameters are permitted only for shadow diagnostics.
            return super()._rgb_braking_paths(speed, angular, previous_angular,
                                              observation_age=observation_age)
        bounded_speed = max(min(speed, cfg.max_linear_m_s), self.measured_speed) + cfg.velocity_error_bound_m_s
        return stopping_paths(bounded_speed, angular, previous_angular,
                              reaction=cfg.reaction_time_s,
                              deceleration=cfg.minimum_deceleration_m_s2,
                              observation_age=max(observation_age, cfg.max_observation_age_s),
                              yaw_error=cfg.angular_error_bound_rad_s,
                              buffer=cfg.braking_distance_margin_m,
                              measured_angular=self.measured_angular_rate,
                              previous_measured_angular=self.previous_measured_angular_rate)

    def update(self, observation):
        linear, angular = super().update(observation)
        cfg = self.hardware_config
        command = (max(0., min(linear, cfg.max_linear_m_s or .08)),
                   float(np.clip(angular, -(cfg.max_angular_rad_s or .15), cfg.max_angular_rad_s or .15)))
        if not cfg.live_blockers() and any(command):
            reason = self._rgb_swept_stop_reason(observation, *command, self.traversable(),
                                                 observation_age=cfg.max_observation_age_s)
            if reason:
                self._request_replan(reason, observation.timestamp)
                command = (0., 0.)
        self._last_command = command
        return command


@dataclass(frozen=True)
class CommandProof:
    linear: float
    angular: float
    issued_at: float
    observation_time: float
    generation: int


class MotorAuthority:
    """Only this watchdog may renew a motor lease; vision never writes motors.

    A proof covers the *maximum* allowed image age. Every renewal checks that
    bound, command age, operator presence and generation independently of vision.
    There is no queue of delayed commands and no automatic rearm.
    """
    def __init__(self, transport, config, *, clock=time.monotonic):
        self.transport, self.config, self.clock = transport, config, clock
        self.armed = False
        self.fault = None
        self.generation = 0
        self.proof = None
        self.operator_until = -math.inf
        self.last_pwm = (0, 0)
        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread = None
        self._arming = False
        self._sending = False

    def arm(self, *, observation_time=None, cancelled=None):
        with self._lock:
            if _arming_cancelled(cancelled):
                raise ValueError("Arming was cancelled")
            blockers = self.config.live_blockers()
            if blockers:
                raise ValueError("Hardware preflight: " + "; ".join(blockers))
            if self.armed or self._arming or self._sending or self._shutdown.is_set():
                raise ValueError("Stop and finish pending motor operations before arming")
            initial_now = self.clock()
            initial_exposure = initial_now if observation_time is None else observation_time
            if (not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                        for value in (initial_now, initial_exposure)) or initial_exposure > initial_now
                    or initial_now - initial_exposure > self.config.max_observation_age_s):
                raise ValueError("Fresh finite observation time is required to arm")
            self.generation += 1
            generation = self.generation
            self._arming = True
        try:
            # Do not hold the authority lock across I/O. STOP must invalidate
            # this acquisition while a request is still blocked.
            self.transport.arm()
            now = self.clock()
            exposure = initial_exposure
            if (not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                        for value in (now, exposure)) or exposure > now
                    or now - exposure > self.config.max_observation_age_s):
                raise ValueError("Fresh finite observation time is required to arm")
            with self._lock:
                if generation != self.generation or self._shutdown.is_set() or _arming_cancelled(cancelled):
                    raise ValueError("Arming was cancelled")
                # The initial zero-only proof and armed state become visible
                # atomically; the watchdog can never observe an unproved arm.
                self.proof = CommandProof(0., 0., now, exposure, generation)
                self.fault = None
                self.operator_until = now + .75
                self.armed = True
        except Exception:
            with self._lock:
                current = generation == self.generation
                if current:
                    self._arming = False
            if current:
                # arm() can have acquired a remote zero session before failing
                # freshness validation. Release only that owned acquisition.
                self.transport.stop("arming_failed")
            raise
        finally:
            with self._lock:
                self._arming = False

    def heartbeat(self):
        with self._lock:
            if self.armed:
                self.operator_until = self.clock() + .75

    def publish(self, proof):
        with self._lock:
            if not self.armed:
                return False
            if isinstance(proof, CommandProof) and proof.generation != self.generation:
                return False
            reason = self._proof_fault(proof, self.clock(), self.config, self.generation, self.operator_until)
            if not reason:
                self.proof = proof
                return True
        self.stop(reason)
        return False

    def stop(self, reason="operator_stop"):
        with self._lock:
            owned = self.armed or self._arming
            self.armed = False
            self.generation += 1
            self.proof = None
            self.fault = reason
            self.last_pwm = (0, 0)
        # Never send STOP into someone else's manual session.
        if owned:
            return self.transport.stop(reason)
        return True

    @staticmethod
    def _proof_fault(proof, now, cfg, generation, operator_until):
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
            return "invalid_control_clock"
        if not math.isfinite(operator_until) or now > operator_until:
            return "operator_heartbeat_lost"
        if proof is None:
            return "awaiting_fresh_navigation_proof"
        if not isinstance(proof, CommandProof):
            return "invalid_navigation_proof"
        if (type(proof.generation) is not int or proof.generation != generation):
            return "command_generation_changed"
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
               for value in (proof.linear, proof.angular, proof.issued_at, proof.observation_time)):
            return "invalid_navigation_proof"
        if proof.linear < 0.:
            return "reverse_autonomy_is_unsupported"
        if now < proof.issued_at or now < proof.observation_time or proof.issued_at < proof.observation_time:
            return "future_navigation_proof"
        if now - proof.observation_time > cfg.max_observation_age_s:
            return "camera_stale"
        if now - proof.issued_at > cfg.max_command_age_s:
            return "navigation_processing_stale"
        return None

    def tick(self):
        with self._lock:
            if not self.armed or self._sending:
                return
            now, proof, cfg = self.clock(), self.proof, self.config
            generation = self.generation
            reason = self._proof_fault(proof, now, cfg, generation, self.operator_until)
            if not reason:
                self._sending = True
        if reason:
            self.stop(reason)
            return
        try:
            pwm = cfg.pwm_for_velocity(proof.linear, proof.angular)
            # The transport has its own cancellation epoch. A simultaneous
            # STOP invalidates it independently of this metadata lock.
            self.transport.send_pwm(*pwm, issued_at=proof.issued_at)
            with self._lock:
                current = self.armed and generation == self.generation
                expired = self._proof_fault(proof, self.clock(), cfg, generation, self.operator_until) if current else None
                if current and not expired:
                    self.last_pwm = pwm
            if expired:
                self.stop(expired)
        except Exception:
            with self._lock:
                current = self.armed and generation == self.generation
            if current:
                self.stop("motor_transport_or_calibration_fault")
        finally:
            with self._lock:
                self._sending = False

    def start_watchdog(self):
        if self._thread:
            return
        def worker():
            while not self._shutdown.wait(min(.025, self.config.control_period_s or .025)):
                self.tick()
        self._thread = threading.Thread(target=worker, name="rover-motor-authority", daemon=True)
        self._thread.start()

    def close(self):
        self._shutdown.set()
        self.stop("shutdown")
        if self._thread:
            self._thread.join(timeout=.4)


class HardwareRuntime:
    """Stationary camera/shadow bring-up and explicitly armed hardware autonomy."""
    def __init__(self, camera, config, transport, *, clock=time.monotonic):
        self.camera, self.config, self.clock = camera, config, clock
        self.authority = MotorAuthority(transport, config, clock=clock)
        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread = None
        self.events = deque(maxlen=500)
        self.trajectory = deque(maxlen=20000)
        self.last_observation = None
        self.last_processed_at = None
        self.last_command = (0., 0.)
        self.error = None
        self._reset_pipeline()

    def _reset_pipeline(self):
        self.nav = HardwareNavigator(self.config)
        calibration = self.camera.calibration
        self.perception = CameraPerception(calibration.perception_calibration) if calibration and calibration.verified else None
        self.last_observation = None
        self.last_processed_at = None
        self.last_command = (0., 0.)
        self.trajectory.clear()
        self._last_event = 0

    def log(self, event, **fields):
        self.events.append({"type": event, "time": self.clock(), **fields})

    def blockers(self):
        result = list(self.config.live_blockers())
        camera = self.camera.status()
        if not camera.get("live_ready"):
            result.append("camera_calibration_timing_or_stream_not_ready")
        if self.last_observation is None or not self.last_observation.tracking:
            result.append("valid_visual_tracking_required")
        elif self.clock() - self.last_observation.timestamp > (self.config.max_observation_age_s or .35):
            result.append("fresh_camera_observation_required")
        if self.perception and (not self.perception.last_floor or not self.perception.last_floor.valid):
            result.append("valid_floor_perception_required")
        return list(dict.fromkeys(result))

    def reset(self):
        with self._lock:
            if self.authority.armed or self.authority._arming:
                raise ValueError("Stop autonomy before resetting")
            self.camera.reset()
            self._reset_pipeline()
            self.error = None
            self.log("navigation_reset", note="Map and visual estimator reset together; motors remain disarmed")

    def arm(self, *, cancelled=None):
        with self._lock:
            if _arming_cancelled(cancelled):
                raise ValueError("Arming was cancelled")
            blockers = self.blockers()
            if blockers:
                raise ValueError("; ".join(blockers))
            if _arming_cancelled(cancelled):
                raise ValueError("Arming was cancelled")
            # Do not carry a map built while the mount was being adjusted into drive.
            exposure_time = self.last_observation.timestamp
            self._reset_pipeline()
            if _arming_cancelled(cancelled):
                raise ValueError("Arming was cancelled")
            self.authority.arm(observation_time=exposure_time, cancelled=cancelled)
            if _arming_cancelled(cancelled):
                self.authority.stop("arming_cancelled")
                raise ValueError("Arming was cancelled")
            self.log("autonomy_armed", note="Explicit operator request; fresh map and RGB estimator")

    def process_once(self):
        from .hardware_camera import CameraFrameError
        with self._lock:
            if self.camera.status().get("reset_required"):
                self.authority.stop("camera_stream_changed_reset_required")
                self.error = "camera_stream_changed_reset_required"
                return
            if self.perception is None:
                return
            try:
                packet = self.camera.read()
            except CameraFrameError:
                return  # Independent motor authority expires; no stale frame reuse.
            generation = self.authority.generation
            try:
                observation = self.perception.process(packet)
                now = self.clock()
                if now - packet.timestamp > (self.config.max_observation_age_s or .35):
                    raise ValueError("camera_processing_too_slow")
                previous = self.last_observation
                speed = yaw_rate = lateral_speed = forward_speed = 0.
                if previous and observation.timestamp > previous.timestamp:
                    dt = observation.timestamp - previous.timestamp
                    dx = (observation.pose.x - previous.pose.x) / dt
                    dy = (observation.pose.y - previous.pose.y) / dt
                    speed = math.hypot(dx, dy)
                    yaw_rate = wrap_angle(observation.pose.yaw - previous.pose.yaw) / dt
                    # Midpoint attitude prevents an ordinary turning arc from
                    # masquerading as lateral slip over this image interval.
                    heading = previous.pose.yaw + yaw_rate * dt / 2.
                    forward_speed = math.cos(heading) * dx + math.sin(heading) * dy
                    lateral_speed = -math.sin(heading) * dx + math.cos(heading) * dy
                self.nav.measured_speed = speed
                self.nav.previous_measured_angular_rate = self.nav.measured_angular_rate
                self.nav.measured_angular_rate = yaw_rate
                command = self.nav.update(observation)
                now = self.clock()
                if (not math.isfinite(now) or now < packet.timestamp
                        or now - packet.timestamp > (self.config.max_observation_age_s or .35)):
                    raise ValueError("camera_processing_too_slow")
                self.last_observation, self.last_processed_at = observation, now
                self.last_command = command
                self.trajectory.append([observation.pose.x, observation.pose.y])
                invalid = self.nav.status in {"paused", "stalled", "blocked"}
                if self.authority.armed:
                    cfg = self.config
                    if speed > cfg.max_linear_m_s + cfg.velocity_error_bound_m_s:
                        raise ValueError("measured_linear_overspeed")
                    if abs(yaw_rate) > cfg.max_angular_rad_s + cfg.angular_error_bound_rad_s:
                        raise ValueError("measured_angular_overspeed")
                    if (forward_speed < -cfg.velocity_error_bound_m_s
                            or abs(lateral_speed) > cfg.velocity_error_bound_m_s):
                        raise ValueError("measured_reverse_or_lateral_slip")
                    if not observation.tracking or invalid or not self.perception.last_floor.valid:
                        raise ValueError("visual_or_navigation_fault")
                    if self.nav.status == "complete":
                        self.authority.stop("exploration_complete")
                    else:
                        self.authority.publish(CommandProof(*command, now, packet.timestamp, generation))
                for event in self.nav.events[self._last_event:]:
                    self.events.append(dict(event, source="navigator"))
                self._last_event = len(self.nav.events)
                self.error = None
            except Exception as error:
                self.error = str(error)
                self.authority.stop("perception_or_navigation_fault")
                self.log("navigation_fault", reason=self.error)

    def status(self):
        # Status never contacts or arms motors. It may wait for one frame's processing.
        with self._lock:
            nav = self.nav.snapshot()
            observation = self.last_observation
            return {"mode": "autonomy" if self.authority.armed else "monitor",
                    "motor_output_enabled": self.authority.armed,
                    "fault": self.authority.fault, "error": self.error,
                    "preflight_blockers": self.blockers(), "camera": self.camera.status(),
                    "navigation": nav, "perception": self.perception.status() if self.perception else None,
                    "proposed_command": list(self.last_command), "physical_pwm": list(self.authority.last_pwm),
                    "pose": vars(observation.pose) if observation else None,
                    "trajectory": list(self.trajectory), "events": list(self.events),
                    "map": {"width": self.nav.shape[1], "height": self.nav.shape[0],
                            "resolution": self.nav.resolution, "origin": self.nav.origin,
                            "cells": self.nav.grid.ravel().tolist()},
                    "known_layout": False,
                    "footprint_measured": self.config.footprint_radius_m is not None,
                    "timing_note": "Browser timing is a bounded estimate only after physical timing validation"}

    def start(self):
        self.authority.start_watchdog()
        def worker():
            while not self._shutdown.wait(.01):
                try:
                    self.process_once()
                except Exception:
                    self.error = "camera_worker_fault"
                    self.authority.stop("camera_worker_fault")
        self._thread = threading.Thread(target=worker, name="rover-rgb-navigation", daemon=True)
        self._thread.start()

    def close(self):
        self.authority.close()
        self._shutdown.set()
        if self._thread:
            self._thread.join(timeout=1.)
