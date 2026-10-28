"""Measured physical limits for the real rover. No simulation calibration defaults.

Loading an incomplete profile is useful for shadow inspection. It never makes
that profile eligible to drive: ``live_blockers`` lists every missing proof.
Verification flags record the operator's measurements, not certification.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date
import json
import math
from pathlib import Path
from typing import Any


class HardwareConfigError(ValueError):
    """A physical profile is malformed or cannot support a requested command."""


def _number(section, key, *, minimum=0., maximum=None, strictly_positive=False):
    value = section.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise HardwareConfigError(f"{key} must be a finite number or null")
    value = float(value)
    if value < minimum or strictly_positive and value <= 0. or maximum is not None and value > maximum:
        raise HardwareConfigError(f"{key} is outside its supported range")
    return value


def _verified(section):
    value = section.get("verified", False)
    if not isinstance(value, bool):
        raise HardwareConfigError("verified must be a boolean")
    return value


def _table(value, name):
    if value is None:
        return ()
    if not isinstance(value, (list, tuple)):
        raise HardwareConfigError(f"{name} must contain [speed_m_s, pwm] measurements")
    rows = []
    for item in value:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise HardwareConfigError(f"{name} rows must be [speed_m_s, pwm]")
        speed, pwm = item
        if (isinstance(speed, bool) or not isinstance(speed, (int, float))
                or not math.isfinite(speed) or speed <= 0.
                or isinstance(pwm, bool) or not isinstance(pwm, int) or not 1 <= pwm <= 255):
            raise HardwareConfigError(f"{name} needs positive finite speeds and integer PWM 1..255")
        if rows and (speed <= rows[-1][0] or pwm <= rows[-1][1]):
            raise HardwareConfigError(f"{name} speed and PWM must increase strictly")
        rows.append((float(speed), pwm))
    return tuple(rows)


@dataclass(frozen=True)
class HardwareConfig:
    footprint_radius_m: float | None = None
    safety_margin_m: float | None = None
    max_linear_m_s: float | None = None
    max_angular_rad_s: float | None = None
    reaction_time_s: float | None = None
    minimum_deceleration_m_s2: float | None = None
    max_observation_age_s: float | None = None
    max_command_age_s: float | None = None
    control_period_s: float | None = None
    effective_track_width_m: float | None = None
    max_motor_pwm: int | None = None
    velocity_error_bound_m_s: float | None = None
    angular_error_bound_rad_s: float | None = None
    braking_distance_margin_m: float | None = None
    footprint_verified: bool = False
    dynamics_verified: bool = False
    camera_timing_verified: bool = False
    motor_calibration_verified: bool = False
    left_forward: tuple = ()
    left_reverse: tuple = ()
    right_forward: tuple = ()
    right_reverse: tuple = ()
    measurement_records: dict = field(default_factory=dict)
    measurements: dict = field(default_factory=dict)

    def __post_init__(self):
        # The public dataclass constructor must preserve the same invariants as
        # JSON loading; internal callers cannot accidentally bypass the gates.
        values = vars(self)
        for name in ("footprint_radius_m", "safety_margin_m", "minimum_deceleration_m_s2",
                     "effective_track_width_m", "braking_distance_margin_m"):
            _number(values, name, strictly_positive=True)
        for name, maximum in (("max_linear_m_s", .25), ("max_angular_rad_s", .8),
                              ("max_observation_age_s", .35), ("max_command_age_s", .15),
                              ("control_period_s", .1)):
            _number(values, name, maximum=maximum, strictly_positive=True)
        _number(values, "reaction_time_s", minimum=.5)
        for name in ("velocity_error_bound_m_s", "angular_error_bound_rad_s"):
            _number(values, name)
        for name in ("footprint_verified", "dynamics_verified", "camera_timing_verified", "motor_calibration_verified"):
            if not isinstance(getattr(self, name), bool):
                raise HardwareConfigError(f"{name} must be boolean")
        if self.max_motor_pwm is not None and (type(self.max_motor_pwm) is not int or not 1 <= self.max_motor_pwm <= 255):
            raise HardwareConfigError("max_motor_pwm must be an integer 1..255 or null")
        for name in ("left_forward", "left_reverse", "right_forward", "right_reverse"):
            table = _table(getattr(self, name), name)
            if self.max_motor_pwm is not None and any(pwm > self.max_motor_pwm for _, pwm in table):
                raise HardwareConfigError(f"{name} exceeds max_motor_pwm")
            object.__setattr__(self, name, table)
        if (self.control_period_s is not None and self.max_command_age_s is not None
                and self.control_period_s >= self.max_command_age_s):
            raise HardwareConfigError("control_period_s must be shorter than max_command_age_s")
        if not isinstance(self.measurement_records, dict) or not isinstance(self.measurements, dict):
            raise HardwareConfigError("measurement records and context must be objects")
        object.__setattr__(self, "measurement_records", dict(self.measurement_records))
        object.__setattr__(self, "measurements", dict(self.measurements))

    @classmethod
    def from_dict(cls, values: dict[str, Any]):
        if not isinstance(values, dict):
            raise HardwareConfigError("hardware profile must be an object")
        sections = {}
        for name in ("physical", "motor", "dynamics", "timing"):
            section = values.get(name, {})
            if not isinstance(section, dict):
                raise HardwareConfigError(f"{name} must be an object")
            sections[name] = section
        p, m, d, t = (sections[name] for name in ("physical", "motor", "dynamics", "timing"))
        metadata = values.get("measurements", {})
        if not isinstance(metadata, dict):
            raise HardwareConfigError("measurements must be an object")
        records = {}
        for name, section in sections.items():
            record = section.get("measurement_record", "")
            if not isinstance(record, str):
                raise HardwareConfigError(f"{name}.measurement_record must be text")
            records[name] = record.strip()
        pwm = m.get("max_motor_pwm", m.get("max_pwm"))
        if pwm is not None and (isinstance(pwm, bool) or not isinstance(pwm, int) or not 1 <= pwm <= 255):
            raise HardwareConfigError("max_motor_pwm must be an integer 1..255 or null")
        # Aliases keep the persisted profile readable alongside the core's
        # original names. There are no fallback numerical measurements.
        p = dict(p, footprint_radius_m=p.get("footprint_radius_m", p.get("robot_radius_m")))
        m = dict(m, effective_track_width_m=m.get("effective_track_width_m", m.get("effective_track_m")))
        d = dict(d,
            minimum_deceleration_m_s2=d.get("minimum_deceleration_m_s2", d.get("min_deceleration_m_s2")),
            velocity_error_bound_m_s=d.get("velocity_error_bound_m_s", d.get("speed_error_m_s")),
            angular_error_bound_rad_s=d.get("angular_error_bound_rad_s", d.get("yaw_error_rad_s")),
            braking_distance_margin_m=d.get("braking_distance_margin_m", d.get("stop_buffer_m")))
        result = cls(
            footprint_radius_m=_number(p, "footprint_radius_m", strictly_positive=True),
            safety_margin_m=_number(p, "safety_margin_m", strictly_positive=True),
            max_linear_m_s=_number(d, "max_linear_m_s", maximum=.25, strictly_positive=True),
            max_angular_rad_s=_number(d, "max_angular_rad_s", maximum=.8, strictly_positive=True),
            reaction_time_s=_number(d, "reaction_time_s", minimum=.5),
            minimum_deceleration_m_s2=_number(d, "minimum_deceleration_m_s2", strictly_positive=True),
            max_observation_age_s=_number(t, "max_observation_age_s", maximum=.35, strictly_positive=True),
            max_command_age_s=_number(t, "max_command_age_s", maximum=.15, strictly_positive=True),
            control_period_s=_number(t, "control_period_s", maximum=.1, strictly_positive=True),
            effective_track_width_m=_number(m, "effective_track_width_m", strictly_positive=True),
            max_motor_pwm=pwm,
            velocity_error_bound_m_s=_number(d, "velocity_error_bound_m_s"),
            angular_error_bound_rad_s=_number(d, "angular_error_bound_rad_s"),
            braking_distance_margin_m=_number(d, "braking_distance_margin_m", strictly_positive=True),
            footprint_verified=_verified(p), dynamics_verified=_verified(d),
            camera_timing_verified=_verified(t), motor_calibration_verified=_verified(m),
            left_forward=_table(m.get("left_forward"), "left_forward"),
            left_reverse=_table(m.get("left_reverse"), "left_reverse"),
            right_forward=_table(m.get("right_forward"), "right_forward"),
            right_reverse=_table(m.get("right_reverse"), "right_reverse"),
            measurement_records=records, measurements=dict(metadata))
        if (result.control_period_s is not None and result.max_command_age_s is not None
                and result.control_period_s >= result.max_command_age_s):
            raise HardwareConfigError("control_period_s must be shorter than max_command_age_s")
        if result.max_motor_pwm is not None:
            for name in ("left_forward", "left_reverse", "right_forward", "right_reverse"):
                if any(pwm > result.max_motor_pwm for _, pwm in getattr(result, name)):
                    raise HardwareConfigError(f"{name} exceeds max_motor_pwm")
        return result

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text()))

    def live_blockers(self):
        blockers = []
        for name, flag in (("physical", self.footprint_verified), ("motor", self.motor_calibration_verified),
                           ("dynamics", self.dynamics_verified), ("timing", self.camera_timing_verified)):
            if not flag:
                blockers.append(f"{name} calibration is not verified")
            if not self.measurement_records.get(name):
                blockers.append(f"{name} measurement_record is missing")
        for name in ("footprint_radius_m", "safety_margin_m", "max_linear_m_s", "max_angular_rad_s",
                     "reaction_time_s", "minimum_deceleration_m_s2", "max_observation_age_s",
                     "max_command_age_s", "control_period_s", "effective_track_width_m", "max_motor_pwm",
                     "velocity_error_bound_m_s", "angular_error_bound_rad_s", "braking_distance_margin_m"):
            if getattr(self, name) is None:
                blockers.append(f"{name} has not been measured/configured")
        for name in ("date", "surface", "battery", "load"):
            value = self.measurements.get(name)
            if not isinstance(value, str) or not value.strip():
                blockers.append(f"measurements.{name} is missing")
        if isinstance(self.measurements.get("date"), str):
            try:
                date.fromisoformat(self.measurements["date"])
            except ValueError:
                blockers.append("measurements.date must be an ISO date")
        required_wheel_speed = (None if self.max_linear_m_s is None or self.max_angular_rad_s is None
                                or self.effective_track_width_m is None else
                                self.max_linear_m_s + self.max_angular_rad_s * self.effective_track_width_m / 2.)
        for name in ("left_forward", "left_reverse", "right_forward", "right_reverse"):
            table = getattr(self, name)
            if len(table) < 2:
                blockers.append(f"{name} needs at least two measured nonzero speed/PWM points")
            elif required_wheel_speed is not None and table[-1][0] + 1e-9 < required_wheel_speed:
                blockers.append(f"{name} does not cover the configured wheel speed envelope")
        return blockers

    @staticmethod
    def _interpolate(speed, table, name):
        if abs(speed) < 1e-12:
            return 0
        target = abs(speed)
        if not table or target < table[0][0] - 1e-9 or target > table[-1][0] + 1e-9:
            raise HardwareConfigError(f"{name} requested speed is outside measured range")
        for measured_speed, pwm in table:
            if math.isclose(target, measured_speed, abs_tol=1e-9, rel_tol=0.):
                return pwm if speed > 0 else -pwm
        for (s0, p0), (s1, p1) in zip(table, table[1:]):
            if s0 < target < s1:
                pwm = round(p0 + (target - s0) / (s1 - s0) * (p1 - p0))
                return pwm if speed > 0 else -pwm
        raise HardwareConfigError(f"{name} could not interpolate measured range")

    def pwm_for_velocity(self, linear, angular):
        """Map metres/sec and CCW radians/sec to *physical* left/right PWM.

        No extrapolation through motor deadband or beyond a measured table is
        allowed. The transport alone maps physical sides to installed wiring.
        """
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
               for x in (linear, angular)):
            raise HardwareConfigError("velocity command must be finite")
        if linear == 0. and angular == 0.:
            return (0, 0)
        if (not self.motor_calibration_verified or self.effective_track_width_m is None
                or self.max_linear_m_s is None or self.max_angular_rad_s is None or self.max_motor_pwm is None):
            raise HardwareConfigError("motor calibration and velocity limits are required")
        if abs(linear) > self.max_linear_m_s + 1e-9 or abs(angular) > self.max_angular_rad_s + 1e-9:
            raise HardwareConfigError("requested velocity exceeds configured limits")
        left = linear - angular * self.effective_track_width_m / 2.
        right = linear + angular * self.effective_track_width_m / 2.
        results = []
        for name, speed in (("left", left), ("right", right)):
            direction = "forward" if speed >= 0. else "reverse"
            pwm = self._interpolate(speed, getattr(self, f"{name}_{direction}"), f"{name}_{direction}")
            if abs(pwm) > self.max_motor_pwm:
                raise HardwareConfigError("interpolated PWM exceeds configured limit")
            results.append(pwm)
        return tuple(results)

    # Names used by the frozen controller and earlier roadmap.
    robot_radius_m = property(lambda self: self.footprint_radius_m)
    min_deceleration_m_s2 = property(lambda self: self.minimum_deceleration_m_s2)
    effective_track_m = property(lambda self: self.effective_track_width_m)
    speed_error_m_s = property(lambda self: self.velocity_error_bound_m_s)
    yaw_error_rad_s = property(lambda self: self.angular_error_bound_rad_s)
    stop_buffer_m = property(lambda self: self.braking_distance_margin_m)
    to_pwm = pwm_for_velocity

    def to_dict(self):
        return {
            "physical": {"verified": self.footprint_verified, "measurement_record": self.measurement_records.get("physical", ""),
                         "footprint_radius_m": self.footprint_radius_m, "safety_margin_m": self.safety_margin_m},
            "motor": {"verified": self.motor_calibration_verified, "measurement_record": self.measurement_records.get("motor", ""),
                      "effective_track_width_m": self.effective_track_width_m, "max_motor_pwm": self.max_motor_pwm,
                      **{name: [list(row) for row in getattr(self, name)] for name in
                         ("left_forward", "left_reverse", "right_forward", "right_reverse")}},
            "dynamics": {"verified": self.dynamics_verified, "measurement_record": self.measurement_records.get("dynamics", ""),
                         **{name: getattr(self, name) for name in ("max_linear_m_s", "max_angular_rad_s", "reaction_time_s",
                            "minimum_deceleration_m_s2", "velocity_error_bound_m_s", "angular_error_bound_rad_s", "braking_distance_margin_m")}},
            "timing": {"verified": self.camera_timing_verified, "measurement_record": self.measurement_records.get("timing", ""),
                       "max_observation_age_s": self.max_observation_age_s, "max_command_age_s": self.max_command_age_s,
                       "control_period_s": self.control_period_s},
            "measurements": dict(self.measurements)}
