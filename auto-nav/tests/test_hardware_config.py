"""Hardware profile limits and measured-domain motor conversion, no networking."""
import copy
import json
import math

import pytest

from rover_nav.hardware_config import HardwareConfig, HardwareConfigError


def profile():
    # Synthetic unit-test measurements. These are never a shipped rover profile.
    return {
        "physical": {"verified": True, "measurement_record": "synthetic-test", "footprint_radius_m": .27, "safety_margin_m": .12},
        "motor": {"verified": True, "measurement_record": "synthetic-test", "effective_track_width_m": .3,
                  "max_motor_pwm": 200, "left_forward": [[.01, 50], [.3, 200]], "left_reverse": [[.01, 50], [.3, 200]],
                  "right_forward": [[.01, 50], [.3, 200]], "right_reverse": [[.01, 50], [.3, 200]]},
        "dynamics": {"verified": True, "measurement_record": "synthetic-test", "max_linear_m_s": .15,
                     "max_angular_rad_s": .5, "reaction_time_s": .6, "minimum_deceleration_m_s2": .2,
                     "velocity_error_bound_m_s": .02, "angular_error_bound_rad_s": .05, "braking_distance_margin_m": .08},
        "timing": {"verified": True, "measurement_record": "synthetic-test", "max_observation_age_s": .2,
                   "max_command_age_s": .15, "control_period_s": .05},
        "measurements": {"date": "2026-10-28", "surface": "synthetic", "battery": "synthetic", "load": "synthetic"}}


def test_empty_profile_is_shadow_only_and_has_no_fabricated_values():
    config = HardwareConfig.from_dict({})
    assert config.footprint_radius_m is None
    assert config.reaction_time_s is None
    assert config.max_motor_pwm is None
    assert config.live_blockers()
    assert config.pwm_for_velocity(0., 0.) == (0, 0)
    with pytest.raises(HardwareConfigError, match="calibration"):
        config.pwm_for_velocity(.1, 0.)


def test_measured_profile_roundtrip_and_disk_load(tmp_path):
    config = HardwareConfig.from_dict(profile())
    assert config.live_blockers() == []
    assert config.robot_radius_m == .27
    assert config.min_deceleration_m_s2 == .2
    target = tmp_path / "measured.json"
    target.write_text(json.dumps(config.to_dict()))
    assert HardwareConfig.load(target) == config


@pytest.mark.parametrize("section,key,value", [
    ("physical", "footprint_radius_m", 0), ("physical", "safety_margin_m", -1),
    ("physical", "footprint_radius_m", True), ("physical", "safety_margin_m", "0.1"),
    ("dynamics", "max_linear_m_s", .251), ("dynamics", "max_angular_rad_s", .81),
    ("dynamics", "reaction_time_s", .499), ("dynamics", "minimum_deceleration_m_s2", 0),
    ("dynamics", "velocity_error_bound_m_s", -1), ("dynamics", "angular_error_bound_rad_s", math.nan),
    ("dynamics", "braking_distance_margin_m", math.inf),
    ("timing", "max_observation_age_s", .351), ("timing", "max_command_age_s", .151),
    ("timing", "control_period_s", .101), ("motor", "effective_track_width_m", 0),
    ("motor", "max_motor_pwm", 256), ("motor", "max_motor_pwm", True),
    ("motor", "max_motor_pwm", 200.0), ("motor", "verified", "true"),
])
def test_invalid_limits_are_rejected_even_for_shadow(section, key, value):
    values = profile()
    values[section][key] = value
    with pytest.raises(HardwareConfigError):
        HardwareConfig.from_dict(values)


@pytest.mark.parametrize("section", ["physical", "motor", "dynamics", "timing"])
def test_each_measurement_needs_explicit_verification_and_record(section):
    values = profile()
    values[section]["verified"] = False
    values[section]["measurement_record"] = ""
    blockers = HardwareConfig.from_dict(values).live_blockers()
    assert f"{section} calibration is not verified" in blockers
    assert f"{section} measurement_record is missing" in blockers


def test_context_metadata_is_required_and_date_checked():
    values = profile()
    values["measurements"] = {"date": "yesterday"}
    blockers = HardwareConfig.from_dict(values).live_blockers()
    assert "measurements.surface is missing" in blockers
    assert "measurements.date must be an ISO date" in blockers


@pytest.mark.parametrize("rows", [
    [[.1, 100], [.1, 150]], [[.2, 150], [.1, 100]], [[.1, 100], [.2, 100]],
    [[0, 0]], [[-.1, 100]], [[.1, 0]], [[.1, 256]], [[.1, 100.0]], [[True, 100]], [[.1]], "not a table",
])
def test_motor_tables_are_strict_monotonic_nonzero_measurements(rows):
    values = profile()
    values["motor"]["left_forward"] = rows
    with pytest.raises(HardwareConfigError):
        HardwareConfig.from_dict(values)


def test_motor_speed_envelope_and_pwm_cap_are_checked():
    values = profile()
    values["motor"]["left_reverse"] = [[.01, 50], [.1, 100]]
    assert "left_reverse does not cover the configured wheel speed envelope" in HardwareConfig.from_dict(values).live_blockers()
    values = profile()
    values["motor"]["max_motor_pwm"] = 199
    with pytest.raises(HardwareConfigError, match="exceeds"):
        HardwareConfig.from_dict(values)


def test_pwm_mapping_uses_physical_sides_and_signed_rotation():
    config = HardwareConfig.from_dict(profile())
    forward = config.pwm_for_velocity(.1, 0.)
    assert forward[0] == forward[1] > 0
    reverse = config.pwm_for_velocity(-.1, 0.)
    assert reverse == tuple(-x for x in forward)
    left, right = config.pwm_for_velocity(0., .4)
    assert left < 0 < right and left == -right
    assert config.pwm_for_velocity(0., -.4) == (right, left)
    arc = config.pwm_for_velocity(.1, .2)
    assert 0 < arc[0] < arc[1]


def test_no_deadband_or_high_speed_extrapolation_and_no_command_clipping():
    config = HardwareConfig.from_dict(profile())
    with pytest.raises(HardwareConfigError, match="outside measured"):
        config.pwm_for_velocity(.005, 0.)
    with pytest.raises(HardwareConfigError, match="exceeds configured"):
        config.pwm_for_velocity(.151, 0.)
    with pytest.raises(HardwareConfigError, match="finite"):
        config.pwm_for_velocity(float("nan"), 0.)
    assert config.pwm_for_velocity(.01, 0.) == (50, 50)


def test_shorter_command_age_requires_faster_output_loop():
    values = profile()
    values["timing"]["max_command_age_s"] = .05
    with pytest.raises(HardwareConfigError, match="shorter"):
        HardwareConfig.from_dict(values)


def test_profile_copies_mutable_inputs():
    values = profile()
    original = copy.deepcopy(values)
    config = HardwareConfig.from_dict(values)
    values["motor"]["left_forward"][0][1] = 255
    values["measurements"]["load"] = "modified"
    assert config.left_forward[0][1] == original["motor"]["left_forward"][0][1]
    assert config.measurements["load"] == "synthetic"


@pytest.mark.parametrize("values", [{"max_linear_m_s": .5}, {"reaction_time_s": .35},
    {"max_command_age_s": math.nan}, {"footprint_verified": "true"}, {"left_forward": [[.2, 100], [.1, 120]]}])
def test_direct_dataclass_constructor_cannot_bypass_profile_validation(values):
    with pytest.raises(HardwareConfigError):
        HardwareConfig(**values)
