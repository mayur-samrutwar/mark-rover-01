"""Synthetic geometry tests only; these do not verify a physical rover mount."""
from copy import deepcopy
import json

import cv2
import numpy as np
import pytest

import rover_nav.hardware_floor_calibration as floor
from rover_nav.hardware_calibration import CalibrationError, mount_transform
from rover_nav.hardware_floor_calibration import image_intrinsics, solve_floor_mount


def fixture():
    profile = {"source_width": 720, "source_height": 1280,
               "K": [[649.455, 0., 357.69], [0., 642.908, 639.379], [0., 0., 1.]],
               "distortion": [-.043, .195, -.0015, .0008, -.256],
               "verified": False, "T_base_camera": None}
    geometry = {"near_x_m": .40, "far_x_m": .70, "half_width_m": .10,
                "max_camera_offset_x_m": .25, "max_camera_offset_y_m": .20}
    transform = mount_transform(height_m=.22, forward_m=.03, left_m=.005,
                                pitch_down_deg=25., yaw_deg=3., roll_deg=2.)
    return profile, geometry, transform


def project(profile, geometry, transform, image_size=(360, 640)):
    K, distortion = image_intrinsics(profile, image_size)
    near, far, half = (geometry[name] for name in ("near_x_m", "far_x_m", "half_width_m"))
    objects = np.array([[near, half, 0.], [near, -half, 0.], [far, -half, 0.],
                        [far, half, 0.], [(near + far) / 2., 0., 0.]])
    rotation = transform[:3, :3].T
    translation = -rotation @ transform[:3, 3]
    rvec, _ = cv2.Rodrigues(rotation)
    return cv2.projectPoints(objects, rvec, translation, K, distortion)[0].reshape(5, 2)


def transform_with_pitch(pitch_down_deg):
    pitch = np.deg2rad(pitch_down_deg)
    rotation_y = np.array([[np.cos(pitch), 0., np.sin(pitch)], [0., 1., 0.],
                           [-np.sin(pitch), 0., np.cos(pitch)]])
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    transform = np.eye(4)
    transform[:3, :3] = rotation_y @ optical
    transform[:3, 3] = [.03, .005, .22]
    return transform


@pytest.mark.parametrize("image_size", [(360, 640), (720, 1280)])
def test_exact_distorted_raw_pixels_recover_actual_mount_without_verifying(image_size):
    profile, geometry, transform = fixture()
    original_profile, original_geometry = deepcopy(profile), deepcopy(geometry)
    pixels = project(profile, geometry, transform, image_size)
    result = solve_floor_mount(profile, pixels, geometry, image_size, measured_height_m=.22)
    np.testing.assert_allclose(result["T_base_camera"], transform, atol=1e-7, rtol=0.)
    assert result["verified"] is False
    report = result["floor_mount_report"]
    assert report["quality_blockers"] == []
    assert report["training_rms_px"] < 1e-7 and report["training_max_px"] < 1e-7
    assert report["heldout_error_px"] < 1e-7
    assert report["measured_height_difference_m"] < 1e-7
    assert report["heldout_used_for_pose_fit_or_selection"] is False
    assert report["training_point_indices"] == [0, 1, 2, 3] and report["heldout_point_index"] == 4
    assert len(report["candidates"]) == 2
    assert report["pitch_down_deg"] == pytest.approx(25.)
    assert report["floor_patch_visibility"]["seed_pixel_count"] >= 100
    assert report["floor_patch_visibility"]["geometry_only"] is True
    assert report["floor_patch_visibility"]["floor_appearance_or_texture_validated"] is False
    assert profile == original_profile and geometry == original_geometry
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("pitch_down_deg", [0., -5.])
def test_horizontal_or_slightly_upward_axis_can_observe_required_floor_patch(pitch_down_deg):
    profile, geometry, _ = fixture()
    transform = transform_with_pitch(pitch_down_deg)
    pixels = project(profile, geometry, transform)
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    np.testing.assert_allclose(result["T_base_camera"], transform, atol=1e-7)
    report = result["floor_mount_report"]
    assert report["quality_blockers"] == []
    assert report["pitch_down_deg"] == pytest.approx(pitch_down_deg, abs=1e-5)
    assert report["floor_patch_visibility"]["seed_visible"] is True
    assert report["floor_patch_visibility"]["seed_pixel_count"] >= 100
    assert result["verified"] is False


def test_strong_upward_view_with_distant_visible_markers_still_rejects_missing_near_floor_patch():
    profile, geometry, _ = fixture()
    # This wide vertical view can see the measured rectangle beyond .95 m,
    # but its lowest image ray cannot see the required .3--.7 m seed region.
    profile["K"][1][1] = 300.
    profile["distortion"] = [0.] * 5
    geometry.update(near_x_m=.95, far_x_m=1.30)
    transform = transform_with_pitch(-50.)
    pixels = project(profile, geometry, transform)
    assert np.all(pixels >= 0.) and np.all(pixels[:, 0] < 360.) and np.all(pixels[:, 1] < 640.)
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is None
    report = result["floor_mount_report"]
    assert report["pitch_down_deg"] == pytest.approx(-50., abs=1e-7)
    assert report["floor_patch_visibility"]["seed_pixel_count"] == 0
    assert report["floor_patch_visibility"]["downward_ray_pixels"] > 0
    assert "required_initial_floor_patch_not_geometrically_visible" in report["quality_blockers"]


def test_noisy_training_corner_exceeds_reprojection_limit_even_with_complete_rectangle():
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    pixels[0] += [8., -6.]
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    report = result["floor_mount_report"]
    assert result["T_base_camera"] is None
    assert report["training_max_px"] > 1.
    assert "training_marker_error_exceeds_1_px" in report["quality_blockers"]


def test_image_intrinsics_uses_full_frame_pixel_centres_and_leaves_distortion_once():
    profile, _, _ = fixture()
    K, distortion = image_intrinsics(profile, (360, 640))
    assert K[0, 0] == profile["K"][0][0] / 2.
    assert K[1, 1] == profile["K"][1][1] / 2.
    assert K[0, 2] == (profile["K"][0][2] + .5) / 2. - .5
    assert K[1, 2] == (profile["K"][1][2] + .5) / 2. - .5
    np.testing.assert_array_equal(distortion, profile["distortion"])
    K[0, 0] = 1.
    distortion[0] = 0.
    assert profile["K"][0][0] != 1. and profile["distortion"][0] != 0.


def test_fifth_marker_is_an_independent_check_and_cannot_pull_pose_towards_it():
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    baseline = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    pixels[4] += [14., -8.]
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is None and result["verified"] is False
    np.testing.assert_array_equal(result["candidate_T_base_camera"], baseline["candidate_T_base_camera"])
    assert result["floor_mount_report"]["training_rms_px"] == baseline["floor_mount_report"]["training_rms_px"]
    assert result["floor_mount_report"]["heldout_error_px"] > 10.
    assert "independent_heldout_error_exceeds_2_px" in result["floor_mount_report"]["quality_blockers"]


def test_pnp_and_refinement_never_receive_fifth_point(monkeypatch):
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    original_solve, original_refine = cv2.solvePnPGeneric, cv2.solvePnPRefineLM
    calls = []

    def checked_solve(objects, image_points, *args, **kwargs):
        calls.append("IPPE")
        assert objects.shape == (4, 3) and image_points.shape == (4, 2)
        np.testing.assert_array_equal(image_points, pixels[:4])
        return original_solve(objects, image_points, *args, **kwargs)

    def checked_refine(objects, image_points, *args, **kwargs):
        calls.append("LM")
        assert objects.shape == (4, 3) and image_points.shape == (4, 2)
        return original_refine(objects, image_points, *args, **kwargs)

    monkeypatch.setattr(cv2, "solvePnPGeneric", checked_solve)
    monkeypatch.setattr(cv2, "solvePnPRefineLM", checked_refine)
    assert solve_floor_mount(profile, pixels, geometry, (360, 640), .22)["T_base_camera"] is not None
    assert calls == ["IPPE", "LM", "LM"]


def test_independent_height_is_only_a_crosscheck_and_is_not_forced_into_fit():
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    accepted = solve_floor_mount(profile, pixels, geometry, (360, 640), .24)
    np.testing.assert_allclose(accepted["T_base_camera"], transform, atol=1e-7)
    assert accepted["floor_mount_report"]["measured_height_difference_m"] == pytest.approx(.02)
    blocked = solve_floor_mount(profile, pixels, geometry, (360, 640), .40)
    assert blocked["T_base_camera"] is None
    assert blocked["floor_mount_report"]["measured_height_difference_m"] == pytest.approx(.18)
    assert "no_physically_plausible_camera_pose" in blocked["floor_mount_report"]["quality_blockers"]
    assert any("estimated_height_disagrees_with_independent_measurement" in candidate["physical_blockers"]
               for candidate in blocked["floor_mount_report"]["candidates"])
    missing = solve_floor_mount(profile, pixels, geometry, (360, 640))
    assert missing["verified"] is False
    assert missing["floor_mount_report"]["measured_height_difference_m"] is None
    assert "independent_camera_height_crosscheck_missing" in missing["floor_mount_report"]["warnings"]


@pytest.mark.parametrize("field,value", [("near_x_m", None), ("far_x_m", .42),
                                        ("half_width_m", 0.), ("half_width_m", True),
                                        ("max_camera_offset_x_m", None),
                                        ("max_camera_offset_y_m", float("inf")),
                                        ("height_tolerance_m", .5)])
def test_invalid_or_implicit_measured_geometry_is_rejected(field, value):
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    geometry[field] = value
    with pytest.raises(CalibrationError):
        solve_floor_mount(profile, pixels, geometry, (360, 640), .22)


@pytest.mark.parametrize("size", [(640, 360), (361, 640), (1440, 2560), (True, 640), (360,), None])
def test_crop_rotation_upscale_or_invalid_image_size_is_rejected(size):
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    with pytest.raises(CalibrationError):
        solve_floor_mount(profile, pixels, geometry, size, .22)


@pytest.mark.parametrize("order", [[1, 0, 2, 3, 4], [0, 2, 1, 3, 4], [3, 2, 1, 0, 4]])
def test_wrong_or_mirrored_corner_order_is_rejected(order):
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)[order]
    with pytest.raises(CalibrationError, match="Marker order"):
        solve_floor_mount(profile, pixels, geometry, (360, 640), .22)


def test_wrong_near_and_far_labels_cannot_create_an_accepted_backwards_mount():
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)[[2, 3, 0, 1, 4]]
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is None
    assert "no_physically_plausible_camera_pose" in result["floor_mount_report"]["quality_blockers"]


@pytest.mark.parametrize("pixels", [
    [[100., 300.], [200., 300.], [200., 299.9], [100., 299.9], [150., 299.95]],
    [[100., 300.], [102., 300.], [102., 298.], [100., 298.], [101., 299.]],
    [[100., 300.], [200., 300.], [200., 300.], [100., 300.], [150., 300.]]])
def test_tiny_or_near_degenerate_marker_quadrilateral_is_rejected(pixels):
    profile, geometry, _ = fixture()
    with pytest.raises(CalibrationError, match="nearly collinear|Marker order"):
        solve_floor_mount(profile, pixels, geometry, (360, 640), .22)


@pytest.mark.parametrize("value", [None, [[1., 2.]], [[float("nan"), 2.]] * 5,
                                  [[True, 2.]] * 5, [["1", 2.]] * 5,
                                  [[-1., 2.]] * 5, [[360., 2.]] * 5])
def test_malformed_nonfinite_or_out_of_image_clicks_are_rejected(value):
    profile, geometry, _ = fixture()
    with pytest.raises(CalibrationError):
        solve_floor_mount(profile, value, geometry, (360, 640), .22)


def test_explicit_footprint_envelope_is_enforced():
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    geometry["max_camera_offset_x_m"] = .01
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is None and result["verified"] is False
    assert any("camera_forward_offset_outside_supplied_envelope" in item["physical_blockers"]
               for item in result["floor_mount_report"]["candidates"])


def test_ambiguous_distinct_physical_candidates_are_reported_and_blocked(monkeypatch):
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    evaluated = floor._candidate

    def two_close_candidates(index, *args, **kwargs):
        candidate = evaluated(index, *args, **kwargs)
        # Isolate the ambiguity gate from OpenCV's convergence to a single
        # minimum: both are physically admitted, numerically similar solutions.
        candidate["physical_blockers"] = []
        candidate["physically_plausible"] = True
        candidate["training_rms_px"] = .20 + .05 * index
        candidate["training_max_px"] = .30 + .05 * index
        candidate["heldout_error_px"] = .5
        return candidate

    monkeypatch.setattr(floor, "_candidate", two_close_candidates)
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is None
    report = result["floor_mount_report"]
    assert "ambiguous_planar_pose_candidates" in report["quality_blockers"]
    assert report["ambiguous_alternative_indices"] == [1]
    assert report["selected_candidate_index"] is None and len(report["candidates"]) == 2


def test_duplicate_converged_candidates_do_not_invent_ambiguity(monkeypatch):
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    original = cv2.solvePnPGeneric

    def duplicated(*args, **kwargs):
        solved = original(*args, **kwargs)
        return 2, (solved[1][0], solved[1][0].copy()), (solved[2][0], solved[2][0].copy())

    monkeypatch.setattr(cv2, "solvePnPGeneric", duplicated)
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is not None
    assert result["floor_mount_report"]["ambiguous_alternative_indices"] == []


def test_existing_bad_intrinsic_quality_cannot_be_excused_by_good_floor_fit():
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    profile["intrinsics_report"] = {"quality_blockers": ["poor_heldout_lens_fit"]}
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is None
    assert "intrinsics_report_contains_quality_blockers" in result["floor_mount_report"]["quality_blockers"]


def test_no_pnp_candidates_is_a_blocked_finite_report(monkeypatch):
    profile, geometry, transform = fixture()
    pixels = project(profile, geometry, transform)
    monkeypatch.setattr(cv2, "solvePnPGeneric", lambda *args, **kwargs: (0, (), ()))
    result = solve_floor_mount(profile, pixels, geometry, (360, 640), .22)
    assert result["T_base_camera"] is None and result["candidate_T_base_camera"] is None
    assert "no_planar_pose_candidates" in result["floor_mount_report"]["quality_blockers"]
    json.dumps(result, allow_nan=False)
