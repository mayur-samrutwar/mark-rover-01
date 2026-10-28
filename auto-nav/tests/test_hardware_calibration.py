"""Offline synthetic target geometry; never a physical camera calibration."""
import io
import json
import math

import cv2
import numpy as np
from PIL import Image
import pytest

import rover_nav.hardware_calibration as calibration
from rover_nav.hardware_calibration import (
    CalibrationError, calibrate_intrinsics, detect_chessboard, main,
    mount_transform, normalize_capture_settings, output_to_source_K,
)
from rover_nav.hardware_camera import CameraCalibration


def synthetic_views(count=20, *, fixed_centre=False, fixed_size=False, flat=False):
    K = np.array([[600., 0., 319.5], [0., 620., 239.5], [0., 0., 1.]])
    distortion = np.array([-.05, .01, .001, -.0005, .005])
    objects = np.zeros((9 * 6, 3), np.float32)
    objects[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * .025
    centre = objects.mean(axis=0)
    result = []
    for index in range(count):
        a, b, roll = (.35 + .09 * (index % 4), (-1 if index % 2 else 1) * (.25 + .07 * (index % 3)),
                      -.18 + .06 * (index % 7))
        Rx = np.array([[1., 0., 0.], [0., math.cos(a), -math.sin(a)], [0., math.sin(a), math.cos(a)]])
        Ry = np.array([[math.cos(b), 0., math.sin(b)], [0., 1., 0.], [-math.sin(b), 0., math.cos(b)]])
        Rz = np.array([[math.cos(roll), -math.sin(roll), 0.], [math.sin(roll), math.cos(roll), 0.], [0., 0., 1.]])
        rotation = np.eye(3) if flat else Ry @ Rx @ Rz
        z = .8 if fixed_size else [.55, .75, 1., .6, .85][index % 5]
        u = 319.5 if fixed_centre else [180., 270., 370., 460.][index % 4]
        v = 239.5 if fixed_centre else [140., 230., 330.][(index // 4) % 3]
        tvec = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]) - rotation @ centre
        rvec, _ = cv2.Rodrigues(rotation)
        projected, _ = cv2.projectPoints(objects, rvec, tvec, K, distortion)
        result.append(projected.reshape(-1, 2).astype(np.float32))
    return K, distortion, result


def capture_folder(tmp_path, count=20):
    metadata = {"source_width": 1280, "source_height": 960, "width": 640, "height": 480,
                "rotation_degrees": 0, "mirrored": False, "crop": "none",
                "settings": {"device_id": "stable-subset"},
                "source_settings": {"device_id": "OFFLINE-SYNTHETIC-NOT-HARDWARE",
                                    "width": 1280, "height": 960, "zoom": 1., "focus_mode": "manual"}}
    for index in range(count):
        image = Image.new("RGB", (640, 480), (index * 7 % 255, index * 11 % 255, index * 13 % 255))
        image.save(tmp_path / f"frame-{index:03}.jpg", quality=90)
        (tmp_path / f"frame-{index:03}.json").write_text(json.dumps(metadata))
    return tmp_path


def run_with_corners(monkeypatch, folder, corners):
    stream = iter(corners)
    monkeypatch.setattr(calibration, "detect_chessboard", lambda *_: next(stream))
    return calibrate_intrinsics(folder, columns=9, rows=6, square_size_m=.025,
                                camera_id="OFFLINE-SYNTHETIC-NOT-HARDWARE")


def test_real_chessboard_detector_uses_saved_pixel_corner_geometry():
    board = np.full((280, 400, 3), 255, np.uint8)
    for row in range(7):
        for column in range(10):
            if (row + column) % 2 == 0:
                board[row * 40:(row + 1) * 40, column * 40:(column + 1) * 40] = 0
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [399, 0], [399, 279], [0, 279]]),
                                   np.float32([[100, 90], [510, 65], [530, 390], [80, 370]]))
    image = cv2.warpPerspective(board, H, (640, 480), borderValue=(255, 255, 255))
    out = io.BytesIO()
    Image.fromarray(image).save(out, format="JPEG", quality=90)
    saved = np.asarray(Image.open(io.BytesIO(out.getvalue())).convert("RGB"))
    detected = detect_chessboard(saved, 9, 6)
    assert detected is not None and detected.shape == (54, 2)
    points = np.mgrid[1:10, 1:7].T.reshape(-1, 2).astype(np.float32) * 40 - .5
    expected = cv2.perspectiveTransform(points.reshape(-1, 1, 2), H).reshape(-1, 2)
    error = min(np.sqrt(np.mean(np.sum((detected - ordering) ** 2, axis=1)))
                for ordering in (expected, expected[::-1]))
    assert error < 1.


def test_diverse_synthetic_fit_recovers_intrinsics_and_keeps_all_authority_unverified(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    K, distortion, corners = synthetic_views()
    result = run_with_corners(monkeypatch, folder, corners)
    expected = output_to_source_K(K, source_width=1280, source_height=960, width=640, height=480)
    np.testing.assert_allclose(result["K"], expected, rtol=.002, atol=.2)
    np.testing.assert_allclose(result["distortion"], distortion, atol=.01)
    report = result["intrinsics_report"]
    assert report["training_view_count"] == 16 and report["heldout_view_count"] == 4
    assert report["training_rms_px"] < .01 and report["heldout_rms_px"] < .01
    assert report["quality_blockers"] == []
    assert result["settings"]["device_id"] == "OFFLINE-SYNTHETIC-NOT-HARDWARE"
    assert result["verified"] is False and result["T_base_camera"] is None
    assert result["timing"] == {"max_pipeline_age_s": None, "validated": False}
    with pytest.raises((ValueError, TypeError)):
        CameraCalibration.from_dict(result)


@pytest.mark.parametrize("wrapped", [False, True])
def test_recorded_display_measurement_matches_explicit_cli_scale(monkeypatch, tmp_path, wrapped):
    folder = capture_folder(tmp_path)
    for path in folder.glob("*.json"):
        metadata = json.loads(path.read_text())
        metadata["calibration_target"] = {"columns": 9, "rows": 6, "square_size_cm": 2.5,
                                          "display_width_css_px": 500.,
                                          "display_height_css_px": 350.}
        path.write_text(json.dumps({"metadata": metadata} if wrapped else metadata))
    _, _, corners = synthetic_views()
    result = run_with_corners(monkeypatch, folder, corners)
    assert result["intrinsics_report"]["target"]["square_size_m"] == .025
    assert result["verified"] is False


@pytest.mark.parametrize("target", [None, {},
    {"columns": True, "rows": 6, "square_size_cm": 2.5},
    {"columns": 10, "rows": 6, "square_size_cm": 2.5},
    {"columns": 9, "rows": 5, "square_size_cm": 2.5},
    {"columns": 9, "rows": 6, "square_size_cm": True},
    {"columns": 9, "rows": 6, "square_size_cm": "2.5"},
    {"columns": 9, "rows": 6, "square_size_cm": 0.},
    {"columns": 9, "rows": 6, "square_size_cm": float("nan")},
    {"columns": 9, "rows": 6, "square_size_cm": 3.}])
def test_invalid_or_cli_mismatched_recorded_target_is_rejected_before_fitting(monkeypatch, tmp_path, target):
    folder = capture_folder(tmp_path)
    path = folder / "frame-000.json"
    metadata = json.loads(path.read_text())
    metadata["calibration_target"] = target
    path.write_text(json.dumps(metadata))
    monkeypatch.setattr(calibration, "detect_chessboard", lambda *_: pytest.fail("Invalid target reached fitting"))
    with pytest.raises(CalibrationError, match="calibration_target"):
        calibrate_intrinsics(folder, columns=9, rows=6, square_size_m=.025, camera_id="fixture")


def test_mixed_recorded_square_sizes_cannot_be_fitted_together(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    for index, path in enumerate(sorted(folder.glob("*.json"))):
        metadata = json.loads(path.read_text())
        metadata["calibration_target"] = {"columns": 9, "rows": 6,
                                          "square_size_cm": 3. if index == 5 else 2.5}
        path.write_text(json.dumps(metadata))
    _, _, corners = synthetic_views()
    with pytest.raises(CalibrationError, match="square size.*frame-005"):
        run_with_corners(monkeypatch, folder, corners)


def test_heldout_noise_is_reported_without_contaminating_fitted_K(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    _, _, corners = synthetic_views()
    baseline = run_with_corners(monkeypatch, folder, corners)
    noisy = [points.copy() for points in corners]
    rng = np.random.default_rng(41)
    for index in range(4, len(noisy), 5):
        noisy[index] += rng.normal(0., 4., noisy[index].shape)
    result = run_with_corners(monkeypatch, folder, noisy)
    np.testing.assert_allclose(result["K"], baseline["K"], rtol=0., atol=1e-6)
    assert result["intrinsics_report"]["heldout_rms_px"] > 1.5
    assert result["intrinsics_report"]["quality_blockers"]
    assert result["verified"] is False


def test_pixel_centre_conversion_roundtrip_has_no_half_pixel_error():
    K = np.array([[600., 0., 319.5], [0., 620., 239.5], [0., 0., 1.]])
    converted = output_to_source_K(K, source_width=1280, source_height=960, width=640, height=480)
    np.testing.assert_allclose(converted, [[1200., 0., 639.5], [0., 1240., 479.5], [0., 0., 1.]])
    copied = output_to_source_K(K, source_width=640, source_height=480, width=640, height=480)
    np.testing.assert_array_equal(copied, K)
    with pytest.raises(CalibrationError, match="aspect"):
        output_to_source_K(K, source_width=1280, source_height=960, width=640, height=400)


@pytest.mark.parametrize("change", [{"crop": "centre"}, {"mirrored": True}, {"rotation_degrees": 90},
                                   {"width": 320}, {"source_width": 1920}])
def test_capture_geometry_or_processing_change_is_rejected(monkeypatch, tmp_path, change):
    folder = capture_folder(tmp_path)
    metadata_path = folder / "frame-005.json"
    metadata = json.loads(metadata_path.read_text())
    metadata.update(change)
    metadata_path.write_text(json.dumps(metadata))
    _, _, corners = synthetic_views()
    with pytest.raises(CalibrationError):
        run_with_corners(monkeypatch, folder, corners)


def test_source_settings_change_cannot_mix_lens_modes(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    metadata_path = folder / "frame-005.json"
    metadata = json.loads(metadata_path.read_text())
    metadata["source_settings"]["zoom"] = 2.
    metadata_path.write_text(json.dumps(metadata))
    _, _, corners = synthetic_views()
    with pytest.raises(CalibrationError, match="settings changed"):
        run_with_corners(monkeypatch, folder, corners)


def test_continuous_illumination_values_can_vary_without_unpinning_optics(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    for index, path in enumerate(sorted(folder.glob("*.json"))):
        metadata = json.loads(path.read_text())
        metadata["source_settings"].update(
            exposure_mode="continuous", exposure_time=100. + index * 5.,
            exposure_compensation=-1. + index * .1,
            white_balance_mode="continuous", color_temperature=4000. + index * 40.,
            focus_mode="continuous", focus_distance=.5, frame_rate=30., stabilization_mode="off")
        path.write_text(json.dumps(metadata))
    _, _, corners = synthetic_views()
    result = run_with_corners(monkeypatch, folder, corners)
    settings = result["settings"]
    assert settings["exposure_mode"] == settings["white_balance_mode"] == "continuous"
    assert settings["focus_mode"] == "continuous" and settings["focus_distance"] == .5
    assert settings["frame_rate"] == 30. and settings["stabilization_mode"] == "off"
    assert all(key not in settings for key in ("exposure_time", "exposure_compensation", "color_temperature"))
    assert result["verified"] is False


@pytest.mark.parametrize("field,value", [("focus_distance", .6), ("focus_mode", "manual"), ("zoom", 2.),
                                        ("exposure_mode", "manual"), ("white_balance_mode", "manual"),
                                        ("stabilization_mode", "on"), ("frame_rate", 60.)])
def test_automatic_illumination_exception_keeps_optics_modes_and_processing_strict(monkeypatch, tmp_path, field, value):
    folder = capture_folder(tmp_path)
    for path in sorted(folder.glob("*.json")):
        metadata = json.loads(path.read_text())
        metadata["source_settings"].update(exposure_mode="continuous", exposure_time=100.,
            white_balance_mode="continuous", color_temperature=4000.,
            focus_mode="continuous", focus_distance=.5, frame_rate=30., stabilization_mode="off")
        path.write_text(json.dumps(metadata))
    path = folder / "frame-005.json"
    metadata = json.loads(path.read_text())
    metadata["source_settings"][field] = value
    path.write_text(json.dumps(metadata))
    _, _, corners = synthetic_views()
    with pytest.raises(CalibrationError, match="settings changed"):
        run_with_corners(monkeypatch, folder, corners)


@pytest.mark.parametrize("mode", ["manual", "single-shot", None])
def test_only_explicit_continuous_modes_allow_illumination_values_to_be_omitted(mode):
    settings = {"exposure_mode": mode, "exposure_time": 150., "exposure_compensation": .5,
                "white_balance_mode": mode, "color_temperature": 5000.,
                "focus_mode": "continuous", "focus_distance": .7}
    assert normalize_capture_settings(settings) == settings


def test_only_document_group_id_is_removed_from_otherwise_strict_capture_settings():
    settings = {"device_id": "measured-lens", "group_id": "document-group",
        "width": 720, "height": 1280, "zoom": 1., "focus_mode": "manual",
        "focus_distance": .5, "frame_rate": 20., "processing_mode": "original"}
    expected = {name: value for name, value in settings.items() if name != "group_id"}
    assert normalize_capture_settings(settings) == expected
    assert normalize_capture_settings({**settings, "group_id": "reloaded-document"}) == expected
    assert settings["group_id"] == "document-group"


def test_omitted_capture_group_diagnostic_must_still_be_finite_json():
    with pytest.raises(CalibrationError, match="finite JSON"):
        normalize_capture_settings({"device_id": "measured-lens", "group_id": float("nan")})


def test_capture_group_id_can_rotate_without_changing_intrinsic_source_settings(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    for index, path in enumerate(sorted(folder.glob("*.json"))):
        metadata = json.loads(path.read_text())
        metadata["source_settings"]["group_id"] = f"document-group-{index}"
        path.write_text(json.dumps(metadata))
    _, _, corners = synthetic_views()
    result = run_with_corners(monkeypatch, folder, corners)
    assert "group_id" not in result["settings"]
    assert result["settings"]["device_id"] == "OFFLINE-SYNTHETIC-NOT-HARDWARE"
    assert result["verified"] is False and not result["timing"]["validated"]
    assert json.loads((folder / "frame-000.json").read_text())["source_settings"]["group_id"] == "document-group-0"


def test_missing_metadata_or_wrong_jpeg_dimensions_are_rejected(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    (folder / "frame-000.json").unlink()
    with pytest.raises(CalibrationError, match="metadata"):
        calibrate_intrinsics(folder, columns=9, rows=6, square_size_m=.025, camera_id="fixture")
    # A fresh directory is not necessary: restore metadata from another image.
    (folder / "frame-000.json").write_bytes((folder / "frame-001.json").read_bytes())
    Image.new("RGB", (320, 240)).save(folder / "frame-000.jpg")
    with pytest.raises(CalibrationError, match="geometry"):
        calibrate_intrinsics(folder, columns=9, rows=6, square_size_m=.025, camera_id="fixture")


def test_insufficient_images_or_detections_never_emit_a_profile(monkeypatch, tmp_path):
    with pytest.raises(CalibrationError, match="At least 15"):
        calibrate_intrinsics(tmp_path, columns=9, rows=6, square_size_m=.025, camera_id="fixture")
    capture_folder(tmp_path)
    monkeypatch.setattr(calibration, "detect_chessboard", lambda *_: None)
    with pytest.raises(CalibrationError, match="Only 0"):
        calibrate_intrinsics(tmp_path, columns=9, rows=6, square_size_m=.025, camera_id="fixture")


def test_repeated_board_pose_is_not_counted_as_calibration_diversity(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    _, _, corners = synthetic_views()
    with pytest.raises(CalibrationError, match="Only 1 distinct"):
        run_with_corners(monkeypatch, folder, [corners[0]] * 20)


@pytest.mark.parametrize("options,match", [({"fixed_centre": True}, "across the image"),
                                         ({"fixed_size": True}, "different distances"),
                                         ({"flat": True}, "clearly tilted")])
def test_position_scale_and_tilt_diversity_are_all_required(monkeypatch, tmp_path, options, match):
    folder = capture_folder(tmp_path)
    _, _, corners = synthetic_views(**options)
    with pytest.raises(CalibrationError, match=match):
        run_with_corners(monkeypatch, folder, corners)


@pytest.mark.parametrize("change", [{"square_size_m": 0.}, {"square_size_m": float("nan")},
                                   {"columns": True}, {"rows": 2}, {"camera_id": ""}])
def test_target_measurements_are_explicit_and_valid(tmp_path, change):
    kwargs = dict(columns=9, rows=6, square_size_m=.025, camera_id="fixture")
    kwargs.update(change)
    with pytest.raises(CalibrationError):
        calibrate_intrinsics(tmp_path, **kwargs)


def test_measured_mount_transform_axes_rigidity_and_translation():
    T = mount_transform(height_m=.31, forward_m=.02, left_m=-.03,
                        pitch_down_deg=25., yaw_deg=90., roll_deg=0.)
    np.testing.assert_allclose(T[:3, 3], [.02, -.03, .31])
    np.testing.assert_allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-12)
    assert np.linalg.det(T[:3, :3]) == pytest.approx(1.)
    np.testing.assert_allclose(T[:3, 2], [0., math.cos(math.radians(25)), -math.sin(math.radians(25))], atol=1e-12)
    np.testing.assert_allclose(T[:3, 0], [1., 0., 0.], atol=1e-12)


@pytest.mark.parametrize("change", [{"height_m": 0}, {"forward_m": float("nan")},
                                   {"pitch_down_deg": float("nan")}, {"roll_deg": True}])
def test_invalid_mount_measurements_do_not_get_a_transform(change):
    kwargs = dict(height_m=.31, forward_m=.02, left_m=-.03,
                  pitch_down_deg=25., yaw_deg=0., roll_deg=0.)
    kwargs.update(change)
    with pytest.raises(CalibrationError):
        mount_transform(**kwargs)


@pytest.mark.parametrize("pitch", [0., -5., -25.])
def test_measured_mount_conversion_does_not_assume_floor_visibility_from_pitch(pitch):
    T = mount_transform(height_m=.22, forward_m=.02, left_m=0.,
                        pitch_down_deg=pitch, yaw_deg=0., roll_deg=0.)
    np.testing.assert_allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-12)
    assert T[2, 2] == pytest.approx(-math.sin(math.radians(pitch)))
    assert T[2, 3] == pytest.approx(.22)


def test_mount_cli_writes_only_transform_and_never_overwrites_without_explicit_force(tmp_path):
    target = tmp_path / "mount.json"
    args = ["mount", "--height-m", ".31", "--forward-m", ".02", "--left-m", "-.03",
            "--pitch-down-deg", "25", "--yaw-deg", "0", "--roll-deg", "0", "--output", str(target)]
    assert main(args) == 0
    result = json.loads(target.read_text())
    assert list(result) == ["T_base_camera"]
    assert np.asarray(result["T_base_camera"]).shape == (4, 4)
    target.write_text("keep-existing")
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2 and target.read_text() == "keep-existing"
    assert main(args + ["--force"]) == 0


def test_mount_cli_requires_every_measured_value():
    with pytest.raises(SystemExit) as error:
        main(["mount", "--height-m", ".31"])
    assert error.value.code == 2


def test_intrinsics_cli_accepts_saved_capture_wrapper_and_writes_unverified_profile(monkeypatch, tmp_path):
    folder = capture_folder(tmp_path)
    for path in folder.glob("*.json"):
        path.write_text(json.dumps({"metadata": json.loads(path.read_text()), "purpose": "offline-fixture"}))
    _, _, corners = synthetic_views()
    stream = iter(corners)
    monkeypatch.setattr(calibration, "detect_chessboard", lambda *_: next(stream))
    output = tmp_path / "intrinsics.unverified.json"
    assert main(["intrinsics", str(folder), "--columns", "9", "--rows", "6",
                 "--square-size-m", ".025", "--camera-id", "OFFLINE-FIXTURE",
                 "--output", str(output)]) == 0
    result = json.loads(output.read_text())
    assert result["verified"] is False and result["T_base_camera"] is None
    assert result["timing"]["validated"] is False
