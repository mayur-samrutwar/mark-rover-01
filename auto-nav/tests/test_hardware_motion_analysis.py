"""Known synthetic floor motion and evidence boundaries; no physical rover."""
import copy
import hashlib
import inspect
import io
import json
import math

import cv2
import numpy as np
from PIL import Image
import pytest

from rover_nav.hardware_camera import CameraCalibration, CameraIngest
from rover_nav.hardware_motion_analysis import (
    MotionAnalysisError, analyze_motion, clear_floor_seed,
)
from rover_nav.navigation import Pose
from test_visual_odometry import FloorImages, HEIGHT, K, T, WIDTH


@pytest.fixture(scope="module")
def floor():
    return FloorImages()


def profile(**changes):
    value = {"version": 1, "camera_id": "synthetic-floor-test-not-hardware",
        "source_width": WIDTH, "source_height": HEIGHT, "width": WIDTH, "height": HEIGHT,
        "K": K.tolist(), "T_base_camera": T.tolist(), "distortion": [0.] * 5,
        "verified": False, "timing": {"max_pipeline_age_s": None, "validated": False},
        "settings": {"device_id": "synthetic-test-lens", "width": WIDTH, "height": HEIGHT,
            "zoom": 1., "focus_mode": "manual", "group_id": "document-a",
            "exposure_mode": "continuous", "white_balance_mode": "continuous"}}
    value.update(changes)
    return value


def recording(tmp_path, floor, poses=None, phases=None, image_overrides=None):
    poses = poses or ([Pose(0., 0., 0.)] * 6
        + [Pose(.01 * i, 0., 0.) for i in range(1, 5)] + [Pose(.04, 0., 0.)] * 8)
    phases = phases or (["before"] * 6 + ["pulse"] * 4 + ["after"] * 8)
    manifest = {"version": 1, "verified": False, "session_id": "synthetic-test-session",
                "requested": {"physical_left_pwm": 40, "physical_right_pwm": 40, "duration_s": .1},
                "frames": []}
    for index, (pose, phase) in enumerate(zip(poses, phases)):
        pixels = (image_overrides or {}).get(index, floor.frame(pose))
        data = io.BytesIO()
        Image.fromarray(pixels).save(data, format="JPEG", quality=98, subsampling=0)
        jpg = tmp_path / f"frame-{index:03d}.jpg"
        jpg.write_bytes(data.getvalue())
        meta = {"source_width": WIDTH, "source_height": HEIGHT, "width": WIDTH, "height": HEIGHT,
            "media_time": 100. + .1 * index, "presented_frames": index + 1, "sequence": index,
            "stream_id": "fixed-test-stream", "rotation_degrees": 0, "mirrored": False, "crop": "none",
            "source_settings": {**profile()["settings"], "exposure_time": 300. + index,
                                "color_temperature": index * 20}}
        sidecar = tmp_path / f"frame-{index:03d}.json"
        sidecar.write_text(json.dumps(meta))
        manifest["frames"].append({"phase": phase, "host_requested_at": 200. + index * .1,
            "host_saved_at": 200.01 + index * .1, "source_sequence": index,
            "stream_id": meta["stream_id"], "image": str(jpg), "metadata": str(sidecar),
            "jpeg_sha256": hashlib.sha256(jpg.read_bytes()).hexdigest(),
            "metadata_sha256": hashlib.sha256(sidecar.read_bytes()).hexdigest()})
    return manifest


def change_metadata(manifest, index, modify):
    entry = manifest["frames"][index]
    from pathlib import Path
    path = Path(entry["metadata"])
    metadata = json.loads(path.read_text())
    modify(metadata)
    path.write_text(json.dumps(metadata))
    entry["metadata_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()


def test_known_translation_is_provisional_and_never_validates_speed_or_free_space(tmp_path, floor):
    manifest = recording(tmp_path, floor)
    result = analyze_motion(manifest, profile(), clear_floor_verified=True)
    assert result["quality_blockers"] == []
    response = result["response"]
    assert response["translation_resolved"] and response["direction"] == "forward"
    assert response["delta_forward_m"] == pytest.approx(.04, abs=.006)
    assert abs(response["delta_left_m"]) < .004
    assert abs(response["delta_yaw_rad"]) < .01
    assert result["baseline"]["minimum_resolvable_translation_m"] >= .01
    assert result["settling"]["visually_stable_final_window"]
    assert not result["valid_for_autonomous_free_space"] and not result["verified"]
    assert result["speed_calibration"] is result["braking_calibration"] is None
    assert response["provisional_window_average_translation_m_s"] < .04
    assert "NOT pulse velocity" in response["average_note"]
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("yaw", [.12, -.12])
def test_known_rotation_direction_and_base_motion_remain_separate(tmp_path, floor, yaw):
    poses = [Pose(0., 0., 0.)] * 6 + [Pose(0., 0., yaw * i / 4) for i in range(1, 5)] + [Pose(0., 0., yaw)] * 8
    report = analyze_motion(recording(tmp_path, floor, poses), profile(),
                            ground_roi=floor.valid.astype(bool))
    assert report["quality_blockers"] == []
    response = report["response"]
    assert response["yaw_resolved"]
    assert response["rotation_direction"] == ("left" if yaw > 0 else "right")
    assert response["delta_yaw_rad"] == pytest.approx(yaw, abs=.012)
    assert not response["translation_resolved"]


def test_stationary_sequence_and_subthreshold_motion_are_not_claimed_as_resolution(tmp_path, floor):
    poses = [Pose(0., 0., 0.)] * 6 + [Pose(.001 * i, 0., 0.) for i in range(1, 5)] + [Pose(.004, 0., 0.)] * 8
    report = analyze_motion(recording(tmp_path, floor, poses), profile(), clear_floor_verified=True)
    assert report["quality_blockers"] == []
    assert report["response"]["direction"] == "not_resolved"
    assert not report["response"]["translation_resolved"]
    assert report["response"]["provisional_window_average_translation_m_s"] is None


def test_baseline_noise_increases_minimum_resolvable_motion(tmp_path, floor):
    poses = [Pose(x, 0., 0.) for x in [0., .005, -.005, .005, -.005, 0.]]
    poses += [Pose(.003 * i, 0., 0.) for i in range(1, 5)] + [Pose(.012, 0., 0.)] * 8
    report = analyze_motion(recording(tmp_path, floor, poses), profile(), ground_roi=floor.valid.astype(bool))
    assert report["quality_blockers"] == []
    baseline = report["baseline"]
    assert baseline["translation_jitter_diameter_m"] > .006
    assert baseline["minimum_resolvable_translation_m"] >= 3. * baseline["translation_jitter_diameter_m"]
    assert not report["response"]["translation_resolved"]


def test_failed_frame_blocks_result_and_no_recovery_is_used_to_bridge_it(tmp_path, floor):
    blank = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    manifest = recording(tmp_path, floor, image_overrides={7: blank})
    report = analyze_motion(manifest, profile(), ground_roi=floor.valid.astype(bool))
    assert any("tracking_failed_at_frame_7" in item for item in report["quality_blockers"])
    assert report["response"] is None
    assert not report["frames"][7]["tracking"]
    assert report["frames"][7]["pose"] is None
    assert all(frame["reason"] == "not_processed_after_tracking_failure" for frame in report["frames"][8:])
    assert all(frame["pose"] is None for frame in report["frames"][8:])


@pytest.mark.parametrize("phase", ["before", "after"])
def test_insufficient_stationary_window_is_explicit_blocker(tmp_path, floor, phase):
    manifest = recording(tmp_path, floor)
    manifest["frames"] = [frame for frame in manifest["frames"]
                          if frame["phase"] != phase or frame["source_sequence"] in (0, 1, 16, 17)]
    report = analyze_motion(manifest, profile(), clear_floor_verified=True)
    assert any("insufficient_" in item for item in report["quality_blockers"])
    assert report["response"] is None


def test_still_moving_final_window_cannot_be_reported_as_settled_response(tmp_path, floor):
    poses = [Pose(0., 0., 0.)] * 6 + [Pose(.01 * i, 0., 0.) for i in range(1, 13)]
    report = analyze_motion(recording(tmp_path, floor, poses), profile(), ground_roi=floor.valid.astype(bool))
    assert report["quality_blockers"] == ["final_window_not_visually_settled"]
    assert report["response"] is None
    assert not report["settling"]["visually_stable_final_window"]


def test_explicit_floor_assumption_is_required_and_exclusions_only_remove(tmp_path, floor):
    manifest = recording(tmp_path, floor)
    with pytest.raises(MotionAnalysisError, match="Manual floor-only ROI"):
        analyze_motion(manifest, profile())
    with pytest.raises(MotionAnalysisError, match="boolean"):
        analyze_motion(manifest, profile(), ground_roi=np.ones((HEIGHT, WIDTH), np.uint8))
    with pytest.raises(MotionAnalysisError, match="fewer than 100"):
        analyze_motion(manifest, profile(), clear_floor_verified=True,
                       roi_exclusions=np.ones((HEIGHT, WIDTH), bool))


def test_manual_exclusion_keeps_moving_object_pixels_out_of_floor_odometry(tmp_path, floor):
    poses = [Pose(0., 0., 0.)] * 6 + [Pose(.01 * i, 0., 0.) for i in range(1, 5)] + [Pose(.04, 0., 0.)] * 8
    rng = np.random.default_rng(37)
    overrides = {}
    for index, pose in enumerate(poses):
        image = floor.frame(pose)
        image[100:180, 100:220] = rng.integers(0, 256, (80, 120, 3), dtype=np.uint8)
        overrides[index] = image
    exclusion = np.zeros((HEIGHT, WIDTH), bool)
    # A margin around the object also excludes LK interpolation patches.
    exclusion[85:195, 85:235] = True
    report = analyze_motion(recording(tmp_path, floor, poses, image_overrides=overrides), profile(),
                            ground_roi=floor.valid.astype(bool), roi_exclusions=exclusion)
    assert report["quality_blockers"] == []
    assert report["response"]["delta_forward_m"] == pytest.approx(.04, abs=.006)
    assert report["roi"]["pixels"] == np.count_nonzero(floor.valid & ~exclusion)


@pytest.mark.parametrize("changes", [{"version": True}, {"verified": True}])
def test_manifest_cannot_claim_verified_evidence(tmp_path, floor, changes):
    manifest = recording(tmp_path, floor)
    manifest.update(changes)
    with pytest.raises(MotionAnalysisError, match="explicitly unverified"):
        analyze_motion(manifest, profile(), clear_floor_verified=True)


@pytest.mark.parametrize("modify,match", [
    (lambda m: m.update(stream_id="new-stream"), "Stream restart"),
    (lambda m: m.update(width=WIDTH // 2), "geometry changed"),
    (lambda m: m.update(rotation_degrees=90), "Rotated"),
    (lambda m: m.update(mirrored=True), "Rotated"),
    (lambda m: m.update(crop="centre"), "Rotated"),
    (lambda m: m["source_settings"].update(zoom=2.), "settings changed"),
    (lambda m: m["source_settings"].update(device_id="another-lens"), "settings changed"),
    (lambda m: m["source_settings"].update(focus_distance=.1), "settings changed"),
    (lambda m: m.update(media_time=100.), "nonmonotonic"),
    (lambda m: m.update(sequence=0), "nonmonotonic"),
    (lambda m: m.update(presented_frames=1), "nonmonotonic"),
])
def test_changed_source_or_camera_order_blocks_analysis(tmp_path, floor, modify, match):
    manifest = recording(tmp_path, floor)
    change_metadata(manifest, 8, modify)
    with pytest.raises(MotionAnalysisError, match=match):
        analyze_motion(manifest, profile(), clear_floor_verified=True)


def test_document_group_and_automatic_illumination_are_diagnostic_not_optical_changes(tmp_path, floor):
    manifest = recording(tmp_path, floor)
    for index in range(len(manifest["frames"])):
        change_metadata(manifest, index, lambda m: m["source_settings"].update(group_id="new-document"))
    report = analyze_motion(manifest, profile(), clear_floor_verified=True)
    assert report["quality_blockers"] == []


def test_incorrect_digest_prevents_mismatched_image_metadata(tmp_path, floor):
    manifest = recording(tmp_path, floor)
    manifest["frames"][1]["jpeg_sha256"] = "0" * 64
    with pytest.raises(MotionAnalysisError, match="digest mismatch"):
        analyze_motion(manifest, profile(), clear_floor_verified=True)


def test_phase_order_and_host_order_are_checked_before_vo(tmp_path, floor):
    manifest = recording(tmp_path, floor)
    manifest["frames"][8]["phase"] = "before"
    with pytest.raises(MotionAnalysisError, match="phases cannot go backwards"):
        analyze_motion(manifest, profile(), clear_floor_verified=True)
    manifest["frames"][8]["phase"] = "pulse"
    manifest["frames"][8]["host_requested_at"] = 200.
    with pytest.raises(MotionAnalysisError, match="host times must increase"):
        analyze_motion(manifest, profile(), clear_floor_verified=True)


def test_rectification_is_exactly_camera_ingest_map_and_virtual_intrinsics(monkeypatch, tmp_path, floor):
    manifest = recording(tmp_path, floor)
    candidate = profile(distortion=[-.04, .02, .001, -.001, 0.])
    calibration = CameraCalibration.from_dict(candidate)
    ingest = CameraIngest(calibration)
    seen = []
    from rover_nav import hardware_motion_analysis as module
    real_vo = module.VisualOdometry

    class InspectVO(real_vo):
        def update(self, rgb, timestamp, ground_mask=None, feature_mask=None):
            if not seen:
                seen.append((rgb.copy(), self.K.copy()))
            return super().update(rgb, timestamp, ground_mask, feature_mask)

    monkeypatch.setattr(module, "VisualOdometry", InspectVO)
    analyze_motion(manifest, candidate, ground_roi=floor.valid.astype(bool))
    raw = np.array(Image.open(manifest["frames"][0]["image"]).convert("RGB"))
    expected = cv2.remap(raw, ingest._map1, ingest._map2, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)
    np.testing.assert_array_equal(seen[0][0], expected)
    np.testing.assert_allclose(seen[0][1], calibration.perception_calibration["K"])


def test_diagnostic_has_no_transport_or_autonomous_authority_imports():
    from rover_nav import hardware_motion_analysis as module
    source = inspect.getsource(module)
    for name in ("import requests", "import httpx", "import urllib", "hardware_transport import",
                 "hardware import", "import mujoco", "import isaac"):
        assert name not in source
