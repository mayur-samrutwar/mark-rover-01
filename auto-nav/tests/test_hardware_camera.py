"""Synthetic transport failures and calibration transforms; no real camera/motor.

The calibration below is a mathematical fixture, never a deployable phone
profile. Tests verify boundaries, not physical calibration or braking.
"""
import copy
import io
import math

import numpy as np
from PIL import Image
import pytest

from rover_nav.hardware_camera import CameraCalibration, CameraFrameError, CameraIngest
from rover_nav.hardware_calibration import mount_transform
from rover_nav.hardware_floor_calibration import floor_patch_visibility
from rover_nav.floor_vision import FloorVision


def profile(**changes):
    angle = math.radians(25)
    c, s = math.cos(angle), math.sin(angle)
    value = {"version": 1, "camera_id": "synthetic-test-fixture-not-hardware",
             "source_width": 640, "source_height": 480, "width": 320, "height": 240,
             "K": [[400., 0., 319.5], [0., 400., 239.5], [0., 0., 1.]],
             "distortion": [0., 0., 0., 0., 0.],
             "T_base_camera": [[0., -s, c, .02], [-1., 0., 0., 0.],
                               [0., -c, -s, .3], [0., 0., 0., 1.]],
             "settings": {"device_id": "synthetic-lens", "width": 640, "height": 480,
                          "facing_mode": "environment", "zoom": 1., "focus_mode": "manual"},
             "verified": True, "timing": {"max_pipeline_age_s": .05, "validated": True}}
    value.update(changes)
    return value


def jpeg(size=(320, 240), *, orientation=None):
    # Non-uniform pixels verify that the decoded/corrected image is the actual
    # supplied RGB frame, not a placeholder or generated perception input.
    yy, xx = np.indices((size[1], size[0]))
    pixels = np.stack((xx % 255, yy % 255, (xx + yy) % 255), axis=-1).astype(np.uint8)
    out = io.BytesIO()
    options = {}
    if orientation is not None:
        exif = Image.Exif()
        exif[274] = orientation
        options["exif"] = exif
    Image.fromarray(pixels).save(out, format="JPEG", **options)
    return out.getvalue()


def metadata(camera, now=10., **changes):
    challenge = camera.challenge(now)
    value = {"challenge_id": challenge["challenge_id"],
             "calibration_id": challenge["calibration_id"], "stream_id": "phone-generation-1",
             "sequence": 0, "presented_frames": 1, "media_time": .1,
             "source_width": 640, "source_height": 480, "width": 320, "height": 240,
             "rotation_degrees": 0, "mirrored": False, "crop": "none",
             "settings": camera.calibration.settings if camera.calibration else {"device_id": "preview"}}
    value.update(changes)
    return value


def test_measured_intrinsics_scaled_with_pixel_centres_and_fixed_transform():
    calibration = CameraCalibration.from_dict(profile())
    measured = calibration.perception_calibration
    np.testing.assert_allclose(measured["K"], [[200., 0., 159.5], [0., 200., 119.5], [0., 0., 1.]])
    np.testing.assert_allclose(measured["T_base_camera"], profile()["T_base_camera"])
    assert calibration.public_descriptor()["calibration_id"] == calibration.calibration_id
    assert len(calibration.calibration_id) == 64


@pytest.mark.parametrize("pitch_down_deg", [0., -3., -5.])
def test_horizontal_or_slightly_upward_mount_can_observe_required_floor_patch(pitch_down_deg):
    transform = mount_transform(height_m=.3, forward_m=.02, left_m=0.,
        pitch_down_deg=pitch_down_deg, yaw_deg=0., roll_deg=0.)
    calibration = CameraCalibration.from_dict(profile(T_base_camera=transform.tolist()))
    final = calibration.perception_calibration
    floor = FloorVision(final["K"], transform, final["width"], final["height"])
    patch = floor_patch_visibility(transform, final["K"], np.zeros(5), (320, 240))
    assert transform[2, 2] >= -.02
    assert patch["seed_visible"] and patch["seed_pixel_count"] >= 100
    assert patch["seed_pixel_count"] == int(floor._seed.sum())
    assert not patch["floor_appearance_or_texture_validated"]


@pytest.mark.parametrize("pitch_down_deg", [-15., -45., -90.])
def test_upward_mount_without_the_required_floor_patch_is_rejected(pitch_down_deg):
    transform = mount_transform(height_m=.3, forward_m=.02, left_m=0.,
        pitch_down_deg=pitch_down_deg, yaw_deg=0., roll_deg=0.)
    with pytest.raises(ValueError, match="required initial floor calibration patch in the final rectified view"):
        CameraCalibration.from_dict(profile(T_base_camera=transform.tolist()))


def test_floor_visibility_gate_uses_final_rectified_view_after_fov_narrowing():
    distortion = [.4, 0., 0., 0., 0.]
    transform = mount_transform(height_m=.3, forward_m=.02, left_m=0.,
        pitch_down_deg=-3., yaw_deg=0., roll_deg=0.)
    # This lens still shows the patch in its raw image, but the bounded virtual
    # pinhole image must narrow enough that it loses the same floor region.
    final = CameraCalibration.from_dict(profile(distortion=distortion)).perception_calibration
    raw_K = np.array([[200., 0., 159.5], [0., 200., 119.5], [0., 0., 1.]])
    raw = floor_patch_visibility(transform, raw_K, distortion, (320, 240))
    rectified = floor_patch_visibility(transform, final["K"], np.zeros(5), (320, 240))
    assert raw["seed_visible"] and raw["seed_pixel_count"] >= 100
    assert not rectified["seed_visible"]
    with pytest.raises(ValueError, match="final rectified view"):
        CameraCalibration.from_dict(profile(distortion=distortion, T_base_camera=transform.tolist()))


def test_floor_visibility_alone_does_not_verify_camera_or_timing():
    transform = mount_transform(height_m=.3, forward_m=.02, left_m=0.,
        pitch_down_deg=-3., yaw_deg=0., roll_deg=0.)
    calibration = CameraCalibration.from_dict(profile(T_base_camera=transform.tolist(),
        verified=False, timing={"max_pipeline_age_s": None, "validated": False}))
    assert not calibration.verified and not calibration.timing_validated
    camera = CameraIngest(calibration)
    result = camera.ingest(jpeg(), metadata(camera), 10.1)
    assert not result["live_ready"] and not result["new_frame"]
    with pytest.raises(CameraFrameError, match="calibration required"):
        camera.read(10.11)


def test_document_group_id_does_not_change_persisted_optics_or_calibration_fingerprint():
    original = profile()
    variants = [copy.deepcopy(original) for _ in range(3)]
    variants[0]["settings"]["group_id"] = "old-document-group"
    variants[1]["settings"]["group_id"] = "new-document-group"
    loaded = [CameraCalibration.from_dict(value) for value in variants]
    assert len({item.calibration_id for item in loaded}) == 1
    for item in loaded:
        assert item.settings == original["settings"]
        assert item.public_descriptor()["settings"] == original["settings"]
        np.testing.assert_array_equal(item.K, loaded[0].K)
    assert variants[0]["settings"]["group_id"] == "old-document-group"


def test_legacy_profile_group_id_removal_does_not_verify_camera_or_timing():
    value = profile(verified=False, timing={"max_pipeline_age_s": None, "validated": False})
    value["settings"]["group_id"] = "legacy-document-group"
    calibration = CameraCalibration.from_dict(value)
    assert "group_id" not in calibration.settings
    assert not calibration.verified and not calibration.timing_validated


def test_group_only_frame_difference_is_accepted_and_raw_group_diagnostics_remain():
    value = profile()
    value["settings"]["group_id"] = "profile-document-group"
    camera = CameraIngest(value)
    for index, group in enumerate(["first-document-group", "rotated-group-diagnostic"]):
        settings = {**camera.calibration.settings, "group_id": group}
        frame = metadata(camera, now=10. + index * .15,
            sequence=index, presented_frames=index + 1, media_time=.1 + index * .1,
            settings=settings, source_settings=dict(settings))
        result = camera.ingest(jpeg(), frame, 10.1 + index * .15)
        assert result["new_frame"] and result["live_ready"] and not result["reset_required"]
        assert result["source_metadata"]["settings"]["group_id"] == group
        assert result["source_metadata"]["source_settings"]["group_id"] == group
        assert frame["settings"]["group_id"] == group
        assert "group_id" not in result["descriptor"]["settings"]
        camera.read(10.11 + index * .15)


@pytest.mark.parametrize("field,item", [
    ("device_id", "another-lens"), ("zoom", 2.), ("focus_mode", "continuous"),
    ("focus_distance", .5), ("frame_rate", 15.), ("exposure_mode", "continuous"),
    ("white_balance_mode", "continuous"), ("stabilization_mode", "on"),
    ("unknown_processing_setting", {"changed": True}),
])
def test_ignoring_group_id_keeps_every_optical_and_processing_field_strict(field, item):
    camera = CameraIngest(profile())
    camera.ingest(jpeg(), metadata(camera), 10.1)
    settings = {**camera.calibration.settings, "group_id": "new-group", field: item}
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), metadata(camera, 10.15, sequence=1, presented_frames=2,
            media_time=.2, settings=settings), 10.2)
    assert error.value.code == "camera_settings_changed_reset_required"
    assert camera.status(10.2)["reset_required"]


def test_rotated_group_does_not_allow_stream_generation_to_change_without_reset():
    camera = CameraIngest(profile())
    camera.ingest(jpeg(), metadata(camera,
        settings={**camera.calibration.settings, "group_id": "first-group"}), 10.1)
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), metadata(camera, 10.15, sequence=1, presented_frames=2,
            media_time=.2, stream_id="new-camera-stream",
            settings={**camera.calibration.settings, "group_id": "second-group"}), 10.2)
    assert error.value.code == "camera_source_changed_reset_required"
    assert camera.status(10.2)["reset_required"]


@pytest.mark.parametrize("group_id", [float("nan"), "x" * 4096])
def test_omitted_profile_group_diagnostic_still_obeys_finite_json_and_size_limits(group_id):
    value = profile()
    value["settings"]["group_id"] = group_id
    with pytest.raises(ValueError, match="Camera settings"):
        CameraCalibration.from_dict(value)


def test_omitted_frame_group_diagnostic_must_still_be_finite_json():
    camera = CameraIngest(profile())
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), metadata(camera,
            settings={**camera.calibration.settings, "group_id": float("nan")}), 10.1)
    assert error.value.code == "invalid_frame_metadata"


@pytest.mark.parametrize("change", [
    {"version": 2}, {"version": True}, {"source_width": None}, {"height": 180}, {"width": 1280, "height": 960},
    {"K": None}, {"K": [[-1., 0., 1.], [0., 1., 1.], [0., 0., 1.]]},
    {"distortion": None}, {"distortion": [0., float("nan"), 0., 0.]},
    {"T_base_camera": np.eye(4).tolist()}, {"settings": {}},
    {"timing": {"max_pipeline_age_s": None, "validated": True}},
    {"verified": False, "timing": {"max_pipeline_age_s": .05, "validated": True}},
])
def test_reject_incomplete_unmeasured_or_inconsistent_calibration(change):
    with pytest.raises((ValueError, TypeError)):
        CameraCalibration.from_dict(profile(**change))


def test_browser_time_is_explicit_conservative_lower_bound_and_read_once():
    camera = CameraIngest(profile())
    image = jpeg()
    result = camera.ingest(image, metadata(camera), 10.1)
    assert result["live_ready"] and result["hardware_timing_validated"]
    assert result["timestamp_source"] == "browser_challenge_lower_bound_not_exact_exposure"
    assert result["frame_age_upper_bound_s"] == pytest.approx(.15)
    packet = camera.read(10.11)
    assert packet.timestamp == pytest.approx(9.95)
    expected = np.asarray(Image.open(io.BytesIO(image)).convert("RGB"))
    np.testing.assert_array_equal(packet.pixels, expected)
    assert not packet.pixels.flags.writeable
    with pytest.raises(CameraFrameError, match="no new camera frame"):
        camera.read(10.12)


def test_uncalibrated_preview_never_produces_perception_packet():
    camera = CameraIngest()
    image = jpeg()
    result = camera.ingest(image, metadata(camera), 10.1)
    assert camera.latest_image() == image
    assert result["accepted_frames"] == 1 and not result["live_ready"]
    assert result["frame_age_upper_bound_s"] is None
    with pytest.raises(CameraFrameError) as error:
        camera.read(10.11)
    assert error.value.code == "calibration_required"


@pytest.mark.parametrize("verified,bound,reason", [(False, .05, "calibration_required"),
                                                 (True, None, "camera_pipeline_latency_bound_required")])
def test_unverified_calibration_or_unknown_pipeline_age_is_preview_only(verified, bound, reason):
    camera = CameraIngest(profile(verified=verified, timing={"max_pipeline_age_s": bound, "validated": False}))
    result = camera.ingest(jpeg(), metadata(camera), 10.1)
    assert not result["live_ready"] and not result["new_frame"]
    with pytest.raises(CameraFrameError) as error:
        camera.read(10.11)
    assert error.value.code == reason


def test_measured_bound_without_physical_timing_validation_allows_shadow_only():
    camera = CameraIngest(profile(timing={"max_pipeline_age_s": .05, "validated": False}))
    result = camera.ingest(jpeg(), metadata(camera), 10.1)
    assert result["new_frame"] and not result["live_ready"]
    assert camera.read(10.11).timestamp == pytest.approx(9.95)


def test_stale_network_challenge_and_age_bound_are_rejected():
    camera = CameraIngest(profile())
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), metadata(camera), 10.31)
    assert error.value.code == "stale_frame_challenge"
    camera = CameraIngest(profile(), max_frame_age_s=.1)
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), metadata(camera), 10.1)
    assert error.value.code == "frame_age_bound_exceeded"


def test_frame_age_is_rechecked_after_host_processing_and_no_false_freshness():
    camera = CameraIngest(profile())
    camera.ingest(jpeg(), metadata(camera), 10.1)
    assert not camera.status(10.4)["fresh"]
    assert not camera.status(10.4)["live_ready"]
    with pytest.raises(CameraFrameError) as error:
        camera.read(10.4)
    assert error.value.code == "stale_camera_frame"


def test_injected_host_monotonic_clock_and_idempotent_pending_challenge():
    clock = [20.]
    camera = CameraIngest(profile(), clock=lambda: clock[0])
    a = camera.challenge()
    clock[0] += .02
    assert camera.challenge() == a
    m = metadata(camera, now=clock[0])
    camera.ingest(jpeg(), m)
    assert camera.read().timestamp == pytest.approx(19.95)


@pytest.mark.parametrize("change", [{"sequence": 0}, {"presented_frames": 1}, {"media_time": .1}])
def test_duplicate_or_out_of_order_exposure_cannot_become_fresh(change):
    camera = CameraIngest(profile())
    camera.ingest(jpeg(), metadata(camera), 10.1)
    update = {"sequence": 1, "presented_frames": 2, "media_time": .2}
    update.update(change)
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), metadata(camera, 10.15, **update), 10.2)
    assert error.value.code == "repeated_or_out_of_order_frame"
    assert not camera.status(10.2)["fresh"]


def test_challenge_cannot_be_replayed_even_after_failed_decode():
    camera = CameraIngest(profile())
    m = metadata(camera)
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(b"not jpeg", m, 10.1)
    assert error.value.code == "invalid_jpeg"
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), m, 10.11)
    assert error.value.code == "unknown_or_replayed_challenge"


def test_nonfinite_or_oversized_metadata_cannot_poison_status_or_reuse_challenge():
    for change in [{"source_settings": {"zoom": float("nan")}}, {"extra": "x" * 17000}]:
        camera = CameraIngest(profile())
        m = metadata(camera, **change)
        with pytest.raises(CameraFrameError) as error:
            camera.ingest(jpeg(), m, 10.1)
        assert error.value.code == "invalid_frame_metadata"
        m.pop("source_settings", None)
        m.pop("extra", None)
        with pytest.raises(CameraFrameError) as error:
            camera.ingest(jpeg(), m, 10.11)
        assert error.value.code == "unknown_or_replayed_challenge"


@pytest.mark.parametrize("change", [{"stream_id": "another-camera-generation"},
                                   {"source_width": 1280, "source_height": 960},
                                   {"calibration_id": "wrong"},
                                   {"settings": {"device_id": "another-lens"}}])
def test_source_lens_geometry_or_calibration_change_latches_reset(change):
    camera = CameraIngest(profile())
    camera.ingest(jpeg(), metadata(camera), 10.1)
    with pytest.raises(CameraFrameError):
        camera.ingest(jpeg(), metadata(camera, 10.15, sequence=1, presented_frames=2, media_time=.2, **change), 10.2)
    assert camera.status(10.2)["reset_required"]
    with pytest.raises(CameraFrameError, match="camera session reset required"):
        camera.challenge(10.21)
    camera.reset()
    assert camera.latest_image() is None and not camera.status(10.21)["reset_required"]
    assert camera.status(10.21)["source_metadata"] is None
    assert camera.ingest(jpeg(), metadata(camera, 10.3), 10.35)["new_frame"]


@pytest.mark.parametrize("change", [{"rotation_degrees": 90}, {"mirrored": True}, {"crop": [0, 0, 300, 200]},
                                   {"width": 320, "height": 180}, {"sequence": True},
                                   {"media_time": float("nan")}, {"presented_frames": 0}])
def test_unrecognized_geometry_and_invalid_frame_timing_are_rejected(change):
    camera = CameraIngest(profile())
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), metadata(camera, **change), 10.1)
    assert error.value.code == ("invalid_frame_metadata" if "media_time" in change else "invalid_frame_geometry_or_timing")


def test_encoded_jpeg_dimensions_and_exif_orientation_must_match_raw_frame():
    for image, code in [(jpeg((640, 480)), "jpeg_geometry_mismatch"),
                        (jpeg(orientation=6), "jpeg_orientation_not_canonical")]:
        camera = CameraIngest(profile())
        with pytest.raises(CameraFrameError) as error:
            camera.ingest(image, metadata(camera), 10.1)
        assert error.value.code == code


def test_rotation_change_after_accepted_frame_requires_estimator_and_map_reset():
    camera = CameraIngest(profile())
    camera.ingest(jpeg(), metadata(camera), 10.1)
    with pytest.raises(CameraFrameError):
        camera.ingest(jpeg(), metadata(camera, 10.15, sequence=1, presented_frames=2,
                                      media_time=.2, rotation_degrees=90), 10.2)
    assert camera.status(10.2)["reset_required"]


def test_explicit_reset_invalidates_challenges_and_all_cached_frame_authority():
    camera = CameraIngest(profile())
    camera.ingest(jpeg(), metadata(camera), 10.1)
    old = metadata(camera, 10.15, sequence=1, presented_frames=2, media_time=.2)
    camera.reset()
    assert camera.latest_image() is None
    with pytest.raises(CameraFrameError):
        camera.read(10.2)
    with pytest.raises(CameraFrameError) as error:
        camera.ingest(jpeg(), old, 10.2)
    assert error.value.code == "unknown_or_replayed_challenge"


@pytest.mark.parametrize("distortion", [[-.12, .02, .001, -.001, 0.], [.08, -.01, 0., 0., 0.]])
def test_real_distortion_is_rectified_to_known_pinhole_without_unknown_border(distortion):
    calibration = CameraCalibration.from_dict(profile(distortion=distortion))
    camera = CameraIngest(calibration)
    camera.ingest(jpeg(), metadata(camera), 10.1)
    packet = camera.read(10.11)
    assert packet.pixels.shape == (240, 320, 3)
    assert camera._valid_pixels.all()
    assert np.isfinite(np.asarray(calibration.perception_calibration["K"])).all()
    original = np.asarray(Image.open(io.BytesIO(jpeg())).convert("RGB"))
    assert not np.array_equal(original, packet.pixels)


def test_calibration_has_no_room_structure_or_simulator_input():
    calibration = CameraCalibration.from_dict(profile())
    assert set(calibration.perception_calibration) == {"width", "height", "K", "T_base_camera", "source"}
    assert "world" not in calibration.perception_calibration
