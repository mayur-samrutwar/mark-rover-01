"""Offline, provisional RGB response analysis for a supervised bench recording.

This module has no motor or network interface. A manually confirmed floor ROI
is a measurement assumption, NOT autonomous free-space evidence. Camera mount
uncertainty and browser presentation timestamps prevent a speed or braking
calibration; every report remains unverified even when motion is resolved.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from .hardware_calibration import normalize_capture_settings
from .hardware_camera import CameraCalibration, CameraIngest, MAX_FRAME_BYTES
from .navigation import wrap_angle
from .visual_odometry import VisualOdometry


class MotionAnalysisError(ValueError):
    """Invalid, changed, or incomplete evidence; no motion estimate is allowed."""


def _number(value, name, *, minimum=0.):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < minimum):
        raise MotionAnalysisError(f"{name} must be a finite number >= {minimum}")
    return float(value)


def _json(value):
    if isinstance(value, (str, Path)):
        try:
            value = json.loads(Path(value).read_text())
        except (OSError, ValueError) as error:
            raise MotionAnalysisError("Cannot read JSON evidence") from error
    if not isinstance(value, dict):
        raise MotionAnalysisError("Evidence must be a JSON object")
    try:
        json.dumps(value, allow_nan=False)
    except (ValueError, TypeError) as error:
        raise MotionAnalysisError("Evidence must be finite JSON") from error
    return value


def clear_floor_seed(calibration):
    """Geometric central patch, usable ONLY after manual clear-floor confirmation.

    Matches the frozen FloorVision seed geometry in the final rectified image;
    it does not classify objects or prove any pixel lies on the floor.
    """
    view = calibration.perception_calibration
    vo = VisualOdometry(view["K"], view["T_base_camera"], view["width"], view["height"])
    yy, xx = np.mgrid[:view["height"], :view["width"]]
    ground = vo.pixels_to_ground(np.column_stack((xx.ravel(), yy.ravel())))
    relative = ground - calibration.T_base_camera[:2, 3]
    distance = np.linalg.norm(relative, axis=1)
    bearing = np.arctan2(relative[:, 1], relative[:, 0])
    return (np.isfinite(distance) & (distance >= .3) & (distance <= .7)
            & (relative[:, 0] > 0.) & (np.abs(bearing) < .32)).reshape(
                view["height"], view["width"])


def _load_records(manifest, calibration, manifest_dir):
    entries = manifest.get("frames")
    if not isinstance(entries, list) or not 1 <= len(entries) <= 600:
        raise MotionAnalysisError("A bounded bench frames list is required")
    expected_settings = normalize_capture_settings(calibration.settings)
    expected_geometry = (calibration.source_width, calibration.source_height,
                         calibration.width, calibration.height)
    last_media = last_seq = last_presented = last_host = -1.
    stream_id = None
    last_phase = -1
    records = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict) or entry.get("phase") not in ("before", "pulse", "after"):
            raise MotionAnalysisError(f"Frame {index} has no valid bench phase")
        phase = ("before", "pulse", "after").index(entry["phase"])
        if phase < last_phase:
            raise MotionAnalysisError("Bench phases cannot go backwards")
        last_phase = phase
        requested = _number(entry.get("host_requested_at"), "host_requested_at")
        saved = _number(entry.get("host_saved_at"), "host_saved_at")
        if saved < requested or requested <= last_host:
            raise MotionAnalysisError("Bench capture host times must increase")
        last_host = requested
        contents = []
        for key, hash_key, limit in (("image", "jpeg_sha256", MAX_FRAME_BYTES),
                                     ("metadata", "metadata_sha256", 65536)):
            name = entry.get(key)
            if not isinstance(name, str) or not name:
                raise MotionAnalysisError(f"Frame {index} is missing its {key} path")
            path = Path(name)
            if not path.is_absolute():
                path = manifest_dir / path
            try:
                if path.stat().st_size > limit:
                    raise MotionAnalysisError(f"Frame {index} {key} exceeds its bound")
                data = path.read_bytes()
            except OSError as error:
                raise MotionAnalysisError(f"Cannot read frame {index} {key}") from error
            if hashlib.sha256(data).hexdigest() != entry.get(hash_key):
                raise MotionAnalysisError(f"Frame {index} {key} digest mismatch")
            contents.append(data)
        try:
            metadata = json.loads(contents[1])
            if isinstance(metadata.get("metadata"), dict):
                metadata = metadata["metadata"]
            json.dumps(metadata, allow_nan=False)
        except (ValueError, TypeError, AttributeError) as error:
            raise MotionAnalysisError(f"Invalid metadata for frame {index}") from error
        geometry = tuple(metadata.get(key) for key in
                         ("source_width", "source_height", "width", "height"))
        if geometry != expected_geometry or any(type(item) is not int for item in geometry):
            raise MotionAnalysisError("Camera source/output geometry changed")
        if (metadata.get("rotation_degrees") != 0 or metadata.get("mirrored") is not False
                or metadata.get("crop") != "none"):
            raise MotionAnalysisError("Rotated, mirrored, or cropped evidence is unsupported")
        settings = metadata.get("source_settings", metadata.get("settings"))
        if normalize_capture_settings(settings) != expected_settings:
            raise MotionAnalysisError("Camera optical/source settings changed")
        if "optical_settings" in entry and normalize_capture_settings(entry["optical_settings"]) != expected_settings:
            raise MotionAnalysisError("Manifest optical settings disagree with raw metadata")
        current_stream = metadata.get("stream_id")
        if not isinstance(current_stream, str) or not current_stream:
            raise MotionAnalysisError("Missing stream identity")
        if stream_id is not None and current_stream != stream_id:
            raise MotionAnalysisError("Stream restart blocks continuous motion analysis")
        stream_id = current_stream
        if entry.get("stream_id", stream_id) != stream_id:
            raise MotionAnalysisError("Manifest stream identity disagrees with raw metadata")
        media = _number(metadata.get("media_time"), "media_time")
        sequence, presented = metadata.get("sequence"), metadata.get("presented_frames")
        if (type(sequence) is not int or type(presented) is not int or sequence < 0 or presented < 1
                or sequence <= last_seq or presented <= last_presented or media <= last_media):
            raise MotionAnalysisError("Repeated or nonmonotonic camera frames")
        if entry.get("source_sequence", sequence) != sequence:
            raise MotionAnalysisError("Manifest source sequence disagrees with raw metadata")
        last_media, last_seq, last_presented = media, sequence, presented
        try:
            picture = Image.open(io.BytesIO(contents[0]))
            if (picture.format != "JPEG" or picture.size != geometry[2:]
                    or picture.getexif().get(274, 1) != 1):
                raise MotionAnalysisError("JPEG geometry or orientation disagrees with metadata")
            picture.load()
            rgb = np.array(picture.convert("RGB"), dtype=np.uint8)
        except (OSError, ValueError, Image.DecompressionBombError) as error:
            raise MotionAnalysisError(f"Invalid JPEG for frame {index}") from error
        records.append({"index": index, "phase": entry["phase"], "media_time": media,
                        "host_requested_at": requested, "host_saved_at": saved,
                        "sequence": sequence, "stream_id": stream_id, "rgb": rgb,
                        "jpeg_sha256": entry["jpeg_sha256"]})
    return records


def _spread(poses):
    xy = np.asarray([[item["x_m"], item["y_m"]] for item in poses])
    yaw = np.asarray([item["yaw_rad"] for item in poses])
    return (float(np.max(np.linalg.norm(xy[:, None] - xy[None, :], axis=2))),
            float(np.max(np.abs(np.arctan2(np.sin(yaw[:, None] - yaw[None, :]),
                                          np.cos(yaw[:, None] - yaw[None, :]))))))


def _centre(poses):
    return {"x_m": float(np.median([item["x_m"] for item in poses])),
            "y_m": float(np.median([item["y_m"] for item in poses])),
            "yaw_rad": float(math.atan2(np.mean([math.sin(item["yaw_rad"]) for item in poses]),
                                          np.mean([math.cos(item["yaw_rad"]) for item in poses])))}


def analyze_motion(manifest, camera_profile, *, ground_roi=None,
                   clear_floor_verified=False, roi_exclusions=None,
                   minimum_window_frames=5, minimum_window_s=.4):
    """Return an UNVERIFIED response report from one immutable bench recording.

    ``ground_roi`` is a boolean mask in the *rectified* output pixel space,
    manually confirmed to contain only stationary floor throughout this pulse.
    Alternatively ``clear_floor_verified=True`` explicitly supplies that manual
    confirmation for the central 0.3--0.7 m seed. Neither input is a perception
    result. Exclusions can only remove pixels. No failed frame is bridged.
    """
    manifest_dir = Path(manifest).resolve().parent if isinstance(manifest, (str, Path)) else Path.cwd()
    manifest = _json(manifest)
    if type(manifest.get("version")) is not int or manifest.get("version") != 1 or manifest.get("verified") is not False:
        raise MotionAnalysisError("An explicitly unverified version 1 bench manifest is required")
    if type(minimum_window_frames) is not int or not 5 <= minimum_window_frames <= 100:
        raise MotionAnalysisError("minimum_window_frames must be an integer 5..100")
    minimum_window_s = _number(minimum_window_s, "minimum_window_s", minimum=.4)
    if minimum_window_s > 5.:
        raise MotionAnalysisError("minimum_window_s exceeds the diagnostic bound")
    calibration = CameraCalibration.from_dict(_json(camera_profile))
    view = calibration.perception_calibration
    shape = (calibration.height, calibration.width)
    if ground_roi is not None:
        mask = np.asarray(ground_roi)
        if mask.shape != shape or mask.dtype != np.bool_:
            raise MotionAnalysisError("Manually confirmed ground_roi must be a boolean rectified-image mask")
        mask = mask.copy()
        roi_source = "explicit_manually_confirmed_rectified_floor_roi"
    elif clear_floor_verified is True:
        mask = clear_floor_seed(calibration)
        roi_source = "manually_confirmed_clear_central_floor_seed"
    else:
        raise MotionAnalysisError("Manual floor-only ROI confirmation is required; object tops cannot be floor")
    if roi_exclusions is not None:
        exclusions = np.asarray(roi_exclusions)
        if exclusions.shape != shape or exclusions.dtype != np.bool_:
            raise MotionAnalysisError("ROI exclusions must be a boolean rectified-image mask")
        mask &= ~exclusions
    if np.count_nonzero(mask) < 100:
        raise MotionAnalysisError("Manually confirmed floor ROI has fewer than 100 pixels")
    records = _load_records(manifest, calibration, manifest_dir)
    # Reuse the actual ingestion rectification maps. No challenge or ingestion
    # occurs, no verified flag is changed, and no packets are made available.
    rectifier = CameraIngest(calibration)
    if not bool(rectifier._valid_pixels.all()):
        raise MotionAnalysisError("Rectification contains invalid source pixels")
    vo = VisualOdometry(view["K"], view["T_base_camera"], calibration.width, calibration.height)
    report = {"version": 1, "verified": False, "valid_for_autonomous_free_space": False,
        "kind": "offline_manual_floor_roi_motion_response_diagnostic",
        "session_id": manifest.get("session_id"), "calibration_id": calibration.calibration_id,
        "mount_verified": False, "timing_validated": False,
        "roi": {"source": roi_source, "pixel_space": "rectified_output", "pixels": int(mask.sum()),
                "sha256": hashlib.sha256(mask.tobytes()).hexdigest(), "verified": False},
        "quality_blockers": [], "frames": [], "baseline": None, "settling": None,
        "response": None, "speed_calibration": None, "braking_calibration": None,
        "notes": ["Manually supplied floor ROI is not autonomous free-space evidence.",
                  "Metric scale and base axes depend on the supplied candidate mount; unmeasured mount offsets, yaw and roll remain uncertain.",
                  "media_time is browser presentation time, not exact sensor exposure; host pulse phases do not establish exposure-aligned motion timing.",
                  "Stable final images do not establish braking time, distance, or a safe driving speed.",
                  "No failed frame is bridged and this report never populates HardwareConfig calibration tables."]}
    failed = False
    origin_time = records[0]["media_time"]
    for record in records:
        frame = {key: value for key, value in record.items() if key != "rgb"}
        if failed:
            frame.update(tracking=False, inliers=0, features=0, reprojection_error_px=None,
                         pose=None, reason="not_processed_after_tracking_failure")
        else:
            rgb = cv2.remap(record["rgb"], rectifier._map1, rectifier._map2,
                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            result = vo.update(rgb, record["media_time"] - origin_time, ground_mask=mask)
            frame.update(tracking=result.tracking, inliers=result.inliers, features=result.features,
                         reprojection_error_px=result.reprojection_error if math.isfinite(result.reprojection_error) else None,
                         reason=result.reason, pose={"x_m": result.pose.x, "y_m": result.pose.y,
                                                   "yaw_rad": result.pose.yaw} if result.tracking else None)
            if not result.tracking:
                failed = True
                report["quality_blockers"].append(f"tracking_failed_at_frame_{record['index']}:{result.reason}")
        report["frames"].append(frame)
    if failed:
        return report
    before = [frame for frame in report["frames"] if frame["phase"] == "before"]
    after = [frame for frame in report["frames"] if frame["phase"] == "after"]
    for label, window in (("baseline", before), ("settling", after)):
        duration = window[-1]["media_time"] - window[0]["media_time"] if window else 0.
        if len(window) < minimum_window_frames or duration + 1e-9 < minimum_window_s:
            report["quality_blockers"].append(f"insufficient_{label}_frames_or_duration")
    if report["quality_blockers"]:
        return report
    baseline_translation, baseline_yaw = _spread([frame["pose"] for frame in before])
    translation_threshold = max(.01, 3. * baseline_translation)
    yaw_threshold = max(math.radians(2.), 3. * baseline_yaw)
    report["baseline"] = {"count": len(before), "duration_s": before[-1]["media_time"] - before[0]["media_time"],
        "translation_jitter_diameter_m": baseline_translation, "yaw_jitter_diameter_rad": baseline_yaw,
        "minimum_resolvable_translation_m": translation_threshold,
        "minimum_resolvable_yaw_rad": yaw_threshold, "pose": _centre([frame["pose"] for frame in before])}
    # The final tail must include enough frames AND a measured media-time span.
    start = len(after) - minimum_window_frames
    while start > 0 and after[-1]["media_time"] - after[start]["media_time"] < minimum_window_s - 1e-9:
        start -= 1
    tail = after[start:]
    settled_translation, settled_yaw = _spread([frame["pose"] for frame in tail])
    stable = settled_translation <= translation_threshold / 3. and settled_yaw <= yaw_threshold / 3.
    report["settling"] = {"count": len(tail), "duration_s": tail[-1]["media_time"] - tail[0]["media_time"],
        "translation_spread_m": settled_translation, "yaw_spread_rad": settled_yaw,
        "visually_stable_final_window": stable, "stop_timing_verified": False,
        "pose": _centre([frame["pose"] for frame in tail])}
    if not stable:
        report["quality_blockers"].append("final_window_not_visually_settled")
        return report
    pre, post = report["baseline"]["pose"], report["settling"]["pose"]
    c, s = math.cos(pre["yaw_rad"]), math.sin(pre["yaw_rad"])
    dx, dy = post["x_m"] - pre["x_m"], post["y_m"] - pre["y_m"]
    dx, dy = c * dx + s * dy, -s * dx + c * dy
    yaw = wrap_angle(post["yaw_rad"] - pre["yaw_rad"])
    distance = math.hypot(dx, dy)
    translation_resolved, yaw_resolved = distance > translation_threshold, abs(yaw) > yaw_threshold
    direction = "not_resolved"
    if translation_resolved:
        direction = ("forward" if dx > 0 else "backward") if abs(dx) >= abs(dy) else ("left" if dy > 0 else "right")
    rotation = "left" if yaw > 0 else "right"
    elapsed = float(np.median([frame["media_time"] for frame in tail])
                    - np.median([frame["media_time"] for frame in before]))
    report["response"] = {"verified": False, "camera_relative_mount_uncertain": True,
        "translation_resolved": translation_resolved, "yaw_resolved": yaw_resolved,
        "direction": direction, "rotation_direction": rotation if yaw_resolved else "not_resolved",
        "delta_forward_m": dx, "delta_left_m": dy, "translation_m": distance, "delta_yaw_rad": yaw,
        "provisional_media_interval_s": elapsed,
        "provisional_window_average_translation_m_s": distance / elapsed if translation_resolved else None,
        "provisional_window_average_yaw_rad_s": yaw / elapsed if yaw_resolved else None,
        "average_note": "Net displacement divided by baseline-to-final presentation interval, including stationary windows; NOT pulse velocity or a speed calibration."}
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("camera_profile", type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--clear-floor-verified", action="store_true", help="Manual diagnostic confirmation only; never autonomous floor proof")
    group.add_argument("--ground-roi", type=Path, help="Boolean .npy mask in rectified output pixels")
    parser.add_argument("--exclude-roi", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = analyze_motion(args.manifest, args.camera_profile,
            ground_roi=np.load(args.ground_roi, allow_pickle=False) if args.ground_roi else None,
            clear_floor_verified=args.clear_floor_verified,
            roi_exclusions=np.load(args.exclude_roi, allow_pickle=False) if args.exclude_roi else None)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    except (MotionAnalysisError, ValueError, OSError) as error:
        parser.error(str(error))
    print(json.dumps({"verified": False, "quality_blockers": report["quality_blockers"],
                      "response": report["response"], "output": str(args.output)}, allow_nan=False))
    return 0 if not report["quality_blockers"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
