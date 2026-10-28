"""Offline calibration tools for saved phone JPEGs and physical mount measures.

There is no network, motor transport, arming, or simulator dependency here.
Every generated camera profile remains UNVERIFIED and lacks timing validation.
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


MIN_VIEWS = 15


class CalibrationError(ValueError):
    """The saved observations cannot support a credible intrinsic calibration."""


def _finite(value, name, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or positive and value <= 0.):
        raise CalibrationError(f"{name} must be a finite {'positive ' if positive else ''}number")
    return float(value)


def _dimension(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or not 32 <= value <= 8192:
        raise CalibrationError(f"{name} must be an integer from 32 to 8192")
    return value


def normalize_capture_settings(settings):
    """Pin optical/source settings without freezing automatic illumination.

    Continuous exposure and white balance report changing control values. Their
    modes stay pinned, while those automatic values are omitted. Focus mode and
    every reported focus distance remain strict because focusing can change K.
    Unknown processing fields also remain pinned rather than being discarded.
    Browser group_id is a document-scoped association, not an optical setting;
    it remains in the original capture diagnostics but is never persisted here.
    This normalization does not establish autofocus or stabilization validity.
    """
    if not isinstance(settings, dict):
        raise CalibrationError("Source settings must be an object")
    try:
        # Validate before omitting diagnostics; malformed fields do not acquire
        # an exception merely because they are outside optical equality.
        json.dumps(settings, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise CalibrationError("Source settings must be finite JSON") from error
    result = dict(settings)
    result.pop("group_id", None)
    if result.get("exposure_mode") == "continuous":
        result.pop("exposure_time", None)
        result.pop("exposure_compensation", None)
    if result.get("white_balance_mode") == "continuous":
        result.pop("color_temperature", None)
    return result


def _geometry(metadata):
    if not isinstance(metadata, dict):
        raise CalibrationError("Capture metadata must be an object")
    if isinstance(metadata.get("metadata"), dict):
        metadata = metadata["metadata"]
    dimensions = tuple(_dimension(metadata.get(name), name) for name in
                       ("source_width", "source_height", "width", "height"))
    sw, sh, width, height = dimensions
    if (sw * sh > 20_000_000 or width * height > 2_000_000 or width > sw or height > sh
            or width * sh != height * sw):
        raise CalibrationError("Saved JPEG must be a bounded full-frame aspect-preserving resize")
    if (metadata.get("rotation_degrees") != 0 or metadata.get("mirrored") is not False
            or metadata.get("crop") != "none"):
        raise CalibrationError("Calibration captures must have no rotation, mirror, or crop")
    settings = metadata.get("source_settings", metadata.get("settings"))
    if (not isinstance(settings, dict) or not isinstance(settings.get("device_id"), str)
            or not settings["device_id"] or settings.get("width") != sw or settings.get("height") != sh):
        raise CalibrationError("Source settings must identify the exact device and full source resolution")
    try:
        # Validate even omitted diagnostics: no malformed/nonfinite data is
        # silently excused by an automatic illumination mode.
        json.dumps(settings, allow_nan=False)
        canonical = json.dumps(normalize_capture_settings(settings), sort_keys=True,
                               separators=(",", ":"), allow_nan=False)
    except (TypeError, ValueError) as error:
        raise CalibrationError("Source settings must be finite JSON") from error
    return dimensions, canonical


def _validate_recorded_target(metadata, *, columns, rows, square_size_m, image_name):
    """Respect a measured display target saved with each capture.

    Older captures have no target entry and still require the explicit CLI
    measurement. A present entry is authoritative evidence and cannot silently
    disagree with the requested board or physical square size.
    """
    if isinstance(metadata.get("metadata"), dict):
        metadata = metadata["metadata"]
    if "calibration_target" not in metadata:
        return
    target = metadata["calibration_target"]
    if not isinstance(target, dict):
        raise CalibrationError(f"Invalid calibration_target at {image_name}")
    for name, expected in (("columns", columns), ("rows", rows)):
        value = target.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value != expected:
            raise CalibrationError(
                f"Recorded calibration_target {name} disagrees with the requested board at {image_name}")
    measured_cm = _finite(target.get("square_size_cm"),
                          f"calibration_target square_size_cm at {image_name}", positive=True)
    if not math.isclose(measured_cm / 100., square_size_m, rel_tol=1e-6, abs_tol=1e-9):
        raise CalibrationError(
            f"Recorded calibration_target square size disagrees with square_size_m at {image_name}; "
            "use captures with one measured square size and the matching CLI measurement")


def detect_chessboard(pixels, columns, rows):
    """Find subpixel internal corners in the actual saved JPEG pixels."""
    gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY)
    pattern = (columns, rows)
    found, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, .001))
    return np.asarray(corners, dtype=np.float32).reshape(-1, 2) if found else None


def _view_geometry(corners, size, columns, rows):
    width, height = size
    normalized = corners / np.array([width, height])
    board = corners.reshape(rows, columns, 2)
    tl, tr, bl, br = board[0, 0], board[0, -1], board[-1, 0], board[-1, -1]
    left, right = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    top, bottom = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    if min(left, right, top, bottom) < 5.:
        raise CalibrationError("Detected chessboard is too small")
    return {"centre": normalized.mean(axis=0),
            "coverage": cv2.contourArea(cv2.convexHull(corners.astype(np.float32))) / (width * height),
            "perspective": max(abs(math.log(left / right)), abs(math.log(top / bottom))),
            "normalized": normalized}


def _diversity(views, *, label):
    centres = np.asarray([view["geometry"]["centre"] for view in views])
    coverage = np.asarray([view["geometry"]["coverage"] for view in views])
    tilted = sum(view["geometry"]["perspective"] >= .08 for view in views)
    spread = np.ptp(centres, axis=0)
    ratio = float(coverage.max() / coverage.min())
    failures = []
    if spread[0] < .30 or spread[1] < .25:
        failures.append("move the target across the image centre, sides, top and bottom")
    if ratio < 1.8:
        failures.append("capture the target at different distances and image sizes")
    if tilted < 4:
        failures.append("capture at least four clearly tilted target views in different directions")
    if failures:
        raise CalibrationError(f"{label} views lack diversity: " + "; ".join(failures))
    return {"centre_span_fraction": spread.tolist(), "coverage_ratio": ratio,
            "tilted_view_count": tilted}


def output_to_source_K(K, *, source_width, source_height, width, height):
    """Undo full-frame cv2/browser resize in pixel-centre coordinates."""
    if source_width * height != source_height * width or width > source_width or height > source_height:
        raise CalibrationError("Only full-frame aspect-preserving downscaling is supported")
    scale = width / source_width
    result = np.array(K, dtype=float, copy=True)
    if result.shape != (3, 3) or not np.isfinite(result).all():
        raise CalibrationError("Intrinsic matrix must be finite 3x3")
    result[0, 0] /= scale
    result[1, 1] /= scale
    result[0, 2] = (result[0, 2] + .5) / scale - .5
    result[1, 2] = (result[1, 2] + .5) / scale - .5
    return result


def _reprojection_error(objects, corners, K, distortion, rvec=None, tvec=None):
    if rvec is None:
        success, rvec, tvec = cv2.solvePnP(objects, corners, K, distortion,
                                         flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            raise CalibrationError("Held-out target pose could not be estimated")
    projected, _ = cv2.projectPoints(objects, rvec, tvec, K, distortion)
    return float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners) ** 2, axis=1))))


def calibrate_intrinsics(folder, *, columns, rows, square_size_m, camera_id):
    """Return an unverified profile fitted to saved chessboards, without writing.

    At least 15 distinct diverse detections are required. Every fifth accepted
    view is held out and never used to fit the returned K/distortion. Quality
    errors are reported in saved-JPEG pixels, before conversion of K to source
    pixels. Bad residuals remain explicit blockers, never a verification flag.
    """
    if (isinstance(columns, bool) or isinstance(rows, bool) or not isinstance(columns, int)
            or not isinstance(rows, int) or not 3 <= columns <= 30 or not 3 <= rows <= 30):
        raise CalibrationError("Chessboard inner-corner columns and rows must be integers 3..30")
    square_size_m = _finite(square_size_m, "square_size_m", positive=True)
    if not isinstance(camera_id, str) or not 1 <= len(camera_id.strip()) <= 200:
        raise CalibrationError("camera_id must identify the measured phone and rear lens")
    folder = Path(folder)
    if not folder.is_dir():
        raise CalibrationError("Capture folder does not exist")
    files = sorted(path for path in folder.iterdir() if path.suffix.lower() in {".jpg", ".jpeg"})
    if len(files) < MIN_VIEWS:
        raise CalibrationError(f"At least {MIN_VIEWS} saved JPEGs with matching JSON metadata are required")
    geometry = None
    views, excluded, hashes = [], [], set()
    for path in files:
        metadata_path = path.with_suffix(".json")
        try:
            metadata = json.loads(metadata_path.read_text())
        except (OSError, ValueError) as error:
            raise CalibrationError(f"Missing or invalid capture metadata: {metadata_path.name}") from error
        current = _geometry(metadata)
        _validate_recorded_target(metadata, columns=columns, rows=rows,
                                  square_size_m=square_size_m, image_name=path.name)
        if geometry is None:
            geometry = current
        elif current != geometry:
            raise CalibrationError(f"Camera source, resolution or settings changed at {path.name}; use one fixed capture mode")
        dimensions, _ = current
        sw, sh, width, height = dimensions
        data = path.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if digest in hashes:
            excluded.append({"image": path.name, "reason": "duplicate_jpeg_bytes"})
            continue
        hashes.add(digest)
        try:
            with Image.open(io.BytesIO(data)) as image:
                if (image.format != "JPEG" or image.size != (width, height)
                        or image.getexif().get(274, 1) != 1):
                    raise CalibrationError(f"JPEG geometry or orientation disagrees with metadata: {path.name}")
                pixels = np.asarray(image.convert("RGB"))
        except (OSError, ValueError) as error:
            if isinstance(error, CalibrationError):
                raise
            raise CalibrationError(f"Invalid JPEG: {path.name}") from error
        corners = detect_chessboard(pixels, columns, rows)
        if corners is None:
            excluded.append({"image": path.name, "reason": "complete_chessboard_not_detected"})
            continue
        if (corners.shape != (columns * rows, 2) or not np.isfinite(corners).all()
                or np.any(corners < 3.) or np.any(corners[:, 0] > width - 4)
                or np.any(corners[:, 1] > height - 4)):
            excluded.append({"image": path.name, "reason": "invalid_or_edge_clipped_corners"})
            continue
        descriptor = _view_geometry(corners, (width, height), columns, rows)
        if descriptor["coverage"] < .015:
            excluded.append({"image": path.name, "reason": "chessboard_coverage_too_small"})
            continue
        if any(np.sqrt(np.mean(np.sum((descriptor["normalized"] - view["geometry"]["normalized"]) ** 2,
                                      axis=1))) < .012 for view in views):
            excluded.append({"image": path.name, "reason": "repeated_chessboard_pose"})
            continue
        views.append({"image": path.name, "sha256": digest, "corners": corners,
                      "geometry": descriptor})
    if len(views) < MIN_VIEWS:
        raise CalibrationError(f"Only {len(views)} distinct usable chessboards; need at least {MIN_VIEWS}")
    diversity = _diversity(views, label="All")
    training = [view for index, view in enumerate(views) if index % 5 != 4]
    heldout = [view for index, view in enumerate(views) if index % 5 == 4]
    _diversity(training, label="Training")
    objects = np.zeros((columns * rows, 3), dtype=np.float32)
    objects[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    try:
        rms, K, distortion, rotations, translations = cv2.calibrateCamera(
            [objects.copy() for _ in training], [view["corners"] for view in training],
            (width, height), None, None,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    except cv2.error as error:
        raise CalibrationError("OpenCV intrinsic calibration did not converge") from error
    distortion = np.asarray(distortion).reshape(-1)
    if (not math.isfinite(rms) or not np.isfinite(K).all() or not np.isfinite(distortion).all()
            or K[0, 0] <= 0 or K[1, 1] <= 0 or not 0 <= K[0, 2] < width
            or not 0 <= K[1, 2] < height):
        raise CalibrationError("Calibration produced inadmissible intrinsics; collect better target views")
    fitted, validated = [], []
    for view, rvec, tvec in zip(training, rotations, translations):
        fitted.append({"image": view["image"], "sha256": view["sha256"],
                       "rms_px": _reprojection_error(objects, view["corners"], K, distortion, rvec, tvec)})
    try:
        for view in heldout:
            validated.append({"image": view["image"], "sha256": view["sha256"],
                "rms_px": _reprojection_error(objects, view["corners"], K, distortion)})
    except cv2.error as error:
        raise CalibrationError("Held-out target validation failed") from error
    holdout_rms = float(np.sqrt(np.mean([view["rms_px"] ** 2 for view in validated])))
    blockers = []
    if rms > 1.:
        blockers.append("training reprojection RMS exceeds 1.0 saved-image pixel")
    if holdout_rms > 1.5:
        blockers.append("held-out reprojection RMS exceeds 1.5 saved-image pixels")
    if max(view["rms_px"] for view in fitted + validated) > 2.:
        blockers.append("a target view exceeds 2.0 saved-image pixels RMS")
    source_K = output_to_source_K(K, source_width=sw, source_height=sh, width=width, height=height)
    return {"version": 1, "camera_id": camera_id.strip(), "source_width": sw, "source_height": sh,
            "width": width, "height": height, "K": source_K.tolist(),
            "distortion": distortion.tolist(), "T_base_camera": None,
            "settings": json.loads(geometry[1]), "verified": False,
            "timing": {"max_pipeline_age_s": None, "validated": False},
            "intrinsics_report": {
                "target": {"inner_columns": columns, "inner_rows": rows, "square_size_m": square_size_m},
                "training_view_count": len(training), "heldout_view_count": len(heldout),
                "training_rms_px": float(rms), "heldout_rms_px": holdout_rms,
                "error_pixel_space": "saved_JPEG_before_source_K_conversion",
                "source_K_conversion": "full_frame_pixel_centres",
                "settings_policy": "pin_optics_modes_and_processing_omit_document_group_and_continuous_illumination_values",
                "heldout_policy": "every_fifth_distinct_usable_view_never_used_for_intrinsic_fit",
                "diversity": diversity, "quality_blockers": blockers,
                "training_views": fitted, "heldout_views": validated, "excluded_views": excluded},
            "notes": ["UNVERIFIED offline intrinsic fit; this profile cannot enable perception or motion.",
                      "T_base_camera is null until physical mount measurements are supplied and checked.",
                      "Held-out reprojection is a software residual, not physical calibration acceptance.",
                      "Continuous exposure/white-balance modes stay pinned; their automatic values are omitted. Focus remains strict.",
                      "Browser group_id remains in raw captures but is omitted from persisted optical settings.",
                      "Timing remains unknown and unvalidated. Never substitute simulator measurements."]}


def mount_transform(*, height_m, forward_m, left_m, pitch_down_deg, yaw_deg, roll_deg):
    """Measured optical-to-base rigid transform, without any assumed values.

    Base axes: forward/left/up. Optical axes: right/down/forward. Positive yaw
    turns left; positive pitch points down. Roll is right-handed about the
    horizontal forward axis before pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll) Roptical.
    This only converts measured geometry; the final camera profile separately
    checks whether the rectified image sees the required initial floor patch.
    """
    height = _finite(height_m, "height_m", positive=True)
    forward, left = _finite(forward_m, "forward_m"), _finite(left_m, "left_m")
    pitch, yaw, roll = (_finite(value, name) * math.pi / 180. for value, name in
                        ((pitch_down_deg, "pitch_down_deg"), (yaw_deg, "yaw_deg"), (roll_deg, "roll_deg")))
    if height < .03 or height > 2.:
        raise CalibrationError("Mount height must be between 0.03 and 2.0 metres")
    cp, sp, cy, sy, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    Ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    Rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    Rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    result = np.eye(4)
    result[:3, :3] = Rz @ Ry @ Rx @ optical
    result[:3, 3] = [forward, left, height]
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    intrinsic = commands.add_parser("intrinsics", help="Fit unverified K/distortion from saved chessboards")
    intrinsic.add_argument("folder", type=Path)
    intrinsic.add_argument("--columns", type=int, required=True, help="Internal chessboard corners across")
    intrinsic.add_argument("--rows", type=int, required=True, help="Internal chessboard corners down")
    intrinsic.add_argument("--square-size-m", type=float, required=True, help="Measured square side in metres")
    intrinsic.add_argument("--camera-id", required=True)
    mount = commands.add_parser("mount", help="Convert all physically measured mount values to a transform")
    for name in ("height-m", "forward-m", "left-m", "pitch-down-deg", "yaw-deg", "roll-deg"):
        mount.add_argument("--" + name, type=float, required=True)
    for command in (intrinsic, mount):
        command.add_argument("--output", type=Path, help="New JSON file; otherwise write JSON to stdout")
        command.add_argument("--force", action="store_true", help="Explicitly allow replacing output JSON")
    args = parser.parse_args(argv)
    try:
        if args.output and args.output.exists() and not args.force:
            raise CalibrationError("Output exists; choose a new file or explicitly use --force")
        if args.command == "intrinsics":
            result = calibrate_intrinsics(args.folder, columns=args.columns, rows=args.rows,
                square_size_m=args.square_size_m, camera_id=args.camera_id)
        else:
            result = {"T_base_camera": mount_transform(**{name: getattr(args, name) for name in
                ("height_m", "forward_m", "left_m", "pitch_down_deg", "yaw_deg", "roll_deg")}).tolist()}
        rendered = json.dumps(result, indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.write_text(rendered)
        else:
            print(rendered, end="")
        return 0
    except (CalibrationError, OSError, ValueError) as error:
        parser.exit(2, f"Calibration failed: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
