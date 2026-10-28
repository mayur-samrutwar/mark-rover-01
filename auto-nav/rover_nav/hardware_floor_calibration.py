"""Offline floor-marker mount estimation; no camera, network, or motor access.

Four measured floor markers determine an optical-camera-to-rover transform. A
fifth, independently clicked marker is a holdout and never enters pose fitting
or candidate selection. Every result remains unverified physical evidence.
"""
from __future__ import annotations

import math
from numbers import Real

import cv2
import numpy as np

from .hardware_calibration import CalibrationError


MARKER_ORDER = ["near_left", "near_right", "far_right", "far_left", "heldout_centre"]
THRESHOLDS = {
    "training_rms_px": 1.,
    "training_max_px": 1.,
    "heldout_error_px": 2.,
    "ambiguity_absolute_rms_px": .25,
    "ambiguity_relative_rms": 1.5,
    "distinct_position_m": .005,
    "distinct_rotation_deg": .5,
    "minimum_image_area_px2": 100.,
    "minimum_image_edge_px": 5.,
    "minimum_image_singular_ratio": .03,
    "minimum_geometric_seed_pixels": 100,
}


def _number(value, name, *, minimum=None, maximum=None):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
        raise CalibrationError(f"{name} must be a finite number")
    result = float(value)
    if minimum is not None and result < minimum or maximum is not None and result > maximum:
        raise CalibrationError(f"{name} is outside its supported range")
    return result


def _dimension(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or not 32 <= value <= 8192:
        raise CalibrationError(f"{name} must be an integer from 32 to 8192")
    return value


def _geometry(value):
    if not isinstance(value, dict):
        raise CalibrationError("Measured floor geometry must be an object")
    result = {name: _number(value.get(name), name, minimum=minimum, maximum=maximum)
              for name, minimum, maximum in (
                  ("near_x_m", .05, 5.), ("far_x_m", .10, 5.),
                  ("half_width_m", .025, 1.),
                  ("max_camera_offset_x_m", .01, 1.),
                  ("max_camera_offset_y_m", .01, 1.))}
    if result["far_x_m"] - result["near_x_m"] < .05:
        raise CalibrationError("Far markers must be at least 0.05 m beyond near markers")
    result["height_tolerance_m"] = _number(value.get("height_tolerance_m", .04),
                                            "height_tolerance_m", minimum=.005, maximum=.10)
    near, far, half = (result[key] for key in ("near_x_m", "far_x_m", "half_width_m"))
    objects = np.array([[near, half, 0.], [near, -half, 0.],
                        [far, -half, 0.], [far, half, 0.],
                        [(near + far) / 2., 0., 0.]], dtype=np.float64)
    return result, objects


def image_intrinsics(camera_profile, image_size):
    """Scale source K to full-frame raw JPEG pixel centres, without undistorting.

    The caller must supply clicks in the original, unmirrored JPEG. OpenCV PnP
    receives distortion once; undistorted preview coordinates are unsupported.
    """
    if not isinstance(camera_profile, dict):
        raise CalibrationError("Camera profile must be an object")
    try:
        width, height = image_size
    except (TypeError, ValueError) as error:
        raise CalibrationError("image_size must contain width and height") from error
    width, height = _dimension(width, "image width"), _dimension(height, "image height")
    sw = _dimension(camera_profile.get("source_width"), "source_width")
    sh = _dimension(camera_profile.get("source_height"), "source_height")
    if width > sw or height > sh or width * sh != height * sw:
        raise CalibrationError("Image must be a full-frame aspect-preserving source downscale")
    if sw * sh > 20_000_000 or width * height > 2_000_000:
        raise CalibrationError("Calibration image exceeds the supported pixel count")
    try:
        K = np.asarray(camera_profile.get("K"), dtype=np.float64)
        distortion = np.asarray(camera_profile.get("distortion"), dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise CalibrationError("Camera intrinsics must be finite numeric arrays") from error
    if (K.shape != (3, 3) or not np.isfinite(K).all()
            or not np.allclose(K[2], [0., 0., 1.], atol=1e-9, rtol=0.)
            or abs(K[0, 1]) > 1e-9 or abs(K[1, 0]) > 1e-9
            or K[0, 0] <= 0. or K[1, 1] <= 0.
            or not 0. <= K[0, 2] < sw or not 0. <= K[1, 2] < sh):
        raise CalibrationError("K must be a measured zero-skew pinhole matrix in source pixels")
    if distortion.shape not in {(4,), (5,), (8,)} or not np.isfinite(distortion).all():
        raise CalibrationError("OpenCV distortion needs 4, 5, or 8 finite coefficients")
    K = K.copy()
    scale = width / sw
    K[0, 0] *= scale
    K[1, 1] *= scale
    K[0, 2] = (K[0, 2] + .5) * scale - .5
    K[1, 2] = (K[1, 2] + .5) * scale - .5
    return K, distortion.copy()


def _points(value, image_size):
    try:
        raw = np.asarray(value, dtype=object)
        if raw.shape != (5, 2):
            raise ValueError("wrong coordinate count")
        points = np.array([[_number(coordinate, "pixel coordinate") for coordinate in point]
                           for point in raw], dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise CalibrationError("Five raw JPEG pixel coordinates are required") from error
    if points.shape != (5, 2) or not np.isfinite(points).all():
        raise CalibrationError("Five finite (x, y) raw JPEG pixel coordinates are required")
    width, height = image_size
    if (points < 0.).any() or (points[:, 0] > width - 1.).any() or (points[:, 1] > height - 1.).any():
        raise CalibrationError("Every marker click must lie inside the raw JPEG")
    corners = points[:4]
    edges = np.roll(corners, -1, axis=0) - corners
    signed_area = float(.5 * np.sum(corners[:, 0] * np.roll(corners[:, 1], -1)
                                   - corners[:, 1] * np.roll(corners[:, 0], -1)))
    if not cv2.isContourConvex(corners.astype(np.float32)) or signed_area >= 0.:
        raise CalibrationError("Marker order must be near-left, near-right, far-right, far-left in an unmirrored view")
    singular = np.linalg.svd(corners - corners.mean(axis=0), compute_uv=False)
    if (abs(signed_area) < THRESHOLDS["minimum_image_area_px2"]
            or np.linalg.norm(edges, axis=1).min() < THRESHOLDS["minimum_image_edge_px"]
            or singular[-1] < 2.
            or singular[-1] / singular[0] < THRESHOLDS["minimum_image_singular_ratio"]):
        raise CalibrationError("Floor markers are too small or nearly collinear; use a larger, clearly visible rectangle")
    return points


def _pixel_rays(K, distortion, image_size):
    width, height = image_size
    yy, xx = np.mgrid[:height, :width]
    raw_pixels = np.column_stack((xx.ravel(), yy.ravel())).astype(np.float64)
    normalized = cv2.undistortPoints(raw_pixels.reshape(-1, 1, 2), K, distortion).reshape(-1, 2)
    return np.column_stack((normalized, np.ones(len(normalized))))


def _floor_patch_visibility(transform, optical_rays, image_size):
    """Count geometric rays into the seed region; never infer floor appearance."""
    rays = optical_rays @ transform[:3, :3].T
    downward = np.isfinite(rays).all(axis=1) & (rays[:, 2] < -1e-5)
    scale = np.full(len(rays), np.nan)
    scale[downward] = -transform[2, 3] / rays[downward, 2]
    relative = rays[:, :2] * scale[:, None]
    distance = np.linalg.norm(relative, axis=1)
    bearing = np.arctan2(relative[:, 1], relative[:, 0])
    seed = (downward & (distance >= .3) & (distance <= .7)
            & (np.abs(bearing) < .32) & (relative[:, 0] > 0.))
    count = int(seed.sum())
    return {"geometry_only": True, "floor_appearance_or_texture_validated": False,
            "pixel_space": "raw_JPEG_rays_with_measured_distortion",
            "image_size": list(image_size), "total_pixels": len(rays),
            "downward_ray_pixels": int(downward.sum()),
            "downward_ray_fraction": float(downward.mean()),
            "seed_pixel_count": count, "seed_pixel_fraction": float(seed.mean()),
            "required_seed_pixel_count": THRESHOLDS["minimum_geometric_seed_pixels"],
            "seed_visible": count >= THRESHOLDS["minimum_geometric_seed_pixels"],
            "seed_range_m": [.3, .7], "seed_absolute_bearing_rad": .32,
            "seed_range_origin": "camera_XY_position_in_base_axes"}


def floor_patch_visibility(T_base_camera, K_image, distortion, image_size):
    """Return raw-image floor-patch geometry, independent of image appearance.

    This diagnostic is not a perception acceptance test: the final undistorted
    perception view must pass its own geometric and texture checks as well.
    """
    try:
        width, height = image_size
        transform = np.asarray(T_base_camera, dtype=np.float64)
        K = np.asarray(K_image, dtype=np.float64)
        distortion = np.asarray(distortion, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise CalibrationError("Floor visibility requires numeric camera arrays and image dimensions") from error
    width, height = _dimension(width, "image width"), _dimension(height, "image height")
    if width * height > 2_000_000:
        raise CalibrationError("Calibration image exceeds the supported pixel count")
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or transform[2, 3] <= 0.
            or not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5, rtol=0.)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-5, rtol=0.)
            or K.shape != (3, 3) or not np.isfinite(K).all()
            or not np.allclose(K[2], [0., 0., 1.], atol=1e-9, rtol=0.)
            or K[0, 0] <= 0. or K[1, 1] <= 0. or abs(K[0, 1]) > 1e-9 or abs(K[1, 0]) > 1e-9
            or distortion.shape not in {(4,), (5,), (8,)} or not np.isfinite(distortion).all()):
        raise CalibrationError("Floor visibility requires finite camera intrinsics and a mount above the floor")
    return _floor_patch_visibility(transform, _pixel_rays(K, distortion, (width, height)), (width, height))


def _candidate(index, rvec, tvec, objects, pixels, K, distortion, geometry, measured_height_m,
               optical_rays, image_size):
    rvec, tvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1), np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    refined = False
    # The centre marker is intentionally excluded here, including from LM.
    try:
        rvec, tvec = cv2.solvePnPRefineLM(objects[:4], pixels[:4], K, distortion,
                                         rvec.copy(), tvec.copy(),
                                         criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                                                   50, 1e-10))
        refined = True
    except cv2.error:
        pass
    R_camera_base, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4)
    transform[:3, :3] = R_camera_base.T
    transform[:3, 3] = (-R_camera_base.T @ tvec).reshape(3)
    projected, _ = cv2.projectPoints(objects, rvec, tvec, K, distortion)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - pixels, axis=1)
    camera_points = (R_camera_base @ objects.T + tvec).T
    position = transform[:3, 3]
    optical_forward = transform[:3, 2]
    physical = []
    if not np.isfinite(transform).all() or not np.isfinite(errors).all():
        raise ValueError("nonfinite_pose_or_reprojection")
    if not np.all(camera_points[:, 2] > .01):
        physical.append("floor_markers_not_in_front_of_camera")
    if not .03 <= position[2] <= 2.:
        physical.append("camera_height_not_between_0.03_and_2_m")
    if optical_forward[0] <= .02:
        physical.append("camera_optical_axis_not_forward")
    if abs(position[0]) > geometry["max_camera_offset_x_m"]:
        physical.append("camera_forward_offset_outside_supplied_envelope")
    if abs(position[1]) > geometry["max_camera_offset_y_m"]:
        physical.append("camera_lateral_offset_outside_supplied_envelope")
    height_difference = None if measured_height_m is None else abs(float(position[2]) - measured_height_m)
    if height_difference is not None and height_difference > geometry["height_tolerance_m"]:
        physical.append("estimated_height_disagrees_with_independent_measurement")
    patch = _floor_patch_visibility(transform, optical_rays, image_size)
    if not patch["seed_visible"]:
        physical.append("required_initial_floor_patch_not_geometrically_visible")
    return {"index": index, "T_base_camera": transform.tolist(), "position_base_m": position.tolist(),
            "optical_forward_base": optical_forward.tolist(), "refined_using_training_only": refined,
            "pitch_down_deg": math.degrees(math.atan2(-optical_forward[2], np.linalg.norm(optical_forward[:2]))),
            "yaw_left_deg": math.degrees(math.atan2(optical_forward[1], optical_forward[0])),
            "floor_patch_visibility": patch,
            "training_rms_px": float(np.sqrt(np.mean(errors[:4] ** 2))),
            "training_max_px": float(errors[:4].max()),
            "training_errors_px": errors[:4].tolist(), "heldout_error_px": float(errors[4]),
            "projected_image_points": projected.tolist(),
            "measured_height_difference_m": height_difference,
            "physical_blockers": physical, "physically_plausible": not physical}


def _same_pose(first, second):
    a, b = np.asarray(first["T_base_camera"]), np.asarray(second["T_base_camera"])
    cosine = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1.) / 2., -1., 1.)
    return (np.linalg.norm(a[:3, 3] - b[:3, 3]) < THRESHOLDS["distinct_position_m"]
            and math.degrees(math.acos(float(cosine))) < THRESHOLDS["distinct_rotation_deg"])


def solve_floor_mount(camera_profile, image_points5, geometry, image_size, measured_height_m=None):
    """Estimate an UNVERIFIED mount from five physically measured floor markers.

    ``geometry`` requires near_x_m, far_x_m, half_width_m and explicitly supplied
    max_camera_offset_x_m/max_camera_offset_y_m. Coordinates are relative to the
    axle-midpoint ground origin: +X forward, +Y left, +Z up. Click order is
    near-left, near-right, far-right, far-left, then the independent centre at
    ((near_x_m + far_x_m) / 2, 0). The fifth point cannot affect the fitted pose.

    ``image_size`` is raw JPEG (width, height); clicks must not be undistorted,
    rotated or mirrored. PnP receives source K scaled to that exact image and the
    measured distortion exactly once. An accepted numeric fit does not verify
    the physical measurements, timing, tracking, or authorize motion.
    """
    K, distortion = image_intrinsics(camera_profile, image_size)
    geometry, objects = _geometry(geometry)
    points = _points(image_points5, image_size)
    if measured_height_m is not None:
        measured_height_m = _number(measured_height_m, "measured_height_m", minimum=.03, maximum=2.)
    optical_rays = _pixel_rays(K, distortion, image_size)
    blockers = []
    intrinsics_report = camera_profile.get("intrinsics_report", {})
    if isinstance(intrinsics_report, dict) and intrinsics_report.get("quality_blockers"):
        blockers.append("intrinsics_report_contains_quality_blockers")
    try:
        solved = cv2.solvePnPGeneric(objects[:4], points[:4], K, distortion,
                                     flags=cv2.SOLVEPNP_IPPE)
    except cv2.error as error:
        raise CalibrationError("Floor-marker pose estimation failed; check marker positions and click order") from error
    candidates = []
    if solved[0]:
        for index, (rotation, translation) in enumerate(zip(solved[1], solved[2])):
            try:
                candidates.append(_candidate(index, rotation, translation, objects, points, K,
                                               distortion, geometry, measured_height_m, optical_rays, image_size))
            except (cv2.error, ValueError, np.linalg.LinAlgError):
                blockers.append(f"candidate_{index}_could_not_be_evaluated")
    if not candidates:
        blockers.append("no_planar_pose_candidates")
    physical = sorted((candidate for candidate in candidates if candidate["physically_plausible"]),
                      key=lambda candidate: candidate["training_rms_px"])
    best = physical[0] if physical else None
    ambiguous = []
    if best is None:
        blockers.append("no_physically_plausible_camera_pose")
    else:
        for alternative in physical[1:]:
            if _same_pose(best, alternative):
                continue
            close_error = (alternative["training_rms_px"] <= best["training_rms_px"]
                           + THRESHOLDS["ambiguity_absolute_rms_px"]
                           or alternative["training_rms_px"] <= best["training_rms_px"]
                           * THRESHOLDS["ambiguity_relative_rms"])
            if close_error:
                ambiguous.append(alternative["index"])
        if ambiguous:
            blockers.append("ambiguous_planar_pose_candidates")
        if best["training_rms_px"] > THRESHOLDS["training_rms_px"]:
            blockers.append("training_reprojection_rms_exceeds_1_px")
        if best["training_max_px"] > THRESHOLDS["training_max_px"]:
            blockers.append("training_marker_error_exceeds_1_px")
        if best["heldout_error_px"] > THRESHOLDS["heldout_error_px"]:
            blockers.append("independent_heldout_error_exceeds_2_px")
    diagnostic = best or min(candidates, key=lambda candidate: candidate["training_rms_px"], default=None)
    if best is None and diagnostic is not None:
        blockers.extend(diagnostic["physical_blockers"])
    report = {"method": "IPPE_planar_candidates_then_training_only_LM",
              "base_axes": "+X_forward_+Y_left_+Z_up_origin_axle_midpoint_ground",
              "camera_axes": "OpenCV_+X_right_+Y_down_+Z_forward",
              "click_pixel_space": "raw_unmirrored_full_frame_JPEG_with_measured_distortion",
              "intrinsic_scaling": "source_to_JPEG_pixel_centres",
              "image_size": list(image_size), "K_image": K.tolist(), "distortion": distortion.tolist(),
              "marker_order": MARKER_ORDER.copy(), "geometry": geometry,
              "object_points_base_m": objects.tolist(), "image_points": points.tolist(),
              "training_point_indices": [0, 1, 2, 3], "heldout_point_index": 4,
              "heldout_used_for_pose_fit_or_selection": False,
              "thresholds": dict(THRESHOLDS), "measured_height_m": measured_height_m,
              "candidates": candidates, "ambiguous_alternative_indices": ambiguous,
              "selected_candidate_index": None if blockers or best is None else best["index"],
              "quality_blockers": blockers,
              "warnings": [] if measured_height_m is not None else ["independent_camera_height_crosscheck_missing"]}
    for name in ("training_rms_px", "training_max_px", "heldout_error_px", "measured_height_difference_m",
                 "pitch_down_deg", "yaw_left_deg", "floor_patch_visibility"):
        report[name] = None if diagnostic is None else diagnostic[name]
    return {"version": 1, "verified": False,
            "T_base_camera": None if blockers or best is None else best["T_base_camera"],
            "candidate_T_base_camera": None if diagnostic is None else diagnostic["T_base_camera"],
            "floor_mount_report": report,
            "notes": ["UNVERIFIED floor-marker estimate; this result cannot authorize motion.",
                      "The supplied footprint bounds are candidate checks, not measured mount coordinates.",
                      "Physical marker placement and camera height need independent validation.",
                      "Timing and floor tracking remain separate validation steps."]}
