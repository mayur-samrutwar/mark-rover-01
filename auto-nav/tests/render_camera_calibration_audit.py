"""Offline GL raster/calibration diagnostic; never imported by perception.

Requires a desktop graphics connection. Run with the Isaac Python and
PYTHONPATH=auto-nav/.deps:auto-nav. Targets exist only in an isolated diagnostic
scene, never in a navigation environment or the perception input pipeline.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np
from PIL import Image

from rover_nav.vision_isaac import VisionRover
from rover_nav.vision_scene import vision_robot_xml
from rover_nav.world import generate_world


ROOT = Path(__file__).resolve().parents[1]
CAD_TO_BASE = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
GL_TO_CV = np.diag([1., -1., -1.])


def _camera_record(camera):
    fields = ("pos", "forward", "up", "frustum_center", "frustum_bottom",
              "frustum_top", "frustum_near", "frustum_far", "frustum_width")
    record = {}
    for name in fields:
        value = getattr(camera, name)
        record[name] = value.tolist() if isinstance(value, np.ndarray) else float(value)
    return record


def _axis_angles(rotation):
    forward = rotation[:, 0]
    yaw = math.atan2(forward[1], forward[0])
    pitch = math.atan2(-forward[2], np.linalg.norm(forward[:2]))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    return {"yaw_degrees": math.degrees(yaw), "pitch_degrees": math.degrees(pitch),
            "roll_degrees": math.degrees(roll)}


def ideal_correspondence_sensitivity(K, nominal, settled):
    """Isolate calibration effects with exact projected floor correspondences.

    This is not image tracking or a mission. The 1080-degree number is a
    linear extrapolation with the same feature support repeated each degree.
    It must not be interpreted as a bound for arbitrary image distributions.
    """
    def ground(pixels, intrinsic, transform):
        homogeneous = np.column_stack((pixels, np.ones(len(pixels))))
        rays = homogeneous @ np.linalg.inv(intrinsic).T @ transform[:3, :3].T
        return (transform[:3, 3] + rays * (-transform[2, 3] / rays[:, 2])[:, None])[:, :2]
    delta = math.radians(1.)
    rotation = np.array([[math.cos(delta), -math.sin(delta), 0.],
                         [math.sin(delta), math.cos(delta), 0.], [0., 0., 1.]])
    cases = []
    for name, actual in (("nominal_mount", nominal), ("settled_mount_and_floor", settled)):
        yy, xx = np.mgrid[60:225:8, 20:301:8]
        pixels = np.column_stack((xx.ravel(), yy.ravel(), np.ones(xx.size)))
        rays = pixels @ np.linalg.inv(K).T @ actual[:3, :3].T
        scale = -actual[2, 3] / rays[:, 2]
        world = actual[:3, 3] + rays * scale[:, None]
        valid = (scale > 0) & (np.linalg.norm(world[:, :2], axis=1) < 2.5)
        world, pixels = world[valid], pixels[valid, :2]
        next_camera = actual.copy()
        next_camera[:3, :3] = rotation @ actual[:3, :3]
        next_camera[:3, 3] = rotation @ actual[:3, 3]
        camera_points = (world - next_camera[:3, 3]) @ next_camera[:3, :3]
        projected = camera_points @ K.T
        projected = projected[:, :2] / projected[:, 2, None]
        valid = ((projected[:, 0] > 3) & (projected[:, 0] < 317)
                 & (projected[:, 1] > 3) & (projected[:, 1] < 237))
        old, new = pixels[valid], projected[valid]
        for cx, cy in ((159.5, 119.5), (160., 120.)):
            assumed = K.copy()
            assumed[0, 2], assumed[1, 2] = cx, cy
            source, target = ground(old, assumed, nominal), ground(new, assumed, nominal)
            weights = 1. / (.2 + np.sum(source ** 2, axis=1))
            weights /= weights.sum()
            source_mean = np.sum(source * weights[:, None], axis=0)
            target_mean = np.sum(target * weights[:, None], axis=0)
            u, _, vt = np.linalg.svd((source - source_mean).T @
                                     ((target - target_mean) * weights[:, None]))
            fitted = vt.T @ u.T
            measured = -math.degrees(math.atan2(fitted[1, 0], fitted[0, 0]))
            cases.append({"true_camera": name, "assumed_principal_point_px": [cx, cy],
                          "exact_matches": int(len(old)), "true_increment_degrees": 1.,
                          "estimated_increment_degrees": measured,
                          "linear_bias_for_1080_degrees": (measured - 1.) * 1080})
    return {"method": "Exact noiseless floor matches, weighted rigid fit; no optical flow",
            "caveat": "Repeated-support linear sensitivity example, not a mission or an arbitrary-feature error bound",
            "cases": cases}


def run_audit(destination: Path):
    destination.mkdir(parents=True, exist_ok=True)
    world = generate_world(7)
    world["source"]["spawn"] = [1.2, 1.3, 0.]
    robot = VisionRover(world)
    try:
        packet = robot.rgb.read()
        Image.fromarray(packet.pixels).save(destination / "settled_camera.png")
        model, data = robot.model, robot.data
        root = robot.root
        root_position = data.xpos[root].copy()
        world_base = data.xmat[root].reshape(3, 3) @ CAD_TO_BASE.T
        world_camera = np.eye(4)
        world_camera[:3, :3] = data.cam_xmat[0].reshape(3, 3) @ GL_TO_CV
        world_camera[:3, 3] = data.cam_xpos[0]
        actual_base_camera = np.eye(4)
        actual_base_camera[:3, :3] = world_base.T @ world_camera[:3, :3]
        actual_base_camera[:3, 3] = world_base.T @ (world_camera[:3, 3] - root_position)
        calibration = robot.calibration
        nominal = np.asarray(calibration["T_base_camera"])
        K = np.asarray(calibration["K"])
        height, width = calibration["height"], calibration["width"]

        def frustum():
            renderer = robot.rgb._renderer
            rect, scene = renderer._rect, renderer.scene
            return {"viewport": {name: int(getattr(rect, name)) for name in
                                  ("left", "bottom", "width", "height")},
                    "stereo_mode": int(scene.stereo),
                    "eyes": [_camera_record(camera) for camera in scene.camera]}

        actual_frustum = robot.sim["_camera_runtime"].submit(frustum)
        eye = actual_frustum["eyes"][0]
        fy_frustum = height * eye["frustum_near"] / (eye["frustum_top"] - eye["frustum_bottom"])
        cy_frustum = height * eye["frustum_top"] / (eye["frustum_top"] - eye["frustum_bottom"]) - .5
        # With zero horizontal center and frustum_width=0, MuJoCo derives the
        # horizontal field from viewport aspect. Image-array pixel centers are
        # at integers, while viewport edges lie at -0.5 and dimension-0.5.
        frustum_K = np.array([[fy_frustum, 0., (width - 1) / 2],
                              [0., fy_frustum, cy_frustum], [0., 0., 1.]])

        # Compile a separate black calibration fixture with frontoparallel
        # emissive squares. Their physical centers are known in camera space;
        # they are not navigation landmarks and never reach CameraPerception.
        source = (ROOT / "assets" / "mark_rover.xml").read_text()
        diagnostic_xml, _ = vision_robot_xml(source)
        document = ET.fromstring(diagnostic_xml)
        body = document.find("worldbody")
        for element in list(body):
            if element.tag != "body":
                body.remove(element)
        asset = document.find("asset")
        ET.SubElement(asset, "material", name="diagnostic_white", rgba="1 1 1 1",
                      emission="1", specular="0", shininess="0")
        visual = document.find("visual")
        if visual is None:
            visual = ET.SubElement(document, "visual")
        ET.SubElement(visual, "headlight", active="0")
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, world_camera[:3, :3].ravel())
        targets = []
        for row, v in enumerate((35.125, 90.25, 149.625, 207.875)):
            for column, u in enumerate((42.125, 99.25, 159.5, 220.625, 278.875)):
                depth = .6 + .08 * ((column + 2 * row) % 5)
                point = np.array([(u - K[0, 2]) / K[0, 0] * depth,
                                  (v - K[1, 2]) / K[1, 1] * depth, depth])
                world_point = world_camera[:3, :3] @ point + world_camera[:3, 3]
                half_width = 4.2 * depth / K[0, 0]
                ET.SubElement(body, "geom", name=f"diagnostic_target_{len(targets)}",
                    type="box", size=f"{half_width:.16g} {half_width:.16g} .0000001",
                    pos=" ".join(map(str, world_point)), quat=" ".join(map(str, quaternion)),
                    contype="0", conaffinity="0", material="diagnostic_white")
                targets.append({"camera_point_m": point.tolist(), "expected_pixel": [u, v]})
        diagnostic_model = mujoco.MjModel.from_xml_string(ET.tostring(document, encoding="unicode"))
        diagnostic_model.stat.extent = model.stat.extent
        diagnostic_model.vis.map.znear = model.vis.map.znear
        diagnostic_model.vis.map.zfar = model.vis.map.zfar
        diagnostic_data = mujoco.MjData(diagnostic_model)
        diagnostic_data.qpos[:] = data.qpos
        mujoco.mj_forward(diagnostic_model, diagnostic_data)

        def raster_fixture():
            renderer = mujoco.Renderer(diagnostic_model, height=height, width=width)
            try:
                renderer.update_scene(diagnostic_data, camera="cam_mark_phone_rear")
                return renderer.render().copy()
            finally:
                renderer.close()

        pixels = robot.sim["_camera_runtime"].submit(raster_fixture)
        Image.fromarray(pixels).save(destination / "raster_targets.png")
        gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY).astype(float)
        components, labels = cv2.connectedComponents((gray > 5).astype(np.uint8), connectivity=8)
        centroids = []
        for component in range(1, components):
            ys, xs = np.nonzero(labels == component)
            if len(xs) < 10:
                continue
            weight = gray[ys, xs]
            centroids.append([float(np.average(xs, weights=weight)),
                              float(np.average(ys, weights=weight))])
        centroids = np.asarray(centroids)
        if len(centroids) != len(targets):
            raise AssertionError(f"Expected {len(targets)} raster squares, found {len(centroids)}")
        observations, current_errors, alternative_errors = [], [], []
        alternative = K.copy()
        alternative[0, 2] = width / 2
        alternative[1, 2] = height / 2
        for target in targets:
            point = np.asarray(target["camera_point_m"])
            projected = K @ point
            expected = projected[:2] / projected[2]
            alternate_projection = alternative @ point
            alternate_expected = alternate_projection[:2] / alternate_projection[2]
            observed = centroids[np.argmin(np.linalg.norm(centroids - expected, axis=1))]
            error, alternate_error = observed - expected, observed - alternate_expected
            observations.append({**target, "raster_centroid": observed.tolist(),
                                 "residual_for_159_5_119_5_px": error.tolist(),
                                 "residual_for_160_120_px": alternate_error.tolist()})
            current_errors.append(error)
            alternative_errors.append(alternate_error)
        points = np.asarray([target["camera_point_m"] for target in targets])
        observed = np.asarray([item["raster_centroid"] for item in observations])
        fit_x = np.linalg.lstsq(np.column_stack((points[:, 0] / points[:, 2], np.ones(len(points)))),
                                observed[:, 0], rcond=None)[0]
        fit_y = np.linalg.lstsq(np.column_stack((points[:, 1] / points[:, 2], np.ones(len(points)))),
                                observed[:, 1], rcond=None)[0]
        errors, alternatives = np.asarray(current_errors), np.asarray(alternative_errors)
        yaw = math.atan2(world_base[1, 0], world_base[0, 0])
        unyaw = np.array([[math.cos(yaw), math.sin(yaw), 0.],
                          [-math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        settled_camera_at_local_floor = world_camera.copy()
        settled_camera_at_local_floor[:3, :3] = unyaw @ world_camera[:3, :3]
        settled_camera_at_local_floor[:3, 3] = unyaw @ (
            world_camera[:3, 3] - np.array([root_position[0], root_position[1], 0.]))
        report = {
            "diagnostic_only": True, "online_perception_inputs_unchanged": True,
            "method": "Inspect actual GL frustum and rasterize 20 isolated emissive camera-facing squares",
            "source_rover_sha256": hashlib.sha256(source.encode()).hexdigest(),
            "static_calibration": calibration, "actual_gl": actual_frustum,
            "K_from_actual_frustum_and_integer_pixel_centers": frustum_K.tolist(),
            "K_fitted_from_rendered_centroids": [[fit_x[0], 0., fit_x[1]], [0., fit_y[0], fit_y[1]], [0., 0., 1.]],
            "ideal_correspondence_calibration_sensitivity": ideal_correspondence_sensitivity(
                K, nominal, settled_camera_at_local_floor),
            "raster_validation": {
                "target_count": len(targets),
                "current_principal_point_mean_residual_px": errors.mean(axis=0).tolist(),
                "current_principal_point_vector_rmse_px": float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
                "alternative_160_120_mean_residual_px": alternatives.mean(axis=0).tolist(),
                "alternative_160_120_vector_rmse_px": float(np.sqrt(np.mean(np.sum(alternatives**2, axis=1)))),
                "targets": observations},
            "settled_extrinsic_check": {
                "simulation_time": float(data.time),
                "base_world_position_m": root_position.tolist(),
                "base_world_angles": _axis_angles(world_base),
                "actual_T_base_camera": actual_base_camera.tolist(),
                "max_abs_static_T_error": float(np.max(np.abs(actual_base_camera - nominal))),
                "static_height_m": float(nominal[2, 3]),
                "actual_camera_height_above_world_floor_m": float(world_camera[2, 3]),
                "camera_height_difference_m": float(world_camera[2, 3] - nominal[2, 3]),
                "actual_optical_pitch_degrees": math.degrees(math.atan2(
                    -world_camera[2, 2], np.linalg.norm(world_camera[:2, 2]))),
            },
            "recommendation": "Retain cx=159.5,cy=119.5 and current static camera extrinsics; investigate estimator/mask/planarity drift instead of applying a half-pixel correction.",
        }
        (destination / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        return report
    finally:
        robot.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts" / "camera-calibration-audit")
    args = parser.parse_args()
    result = run_audit(args.output)
    print(json.dumps({"directory": str(args.output), "raster_validation": {
        key: value for key, value in result["raster_validation"].items() if key != "targets"},
        "fitted_K": result["K_fitted_from_rendered_centroids"],
        "settled_extrinsic_check": result["settled_extrinsic_check"]}, indent=2))
