"""A physically pitched phone and an ordinary textured floor for RGB navigation.

Calibration is derived once from the saved MJCF, in a rover-local frame. This
module never reads a simulation state or the world spawn when calibrating the
camera. The original CAD/MJCF remains untouched on disk.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from .world import combine_rover


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
PHONE_PITCH_DEGREES = 25.0
FLOOR_TEXTURE_NAME = "nav_vision_floor_texture_v2"
FLOOR_MATERIAL_NAME = "nav_vision_floor_material_v2"

# Authored CAD axes are +X right, +Y up, -Z forward. Navigation uses +X
# forward, +Y left, +Z up. This fixed change of basis excludes the free root's
# world transform, which is deliberately unavailable to perception.
_CAD_TO_BASE = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
_OPENGL_TO_OPENCV = np.diag([1., -1., -1.])


def _quaternion_matrix(values: str) -> np.ndarray:
    quaternion = np.asarray([float(value) for value in values.split()])
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("MJCF body orientation must be a finite quaternion")
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("MJCF quaternion must have nonzero norm")
    w, x, y, z = quaternion / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def _element_transform(element: ET.Element) -> np.ndarray:
    # The checked-in exporter uses position/quaternion exclusively. Reject
    # another convention instead of silently returning wrong calibration.
    if any(name in element.attrib for name in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Static calibration expects exported position/quaternion transforms")
    transform = np.eye(4)
    transform[:3, :3] = _quaternion_matrix(element.get("quat", "1 0 0 0"))
    position = np.asarray([float(value) for value in element.get("pos", "0 0 0").split()])
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("MJCF position must have three finite coordinates")
    transform[:3, 3] = position
    return transform


def _calibration(root: ET.Element) -> dict:
    body = root.find("worldbody")
    if body is None:
        raise ValueError("Expected an MJCF worldbody")
    free_roots = [element for element in body.findall("body")
                  if element.find("freejoint") is not None
                  or any(joint.get("type") == "free" for joint in element.findall("joint"))]
    if len(free_roots) != 1:
        raise ValueError("Expected exactly one floating rover root")
    cameras: list[tuple[ET.Element, np.ndarray]] = []

    def visit(element: ET.Element, transform: np.ndarray) -> None:
        for child in element:
            if child.tag == "body":
                visit(child, transform @ _element_transform(child))
            elif child.tag == "camera":
                cameras.append((child, transform @ _element_transform(child)))

    # Start at identity: the free root's world position and quaternion are not
    # fixed camera extrinsics, and must not enter the calibration.
    visit(free_roots[0], np.eye(4))
    if len(cameras) != 1:
        raise ValueError("Expected exactly one mounted RGB camera")
    camera, cad_transform = cameras[0]
    transform = np.eye(4)
    transform[:3, :3] = _CAD_TO_BASE @ cad_transform[:3, :3] @ _OPENGL_TO_OPENCV
    transform[:3, 3] = _CAD_TO_BASE @ cad_transform[:3, 3]
    fov_y = float(camera.get("fovy", "45"))
    if not 0 < fov_y < 180:
        raise ValueError("Camera vertical field of view must lie in (0, 180)")
    focal = CAMERA_HEIGHT / (2 * math.tan(math.radians(fov_y) / 2))
    principal = ((CAMERA_WIDTH - 1) / 2, (CAMERA_HEIGHT - 1) / 2)
    intrinsics = np.array([[focal, 0., principal[0]],
                           [0., focal, principal[1]], [0., 0., 1.]])
    height = float(transform[2, 3])
    forward = transform[:3, 2]
    pitch = math.degrees(math.atan2(-forward[2], math.hypot(forward[0], forward[1])))
    # Project the bottom central pixel through only the known mount geometry.
    lowest_ray = transform[:3, :3] @ np.array([0.,
        (CAMERA_HEIGHT - 1 - principal[1]) / focal, 1.])
    minimum_range = math.inf
    if lowest_ray[2] < -1e-12:
        floor_relative_camera = lowest_ray * (-height / lowest_ray[2])
        minimum_range = float(np.linalg.norm(floor_relative_camera[:2]))
    return {
        "version": 2,
        "source": "saved_mjcf_static_mount_and_intrinsics_only",
        "camera_name": camera.get("name"),
        "width": CAMERA_WIDTH, "height": CAMERA_HEIGHT,
        "fov_y_degrees": fov_y,
        "K": intrinsics.tolist(),
        "T_base_camera": transform.tolist(),
        "height_m": height,
        "mount_pitch_degrees": pitch,
        "min_visible_floor_m": minimum_range,
        "mount_design_note": "25 degree physical cradle pitch keeps near-floor pixels clear of the chassis",
        "base_convention": "+X forward, +Y left, +Z up; initial pose is local origin",
        "optical_convention": "OpenCV: +X right, +Y down, +Z forward",
        "assumptions": ["level planar floor", "fixed rigid camera mount",
                        "authored rover root is at the nominal floor support height"],
    }


def vision_robot_xml(robot_xml: str) -> tuple[str, dict]:
    """Pitch the entire authored phone cradle down, retaining every part.

    The only robot mutation is the cradle body's fixed quaternion. Its phone,
    pads, lenses, collision shapes and mass distribution all rotate together.
    """
    root = ET.fromstring(robot_xml)
    cradles = [body for body in root.iter("body")
               if body.get("name", "").startswith("Phone_cradle___backplate__ledge__")]
    if len(cradles) != 1:
        raise ValueError("Expected exactly one authored phone cradle body")
    cradle = cradles[0]
    if any(name in cradle.attrib for name in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Phone cradle must use the original quaternion convention")
    angle = math.radians(PHONE_PITCH_DEGREES) / 2
    cradle.set("quat", f"{math.cos(angle):.16g} {-math.sin(angle):.16g} 0 0")
    return ET.tostring(root, encoding="unicode"), _calibration(root)


def add_floor_texture(xml: str) -> str:
    """Add unencoded random carpet speckles using built-in MuJoCo assets.

    A 16 m texture period exceeds this apartment's dimensions. Random marks
    have no labels, coordinate encoding or fiducial layout. Perception receives
    pixels and learns floor appearance from its initial view, not these colors.
    """
    root = ET.fromstring(xml)
    floor = root.find("./worldbody/geom[@name='nav_floor']")
    if floor is None:
        raise ValueError("Expected the generated static nav_floor plane")
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    for child in list(asset):
        if child.get("name") in {FLOOR_TEXTURE_NAME, FLOOR_MATERIAL_NAME}:
            asset.remove(child)
    ET.SubElement(asset, "texture", name=FLOOR_TEXTURE_NAME, type="2d",
                  builtin="flat", width="1024", height="1024",
                  rgb1=".54 .56 .59", rgb2=".54 .56 .59",
                  mark="random", markrgb=".25 .27 .30", random=".12")
    ET.SubElement(asset, "material", name=FLOOR_MATERIAL_NAME,
                  texture=FLOOR_TEXTURE_NAME, texrepeat=".0625 .0625",
                  texuniform="true", reflectance="0", specular="0",
                  shininess="0", emission="0", roughness="1", rgba="1 1 1 1")
    floor.set("material", FLOOR_MATERIAL_NAME)
    floor.set("rgba", "1 1 1 1")
    # Approximate diffuse room bounce in the actual renderer. The former
    # point-only lights left large floor regions almost black. Ambient light
    # is ordinary scene illumination; textures remain unencoded and shaded,
    # and there is no image postprocessing or emissive floor material.
    key = root.find("./worldbody/light[@name='nav_key_light']")
    if key is not None:
        key.set("ambient", ".22 .22 .22")
        key.set("diffuse", ".65 .65 .65")
        key.set("specular", ".05 .05 .05")
    fill = root.find("./worldbody/light[@name='nav_fill_light']")
    if fill is not None:
        fill.set("ambient", ".18 .18 .18")
        fill.set("diffuse", ".35 .35 .35")
        fill.set("specular", "0 0 0")
    return ET.tostring(root, encoding="unicode")


def prepare_vision_scene(robot_xml: str, world: dict) -> tuple[str, dict]:
    """Return a full physics scene plus independent, rover-local calibration."""
    modified, calibration = vision_robot_xml(robot_xml)
    return add_floor_texture(combine_rover(modified, world)), calibration
