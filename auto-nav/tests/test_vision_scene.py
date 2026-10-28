"""Static camera calibration and physical preservation for the RGB scene."""
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from rover_nav.vision_scene import (
    FLOOR_MATERIAL_NAME, FLOOR_TEXTURE_NAME, add_floor_texture,
    prepare_vision_scene, vision_robot_xml,
)
from rover_nav.world import generate_world


ROBOT_XML = (Path(__file__).resolve().parents[1] / "assets" / "mark_rover.xml").read_text()


def test_pitch_rotates_whole_cradle_and_preserves_original_robot_parts():
    modified, calibration = vision_robot_xml(ROBOT_XML)
    before, after = ET.fromstring(ROBOT_XML), ET.fromstring(modified)
    old_bodies = {body.get("name"): body for body in before.iter("body")}
    new_bodies = {body.get("name"): body for body in after.iter("body")}
    assert old_bodies.keys() == new_bodies.keys()
    assert len(old_bodies) == 54  # Free root plus all 53 authored parts.
    changed = []
    for name, old in old_bodies.items():
        new = new_bodies[name]
        if old.attrib != new.attrib:
            changed.append(name)
            assert name.startswith("Phone_cradle___backplate__ledge__")
            assert {k: v for k, v in old.attrib.items() if k != "quat"} == {
                k: v for k, v in new.attrib.items() if k != "quat"}
        assert [child.attrib for child in old if child.tag != "body"] == [
            child.attrib for child in new if child.tag != "body"]
    assert len(changed) == 1
    expected = [math.cos(math.radians(12.5)), -math.sin(math.radians(12.5)), 0., 0.]
    assert np.fromstring(new_bodies[changed[0]].get("quat"), sep=" ") == pytest.approx(expected)
    for tag in ("geom", "inertial", "joint", "freejoint", "camera", "actuator"):
        assert [element.attrib for element in before.iter(tag)] == [
            element.attrib for element in after.iter(tag)]
    assert calibration["mount_pitch_degrees"] == pytest.approx(25)


def test_calibration_is_metric_static_and_has_near_floor_visibility():
    _, calibration = vision_robot_xml(ROBOT_XML)
    transform = np.asarray(calibration["T_base_camera"])
    rotation = transform[:3, :3]
    assert rotation.T @ rotation == pytest.approx(np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1)
    assert rotation[:, 2] == pytest.approx([math.cos(math.radians(25)), 0., -math.sin(math.radians(25))])
    assert transform[:3, 3] == pytest.approx([.0077107296, .02, .3001021138])
    assert 0 < calibration["min_visible_floor_m"] <= .25
    focal = 240 / (2 * math.tan(math.radians(30)))
    assert np.asarray(calibration["K"]) == pytest.approx(np.array([
        [focal, 0, 159.5], [0, focal, 119.5], [0, 0, 1]]))
    assert "world" not in calibration["source"]


def test_calibration_excludes_free_root_world_pose_and_world_metadata():
    root = ET.fromstring(ROBOT_XML)
    base = next(body for body in root.find("worldbody").findall("body")
                if body.find("freejoint") is not None)
    base.set("pos", "100 -87 4")
    base.set("quat", ".5 .5 .5 .5")
    _, original = vision_robot_xml(ROBOT_XML)
    _, moved = vision_robot_xml(ET.tostring(root, encoding="unicode"))
    assert original == moved
    world = generate_world(7)
    alternate = deepcopy(world)
    alternate["source"]["spawn"] = [7., 6., 2.]
    alternate["source"]["nav_bounds"] = [-100., -200., 1000., 2000.]
    _, first = prepare_vision_scene(ROBOT_XML, world)
    _, second = prepare_vision_scene(ROBOT_XML, alternate)
    assert first == second


def test_vision_scene_compiles_and_keeps_original_dynamics_and_all_four_drives():
    modified, calibration = prepare_vision_scene(ROBOT_XML, generate_world(7))
    original = mujoco.MjModel.from_xml_string(ROBOT_XML)
    model = mujoco.MjModel.from_xml_string(modified)
    assert model.nbody == original.nbody == 55  # Includes the MuJoCo world body.
    assert model.nu == original.nu == 4
    assert model.ncam == original.ncam == 1
    assert model.njnt == original.njnt
    assert model.body_mass == pytest.approx(original.body_mass)
    assert model.body_inertia == pytest.approx(original.body_inertia)
    assert model.actuator_ctrlrange == pytest.approx(original.actuator_ctrlrange)
    assert model.actuator_forcerange == pytest.approx(original.actuator_forcerange)
    robot_geoms = original.ngeom - 1  # Original export has one world floor.
    assert sum(model.geom_bodyid != 0) == robot_geoms
    assert np.isfinite(model.geom_friction).all()
    assert model.ntex == original.ntex + 1
    assert np.std(model.tex_data) > 0
    # A physics state is used only here as an independent test oracle, never by
    # the calibration function. The scene's known spawn is not part of K/T.
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    direction = data.cam_xmat[0].reshape(3, 3) @ np.array([0., 0., -1.])
    assert direction == pytest.approx([math.cos(math.radians(25)), 0., -math.sin(math.radians(25))], abs=1e-10)
    assert data.cam_xpos[0, 2] == pytest.approx(calibration["height_m"])


def test_bottom_rgb_rays_really_see_near_floor_without_hiding_any_robot_geom():
    # This uses simulator ray intersections exclusively as an offline scene
    # design oracle. Neither ray results nor geom IDs enter RGB perception.
    xml, calibration = prepare_vision_scene(ROBOT_XML, generate_world(7))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    intrinsics = np.asarray(calibration["K"])
    camera_rotation = data.cam_xmat[0].reshape(3, 3)
    origin = data.cam_xpos[0].copy()
    all_groups_visible = np.ones(6, dtype=np.uint8)
    for column in (80., 120., 159.5, 200., 240.):
        ray = camera_rotation @ np.array([
            (column - intrinsics[0, 2]) / intrinsics[0, 0],
            -(239. - intrinsics[1, 2]) / intrinsics[1, 1], -1.])
        ray /= np.linalg.norm(ray)
        geom_id = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(model, data, origin, ray, all_groups_visible, 1, -1, geom_id)
        assert distance > 0
        assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id[0])) == "nav_floor"
        if column == 159.5:
            point = origin + distance * ray
            assert np.linalg.norm(point[:2] - [1., 1.]) <= .25


def test_floor_texture_is_embedded_unencoded_and_idempotent():
    xml, _ = prepare_vision_scene(ROBOT_XML, generate_world(7))
    repeated = add_floor_texture(xml)
    root = ET.fromstring(repeated)
    textures = root.findall(f"./asset/texture[@name='{FLOOR_TEXTURE_NAME}']")
    materials = root.findall(f"./asset/material[@name='{FLOOR_MATERIAL_NAME}']")
    assert len(textures) == len(materials) == 1
    assert textures[0].get("builtin") == "flat"
    assert textures[0].get("mark") == "random"
    assert not any("file" in key for key in textures[0].attrib)
    assert materials[0].get("emission") == "0"
    lights = root.findall("./worldbody/light")
    assert len(lights) >= 2
    assert all(np.min(np.fromstring(light.get("ambient", "0 0 0"), sep=" ")) >= .15
               for light in lights if light.get("name", "").startswith("nav_"))
    assert root.find("./worldbody/geom[@name='nav_floor']").get("material") == FLOOR_MATERIAL_NAME
    mujoco.MjModel.from_xml_string(repeated)
