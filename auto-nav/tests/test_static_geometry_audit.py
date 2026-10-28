"""Reproducible offline footprint/door audit, with no rendering or motion.

Run as a script to persist its independent evidence::

    PYTHONPATH=auto-nav/.deps:auto-nav ../isaacprotocol/simulation/venv/bin/python \
        auto-nav/tests/test_static_geometry_audit.py \
        --output auto-nav/artifacts/static-geometry-audit.json

Truth geometry is used only by this evaluator. Nothing here is imported by
RGB perception or the deployed controller. Static ``mj_forward`` computes
model transforms at qpos0; there are no physics steps or camera/range reads.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch

import mujoco
import numpy as np
import pytest
from scipy import ndimage

from rover_nav.navigation import FREE, OCCUPIED, UNKNOWN, Navigator
from rover_nav.vision_scene import vision_robot_xml
from rover_nav.world import generate_world


ROOT = Path(__file__).resolve().parents[1]
_CAD_TO_BASE = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def footprint_audit(model: mujoco.MjModel) -> dict:
    """Bound collision meshes for every planar yaw and every wheel angle.

    The norm of any convex combination cannot exceed the greatest vertex
    norm, so bounding every mesh vertex also bounds MuJoCo's convex collision
    hull. For wheel hinges parallel to base Y, each vertex keeps its lateral
    coordinate and rotates in the forward/up plane. Its greatest forward
    excursion is |hinge_forward| + radial_distance_to_the_hinge_axis.
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    free = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
    if len(free) != 1:
        raise ValueError("The audit expects one floating rover root")
    root = int(model.jnt_bodyid[free[0]])
    world_from_base = data.xmat[root].reshape(3, 3) @ _CAD_TO_BASE.T
    results, points, envelope_min, envelope_max = [], [], [], []
    wheel_joints = set()
    for geom in range(model.ngeom):
        body = int(model.geom_bodyid[geom])
        if body == 0 or not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError("Add an analytic shape bound before auditing a non-mesh collider")
        mesh = int(model.geom_dataid[geom])
        start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        local = model.mesh_vert[start:start + count].astype(float)
        world = local @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        base = (world - data.xpos[root]) @ world_from_base
        nominal_radius = float(np.linalg.norm(base[:, :2], axis=1).max())
        hinges = []
        ancestor = body
        while ancestor and ancestor != root:
            first = int(model.body_jntadr[ancestor])
            for joint in range(first, first + int(model.body_jntnum[ancestor])):
                if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE:
                    raise ValueError("Unsupported articulated collision part")
                hinges.append(joint)
            ancestor = int(model.body_parentid[ancestor])
        lower, upper = base.min(axis=0), base.max(axis=0)
        radius = nominal_radius
        if hinges:
            if len(hinges) != 1:
                raise ValueError("Nested articulated joints need a separate bound")
            hinge = hinges[0]
            axis = data.xaxis[hinge] @ world_from_base
            if not np.allclose(np.abs(axis), [0., 1., 0.], atol=1e-8):
                raise ValueError("This exact wheel-spin proof requires a lateral hinge axis")
            anchor = (data.xanchor[hinge] - data.xpos[root]) @ world_from_base
            radial = np.linalg.norm((base - anchor)[:, [0, 2]], axis=1)
            all_spin_radius = np.hypot(abs(anchor[0]) + radial, base[:, 1])
            radius = float(all_spin_radius.max())
            lower[[0, 2]] = anchor[[0, 2]] - radial.max()
            upper[[0, 2]] = anchor[[0, 2]] + radial.max()
            wheel_joints.add(hinge)
        margin = max(0., float(model.geom_margin[geom]))
        radius += margin
        envelope_min.append(lower - margin)
        envelope_max.append(upper + margin)
        points.append(base)
        results.append({
            "geom": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom),
            "vertices": count, "wheel_hinges": len(hinges),
            "nominal_radius_m": nominal_radius,
            "all_wheel_angles_radius_including_contact_margin_m": radius,
            "contact_margin_m": margin,
        })
    all_points = np.concatenate(points)
    lower, upper = np.min(envelope_min, axis=0), np.max(envelope_max, axis=0)
    box_radius = float(np.linalg.norm(np.maximum(abs(lower[:2]), abs(upper[:2]))))
    radius = max(item["all_wheel_angles_radius_including_contact_margin_m"] for item in results)
    return {
        "model_bodies": int(model.nbody), "collision_geoms": len(results),
        "mesh_vertices_checked": int(len(all_points)), "wheel_hinges": len(wheel_joints),
        "nominal_xy_radius_m": float(np.linalg.norm(all_points[:, :2], axis=1).max()),
        "all_wheel_angles_xy_radius_m": radius,
        "all_wheel_angles_aabb_min_m": lower.tolist(),
        "all_wheel_angles_aabb_max_m": upper.tolist(),
        "conservative_aabb_disc_radius_m": box_radius,
        "configured_robot_radius_m": .25,
        "remaining_radial_allowance_m": .25 - box_radius,
        "worst_geoms": sorted(results, key=lambda item:
            item["all_wheel_angles_radius_including_contact_margin_m"], reverse=True)[:4],
        "scope": "All planar yaw angles and every wheel spin; level rigid rover, not tipping or deformation",
    }


def door_grid_audit(world: dict) -> dict:
    objects = {item["name"]: item for item in world["source"]["objects"]}
    def box(name):
        return world["boxes"][objects[name]["first_box"]]
    south, north = box("living_study_partition_south"), box("living_study_partition_north")
    west, east = box("study_bedroom_partition_west"), box("study_bedroom_partition_east")
    widths = [north["center"][1] - north["size"][1] / 2
              - south["center"][1] - south["size"][1] / 2,
              east["center"][0] - east["size"][0] / 2
              - west["center"][0] - west["size"][0] / 2]
    width = min(widths)
    trials = []
    for phase in (0., .01, .02, .03, .04):
        nav = Navigator(bounds=(-2 + phase, -2 + phase, 2 + phase, 2 + phase),
                        resolution=.05, robot_radius=.25, safety_margin=.08,
                        account_cell_area=True)
        yy, xx = np.indices(nav.shape)
        x = nav.origin[0] + (xx + .5) * nav.resolution
        y = nav.origin[1] + (yy + .5) * nav.resolution
        occupied = (abs(x) <= .06) & (abs(y) >= width / 2)
        unknown = (abs(x) <= .16) & (abs(y) >= width / 2 - .10) & ~occupied
        nav.grid[:] = FREE
        nav.grid[occupied], nav.grid[unknown] = OCCUPIED, UNKNOWN
        safe = nav.traversable()
        path = nav.astar(nav.world_to_cell(-1., 0.), nav.world_to_cell(1., 0.), safe)
        column = nav.world_to_cell(0., 0.)[1]
        clearance = ndimage.distance_transform_edt(np.pad(nav.grid == FREE, 1))[1:-1, 1:-1]
        minimum = (float(clearance[tuple(np.asarray(path).T)].min() * nav.resolution)
                   if path else None)
        trials.append({"grid_phase_m": phase, "path_exists": bool(path),
                       "path_points": len(path),
                       "safe_crossing_rows": int(safe[:, column].sum()),
                       "minimum_path_edt_clearance_m": minimum})
    threshold = nav.inflation_radius + .02 + nav.cell_area_margin
    return {"actual_door_widths_m": widths,
            "descriptor_door_widths_m": [item["width"] for item in world["source"]["doors"]],
            "resolution_m": nav.resolution, "robot_radius_m": nav.robot_radius,
            "safety_margin_m": nav.safety_margin, "tracking_extra_margin_m": .02,
            "cell_half_diagonal_m": nav.cell_area_margin,
            "effective_edt_threshold_m": threshold,
            "unknown_strip_each_side_m": .10,
            "remaining_known_opening_m": width - .20,
            "continuous_center_band_m": width - .20 - 2 * threshold,
            "phase_trials": trials,
            "scope": "Offline geometry/grid feasibility only, not perception or mission success"}


def placement_audit(world: dict) -> dict:
    def clearance(spawn):
        distances = []
        for box in world["boxes"]:
            cx, cy, cz = box["center"]
            sx, sy, sz = box["size"]
            if cz - sz / 2 >= .323 or cz + sz / 2 <= .035:
                continue
            distances.append(math.hypot(max(0., abs(spawn[0] - cx) - sx / 2),
                                        max(0., abs(spawn[1] - cy) - sy / 2)))
        return min(distances)
    return {"rgb_world_spawn": [1.2, 1.3, 0.], "visual_estimator_initial_pose": [0., 0., 0.],
            "rgb_spawn_collision_clearance_m": clearance((1.2, 1.3)),
            "historical_spawn_collision_clearance_m": clearance((1., 1.)),
            "scope": "Fixed scene placement/evaluator coordinates; never visual-estimator input"}


def build_audit() -> dict:
    source = (ROOT / "assets" / "mark_rover.xml").read_bytes()
    xml, calibration = vision_robot_xml(source.decode())
    model = mujoco.MjModel.from_xml_string(xml)
    world = generate_world(7)
    return {"version": 1, "method": "Static qpos0 forward kinematics; zero physics steps, no GL or camera/range reads",
            "original_rover_sha256": hashlib.sha256(source).hexdigest(),
            "pitched_rover_sha256": hashlib.sha256(xml.encode()).hexdigest(),
            "phone_pitch_degrees": calibration["mount_pitch_degrees"],
            "footprint": footprint_audit(model), "door_grid": door_grid_audit(world),
            "placement": placement_audit(world),
            "not_mission_acceptance": True}


@pytest.fixture(scope="module")
def static_audit():
    # The safety proof must remain runnable without either moving or rendering.
    fail = AssertionError("The static geometry audit must not step or render")
    with patch.object(mujoco, "mj_step", side_effect=fail), \
            patch.object(mujoco, "Renderer", side_effect=fail):
        return build_audit()


def test_configured_circle_encloses_all_collision_parts_and_wheel_angles(static_audit):
    result = static_audit["footprint"]
    assert result["model_bodies"] == 55
    assert result["collision_geoms"] == 53
    assert result["wheel_hinges"] == 4
    assert result["mesh_vertices_checked"] > 200_000
    assert result["nominal_xy_radius_m"] <= result["all_wheel_angles_xy_radius_m"]
    assert result["all_wheel_angles_xy_radius_m"] <= result["conservative_aabb_disc_radius_m"] < .25
    assert result["remaining_radial_allowance_m"] > .035
    assert result["all_wheel_angles_aabb_max_m"][2] < .323


def test_real_openings_remain_plannable_with_cell_area_and_unknown_strips(static_audit):
    result = static_audit["door_grid"]
    assert result["actual_door_widths_m"] == pytest.approx([1.08, 1.08])
    assert result["cell_half_diagonal_m"] == pytest.approx(math.sqrt(.5) * .05)
    assert result["effective_edt_threshold_m"] == pytest.approx(.3853553390593274)
    assert result["continuous_center_band_m"] > .10
    assert len(result["phase_trials"]) == 5
    for trial in result["phase_trials"]:
        assert trial["path_exists"]
        assert trial["safe_crossing_rows"] >= 3
        assert trial["minimum_path_edt_clearance_m"] >= result["effective_edt_threshold_m"]


def test_rgb_scene_placement_has_clearance_without_providing_absolute_pose(static_audit):
    result = static_audit["placement"]
    threshold = static_audit["door_grid"]["effective_edt_threshold_m"]
    assert result["rgb_spawn_collision_clearance_m"] > threshold
    assert result["historical_spawn_collision_clearance_m"] < threshold
    assert result["visual_estimator_initial_pose"] == [0., 0., 0.]
    assert result["rgb_world_spawn"] != result["visual_estimator_initial_pose"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts" / "static-geometry-audit.json")
    arguments = parser.parse_args()
    report = build_audit()
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"artifact": str(arguments.output),
                      "footprint_radius_m": report["footprint"]["all_wheel_angles_xy_radius_m"],
                      "conservative_disc_radius_m": report["footprint"]["conservative_aabb_disc_radius_m"],
                      "door_phase_trials_passed": all(item["path_exists"] for item in report["door_grid"]["phase_trials"])}))
