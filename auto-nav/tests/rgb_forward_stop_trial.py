"""Scripted physical forward-stop checks with a faulty historical free map.

This is a safety test, not an autonomous exploration mission. Its only injected
controller information is an arbitrary local straight path and deliberately
false historical FREE occupancy. RGB localization and floor intervals stay
unchanged. World geometry and true pose are read only after control decisions,
for evaluation. Run with the Isaac Python and PYTHONPATH=auto-nav/.deps:auto-nav.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from rover_nav.mission import Mission, ROOT
from rover_nav.navigation import FREE
from rover_nav.world import generate_world
from rgb_fault_trial import write_json, json_safe


def run_case(destination, *, suppress_positive_hits=False):
    destination = Path(destination)
    images = destination / "rgb_inputs"
    images.mkdir(parents=True, exist_ok=True)
    world = generate_world(42)
    world["source"]["spawn"] = [7.3, 4.9, -math.pi / 2]
    mission = Mission(42, world=world, sensing="rgb")
    robot, nav = mission.robot, mission.nav
    diagnostics = []
    first_motion_pose = first_stop_pose = None
    first_stop_distance = None
    stop_timestamp = None
    first_stop_reason = None
    positive_steps = 0
    actual_hit_frames = 0
    supplied_hit_frames = 0
    from PIL import Image

    try:
        # Stationary image-only initialization, with real renderer exposures.
        for _ in range(5):
            robot.observe()
            robot.drive(0., 0.)
            robot.advance(.1)

        # The fault is a wholly historical false-free map. Do not introduce
        # wall coordinates, true pose or true depth into the controller.
        nav._observed.fill(True)
        nav._evidence.fill(-12)
        nav.grid.fill(FREE)
        nav.status, nav.reason = "following", "scripted_forward_safety_goal"
        nav.goal = (2., 0.)
        nav.path = [(float(x), 0.) for x in np.arange(0., 2.0001, .05)]
        start_time = float(robot.data.time)
        initial_true_pose = robot.pose()  # evaluator only
        initial_distance = float(robot.distance)

        for index in range(120):
            original = robot.observe()
            actual_hits = int(np.count_nonzero(original.hits))
            observation = (replace(original, hits=np.zeros_like(original.hits, dtype=bool),
                                   hit_ranges=np.full_like(original.ranges, np.nan))
                           if suppress_positive_hits else original)
            actual_hit_frames += actual_hits > 0
            supplied_hit_frames += bool(np.any(observation.hits))
            # Keep invoking ordinary Navigator only until its first actual
            # moving stop. The subsequent zero-command hold measures braking
            # without allowing this scripted obstacle-facing goal to replan.
            command = nav.update(observation) if stop_timestamp is None else (0., 0.)
            true_before = robot.pose()  # evaluator only, after control decision
            distance_before = float(robot.distance)
            if command[0] > .01:
                positive_steps += 1
                if first_motion_pose is None:
                    first_motion_pose = true_before
            elif positive_steps and stop_timestamp is None:
                stop_timestamp = float(observation.timestamp)
                first_stop_reason = nav.reason
                first_stop_pose = true_before
                first_stop_distance = distance_before
            robot.drive(*command)
            robot.advance(.1)
            true_after = robot.pose()  # evaluator only
            packet = robot.rgb.last_packet
            Image.fromarray(packet.pixels).save(images / f"frame_{index:04d}.png")
            finite = np.isfinite(original.ranges)
            diagnostics.append({
                "timestamp": observation.timestamp,
                "phase": "braking_hold" if stop_timestamp is not None else "forward_control",
                "tracking": observation.tracking,
                "command": list(command), "navigation_state": nav.status,
                "navigation_reason": nav.reason,
                "estimated_pose": vars(observation.pose),
                "visual_reason": robot.perception.last_vo.reason,
                "vo_inliers": robot.perception.last_vo.inliers,
                "floor_valid": robot.perception.last_floor.valid,
                "floor_reason": robot.perception.last_floor.reason,
                "free_ranges": original.ranges,
                "free_min_ranges": original.min_ranges,
                "free_interval_unions": original.free_intervals,
                "finite_free_intervals": int(np.count_nonzero(finite)),
                "actual_positive_hits": actual_hits,
                "supplied_positive_hits": int(np.count_nonzero(observation.hits)),
                "floor_intervals_preserved": bool(np.array_equal(
                    original.ranges, observation.ranges, equal_nan=True)
                    and np.array_equal(original.min_ranges, observation.min_ranges,
                                       equal_nan=True)),
                "free_interval_union_preserved": original.free_intervals == observation.free_intervals,
                "explicit_rgb_interval_union_received": original.free_intervals is not None,
                "rgb_sha256": hashlib.sha256(packet.pixels.tobytes()).hexdigest(),
                "image": f"rgb_inputs/frame_{index:04d}.png",
                "evaluator_only_true_pose": vars(true_after),
                "evaluator_only_distance_m": float(robot.distance),
                "collision_count": robot.collision_count,
            })
            if robot.collision_count:
                break
            if stop_timestamp is not None and float(robot.data.time) >= stop_timestamp + 1.:
                break

        final_pose = robot.pose()
        stopping_path = (float(robot.distance) - first_stop_distance
                         if first_stop_distance is not None else None)
        forward_displacement = initial_true_pose.y - final_pose.y
        # Supplied test scene has the y=4 partition with a 0.06 m half-depth.
        # This measurement never influences a command or termination decision.
        minimum_clearance = min(frame["evaluator_only_true_pose"]["y"] - 4.06 - .25
                                for frame in diagnostics)
        stopped = stop_timestamp is not None
        criteria = {
            "actual_forward_wheel_motion": positive_steps >= 3 and forward_displacement > .05,
            "navigator_commands_a_stop": stopped,
            "tracking_remains_image_based": all(frame["tracking"] for frame in diagnostics),
            "actual_rgb_floor_intervals_preserved": all(
                frame["floor_intervals_preserved"] for frame in diagnostics),
            "actual_rgb_disjoint_interval_unions_preserved": all(
                frame["free_interval_union_preserved"] for frame in diagnostics),
            "explicit_rgb_interval_unions_received": all(
                frame["explicit_rgb_interval_union_received"] for frame in diagnostics),
            "no_physical_contact": robot.collision_count == 0,
            "conservative_disk_stays_5cm_from_wall": minimum_clearance >= .05,
            "braking_path_below_7cm": stopping_path is not None and stopping_path < .07,
            "test_does_not_claim_exploration_completion": nav.status != "complete",
        }
        if suppress_positive_hits:
            criteria["positive_hits_suppressed"] = supplied_hit_frames == 0
            criteria["fresh_floor_witness_stops_false_free_motion"] = (
                first_stop_reason == "insufficient_current_forward_floor")
        else:
            criteria["real_positive_hit_evidence_observed"] = actual_hit_frames > 0
        report = {
            "trial_passed": all(criteria.values()), "criteria": criteria,
            "test_kind": "scripted forward safety goal; not autonomous exploration",
            "fault": {"historical_map": "every allocated cell falsely FREE before control",
                      "positive_hits_suppressed": suppress_positive_hits,
                      "unchanged_inputs": "actual RGB pixels, timestamps, image pose and floor intervals"},
            "goal_local_m": [2., 0.], "waypoint_spacing_m": .05,
            "sensor_boundary": "RGB pixels plus exposure clock and fixed camera calibration",
            "source_sha256_at_start": mission.source_hashes,
            "evaluation_only": {"scene_seed": 42, "spawn": world["source"]["spawn"],
                "initial_true_pose": vars(initial_true_pose),
                "first_motion_true_pose": None if first_motion_pose is None else vars(first_motion_pose),
                "first_stop_true_pose": None if first_stop_pose is None else vars(first_stop_pose),
                "final_true_pose": vars(final_pose),
                "forward_displacement_m": forward_displacement,
                "total_path_length_m": float(robot.distance) - initial_distance,
                "braking_path_length_m": stopping_path,
                "minimum_conservative_disk_wall_clearance_m": minimum_clearance,
                "collision_count": robot.collision_count,
                "max_collision_force_n": robot.max_collision_force},
            "simulation_duration_seconds": float(robot.data.time) - start_time,
            "frames": len(diagnostics), "positive_command_frames": positive_steps,
            "first_stop_reason": first_stop_reason,
            "first_stop_timestamp": stop_timestamp,
        }
        write_json(destination / "report.json", report)
        write_json(destination / "diagnostics.json", diagnostics)
        write_json(destination / "calibration.json", robot.calibration)
        write_json(destination / "world-evaluator-only.json", world)
        np.save(destination / "final_occupancy.npy", nav.grid)
        print(json.dumps(json_safe(report), indent=2), flush=True)
        assert report["trial_passed"], {name: value for name, value in criteria.items() if not value}
        return report
    finally:
        robot.drive(0., 0.)
        mission.close()


def run(output):
    output = Path(output)
    normal = run_case(output / "normal-positive-hits")
    suppressed = run_case(output / "suppressed-positive-hits", suppress_positive_hits=True)
    write_json(output / "acceptance-summary.json", {
        "passed": normal["trial_passed"] and suppressed["trial_passed"],
        "normal_positive_hits": "normal-positive-hits/report.json",
        "suppressed_positive_hits": "suppressed-positive-hits/report.json"})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "artifacts" / "rgb-forward-stop-final-v4")
    run(parser.parse_args().output)
