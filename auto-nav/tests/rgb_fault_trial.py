"""Actual rendered-RGB occlusion trials; truth is used only for evaluation.

The normal Mission/CameraPerception/Navigator/motor chain stays intact. The
fault replaces RGB exposures with black pixels and preserves exposure time.
No tracking flag, pose, depth or motion command is injected into perception.
Run with the Isaac Python and ``PYTHONPATH=auto-nav/.deps:auto-nav``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from rover_nav.mission import Mission, ROOT
from rover_nav.vision import RGBPacket


def json_safe(value):
    """Write strict JSON even when a failed sensor reports infinite error."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    return value


def write_json(path, data):
    Path(path).write_text(json.dumps(json_safe(data), indent=2, allow_nan=False))


def run_case(destination, *, duration=1.5, warmup=5., recovered_seconds=2.,
             expect_recovery=False):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    mission = Mission(7, sensing="rgb")
    original_read = mission.robot.rgb.read
    start = float(mission.robot.data.time)
    fault_start, fault_end = start + warmup, start + warmup + duration
    end = fault_end + recovered_seconds
    exposures, diagnostics = [], []
    sample_images = {}
    first_blank_pose = None
    first_blank_distance = None
    last_pre_fault_command = (0., 0.)
    last_accepted_pose = None
    try:
        def read_with_occlusion():
            packet = original_read()
            blank = fault_start - 1e-7 <= packet.timestamp < fault_end - 1e-7
            phase = "occluded" if blank else ("warmup" if packet.timestamp < fault_start else "restored")
            pixels = np.zeros_like(packet.pixels) if blank else packet.pixels
            exposures.append({"timestamp": packet.timestamp, "phase": phase,
                "all_pixels_zero": bool(np.all(pixels == 0)),
                "rgb_sha256": hashlib.sha256(pixels.tobytes()).hexdigest()})
            if phase not in sample_images:
                sample_images[phase] = pixels.copy()
            return RGBPacket(pixels, packet.timestamp)

        mission.robot.rgb.read = read_with_occlusion
        while not mission.finished and float(mission.robot.data.time) < end - 1e-7:
            # These measurements are evaluator-only and are never returned by
            # the image wrapper or supplied to the navigation controller.
            before_pose = mission.robot.pose()
            before_distance = float(mission.robot.distance)
            mission.tick()
            phase = exposures[-1]["phase"]
            current_pose = mission.robot.pose()
            visual = mission.robot.perception.last_vo
            floor = mission.robot.perception.last_floor
            if phase == "warmup":
                last_pre_fault_command = tuple(mission.last_command)
                if visual.tracking:
                    last_accepted_pose = visual.pose
            if phase == "occluded" and first_blank_pose is None:
                first_blank_pose = before_pose
                first_blank_distance = before_distance
            diagnostics.append({"timestamp": mission.latest_observation.timestamp,
                "phase": phase, "tracking": mission.latest_observation.tracking,
                "command": list(mission.last_command), "navigation_state": mission.nav.status,
                "navigation_reason": mission.nav.reason, "visual_reason": visual.reason,
                "inliers": visual.inliers, "features": visual.features,
                "reprojection_error_px": visual.reprojection_error,
                "floor_valid": floor.valid, "floor_reason": floor.reason,
                "estimated_pose": vars(visual.pose),
                "evaluator_only_true_pose": vars(current_pose),
                "evaluator_only_distance_m": float(mission.robot.distance),
                "collision_count": mission.robot.collision_count})

        blanks = [frame for frame in diagnostics if frame["phase"] == "occluded"]
        warmups = [frame for frame in diagnostics if frame["phase"] == "warmup"]
        restored = [frame for frame in diagnostics if frame["phase"] == "restored"]
        final_pose = mission.robot.pose()
        braking_distance = (float(mission.robot.distance) - first_blank_distance
                            if first_blank_distance is not None else None)
        blank_and_paused = [frame for frame in diagnostics if frame["phase"] != "warmup"
                            and (not expect_recovery or not frame["tracking"])]
        # Path length is conservative for the stopping translation; the net
        # displacement is separately reported to expose any jitter/spin effect.
        blank_stop_positions = np.array([[frame["evaluator_only_true_pose"]["x"],
                                          frame["evaluator_only_true_pose"]["y"]]
                                         for frame in blank_and_paused])
        max_displacement = (float(np.max(np.linalg.norm(blank_stop_positions -
            [first_blank_pose.x, first_blank_pose.y], axis=1))) if len(blank_stop_positions)
                            and first_blank_pose is not None else None)
        if expect_recovery and blanks and first_blank_distance is not None:
            braking_distance = blanks[-1]["evaluator_only_distance_m"] - first_blank_distance
        mission_report = mission.report()
        criteria = {
            "good_tracking_before_fault": len(warmups) >= 40 and all(frame["tracking"] for frame in warmups),
            "actual_black_rgb_injected": bool(blanks) and all(frame["all_pixels_zero"]
                for frame in exposures if frame["phase"] == "occluded"),
            "every_blank_frame_stops": bool(blanks) and all(frame["command"] == [0., 0.] for frame in blanks),
            "every_blank_frame_loses_tracking": bool(blanks) and all(not frame["tracking"] for frame in blanks),
            "no_pose_fallback_during_occlusion": last_accepted_pose is not None and all(
                frame["estimated_pose"] == vars(last_accepted_pose) for frame in blanks),
            "braking_translation_below_3cm": braking_distance is not None and braking_distance < .03,
            "no_physical_obstacle_contact": mission.robot.collision_count == 0,
            "not_complete": mission.nav.status != "complete",
            "not_all_rooms_visited": not mission_report["all_rooms_visited"],
            "mission_does_not_pass": not mission_report["success"],
        }
        if expect_recovery:
            criteria["recovers_by_rgb_matches"] = bool(restored) and any(frame["tracking"] for frame in restored)
            criteria["recovery_is_not_a_new_pose_origin"] = any(frame["tracking"] and
                abs(frame["estimated_pose"]["yaw"]) > .05 for frame in restored)
        else:
            criteria["long_loss_remains_paused_after_rgb_restored"] = bool(restored) and all(
                not frame["tracking"] and frame["command"] == [0., 0.]
                and frame["navigation_state"] == "paused" for frame in restored)
            criteria["long_loss_explicitly_requires_reset"] = bool(restored) and all(
                frame["visual_reason"] == "reference_too_old_reset_required" for frame in restored)
            criteria["mission_terminates_with_explicit_visual_failure"] = (
                mission.finished and mission.failure == "visual_tracking_reset_required")
        report = {"trial_passed": all(criteria.values()), "criteria": criteria,
            "fault": {"mechanism": "replace actual RGB exposures with zeros; preserve timestamps",
                      "start_simulation_seconds": fault_start, "duration_seconds": duration,
                      "warmup_seconds": warmup, "requested_restored_rgb_seconds": recovered_seconds,
                      "restored_exposures_before_terminal_failure": len(restored)},
            "sensor_boundary": "RGB pixels plus image timestamps and fixed camera calibration only",
            "evaluation_only": {"pre_fault_command": last_pre_fault_command,
                "braking_path_length_m": braking_distance,
                "max_translation_after_first_blank_m": max_displacement,
                "true_pose_at_first_blank": None if first_blank_pose is None else vars(first_blank_pose),
                "final_true_pose": vars(final_pose)},
            "frames": {"warmup": len(warmups), "occluded": len(blanks), "restored": len(restored)},
            "mission": mission_report}
        write_json(destination / "report.json", report)
        write_json(destination / "diagnostics.json", diagnostics)
        write_json(destination / "exposures.json", exposures)
        write_json(destination / "calibration.json", mission.robot.calibration)
        np.save(destination / "occupancy.npy", mission.nav.grid)
        from PIL import Image
        for phase, pixels in sample_images.items():
            Image.fromarray(pixels).save(destination / f"{phase}.png")
        print(json.dumps(json_safe({"trial_passed": report["trial_passed"],
            "criteria": criteria, "frames": report["frames"],
            "evaluation_only": report["evaluation_only"], "output": str(destination)}), indent=2), flush=True)
        assert report["trial_passed"], {name: passed for name, passed in criteria.items() if not passed}
        return report
    finally:
        mission.robot.rgb.read = original_read
        mission.close()


def run(output=None, *, include_short=False):
    destination = Path(output) if output else ROOT / "artifacts" / "rgb-camera-dropout"
    long_report = run_case(destination)
    if include_short:
        short_report = run_case(destination / "short-recovery", duration=.4,
                                 expect_recovery=True)
        write_json(destination / "acceptance-summary.json", {
            "passed": long_report["trial_passed"] and short_report["trial_passed"],
            "long_loss": "report.json", "short_loss": "short-recovery/report.json"})
    return long_report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--include-short", action="store_true")
    args = parser.parse_args()
    run(args.output, include_short=args.include_short)
