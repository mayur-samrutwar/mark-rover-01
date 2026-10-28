"""Negative acceptance trial: a closed bedroom door must prevent mission success.

Runs the exact rover and normal sensor-only Navigator. Only a static door-sized
collision box is added to the furnished seed-7 world. Room targets and evaluator
criteria stay intact, so exhaustion of reachable frontiers cannot masquerade as
visiting the inaccessible bedroom.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from rover_nav.mission import Mission, ROOT
from rover_nav.world import generate_world


def run(output=None, max_seconds=1200., *, sensing="ideal"):
    world = generate_world(7)
    door = next(d for d in world["source"]["doors"]
                if set(d["rooms"]) == {"study", "bedroom"})
    x, y = door["center"]
    world["boxes"].append({"center": [x, y, .9],
                           "size": [door["width"] + .16, .16, 1.8],
                           "color": [.5, .29, .14, 1.]})
    world["name"] += " · bedroom door closed · negative trial"
    world["source"]["negative_trial"] = {"closed_door": door,
        "expected": "Bedroom remains unvisited and success must be false"}
    destination = Path(output) if output else ROOT / "artifacts" / "blocked-door"
    destination.mkdir(parents=True, exist_ok=True)
    if sensing == "rgb":
        # Scene placement only; perception still starts at arbitrary (0,0,0).
        # Explicitly supplied worlds do not receive Mission's default placement.
        world["source"]["spawn"] = [1.2, 1.3, 0.]
    mission = Mission(7, world=world, sensing=sensing,
                      capture_dir=destination / "captures" if sensing == "rgb" else None)
    shutil.copyfile(__file__, destination / "closed_door_trial.py")
    try:
        while not mission.finished and mission.robot.data.time < max_seconds:
            mission.tick()
            if mission.steps % 100 == 0:
                (destination / "progress.json").write_text(json.dumps({
                    "time": float(mission.robot.data.time), "state": mission.nav.status,
                    "reason": mission.nav.reason, "steps": mission.steps,
                    "distance_m": mission.robot.distance,
                    "collision_count": mission.robot.collision_count,
                    "rooms": [r["name"] for r in mission.rooms if r["visited"]],
                    "source_sha256_at_start": mission.source_hashes}, indent=2))
            if mission.steps % 500 == 0:
                print(json.dumps({"time": round(float(mission.robot.data.time), 1),
                                  "state": mission.nav.status,
                                  "rooms": [r["name"] for r in mission.rooms if r["visited"]]}),
                      flush=True)
        if not mission.finished:
            mission.finished, mission.failure = True, "negative_trial_time_limit"
            mission.robot.drive(0., 0.)
            mission.robot.advance(.5)
        report = mission.save(destination)
        current_sources = {name: hashlib.sha256((ROOT / "rover_nav" / name).read_bytes()).hexdigest()
                           for name in mission.source_hashes}
        source_verification = {"verified": current_sources == mission.source_hashes,
            "reported_production_sources": mission.source_hashes,
            "matches_sources_after_trial": current_sources == mission.source_hashes,
            "production_source_count": len(mission.source_hashes),
            "trial_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        (destination / "source-verification.json").write_text(json.dumps(source_verification, indent=2))
        rooms = {room["name"]: room for room in report["rooms"]}
        # A timeout or early sensor failure is not a passing inaccessible-room
        # test. Normal exploration must exhaust the accessible living+study area.
        criteria = {
            "no_physical_contact": report["collision_count"] == 0,
            "living_room_physically_entered": rooms["living_room"]["visited"],
            "study_physically_entered": rooms["study"]["visited"],
            "bedroom_not_entered": not rooms["bedroom"]["visited"],
            "not_all_rooms_visited": not report["all_rooms_visited"],
            "mission_success_false": not report["success"],
            "accessible_exploration_complete": report["navigation_state"] == "complete",
            "no_timeout_or_other_failure": report["failure"] is None,
            "production_sources_unchanged": source_verification["verified"],
        }
        if sensing == "rgb":
            criteria["all_nine_rgb_production_sources_recorded"] = len(mission.source_hashes) == 9
            criteria["rgb_captures_saved"] = report["captures_saved"] > 0 and report["capture_complete"]
        summary = {"negative_trial_passed": all(criteria.values()), "criteria": criteria,
                          "sensing": sensing, "navigation_state": report["navigation_state"],
                          "failure": report["failure"],
                          "mission_success": report["success"], "all_rooms_visited": report["all_rooms_visited"],
                          "rooms": report["rooms"], "coverage": report["coverage"],
                          "collision_count": report["collision_count"],
                          "simulation_seconds": report["simulation_seconds"],
                          "output": str(destination)}
        (destination / "acceptance-summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
        assert summary["negative_trial_passed"], {name: value for name, value in criteria.items() if not value}
        return report
    finally:
        mission.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-seconds", type=float, default=1200.)
    parser.add_argument("--sensing", choices=["ideal", "rgb"], default="ideal")
    arguments = parser.parse_args()
    run(arguments.output, arguments.max_seconds, sensing=arguments.sensing)
