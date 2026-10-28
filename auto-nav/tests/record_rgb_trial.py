"""Record real RGB inputs and separate evaluator truth for repeatable audits.

The recorded truth is written beside images; it never crosses RGBPacket or
CameraPerception. Run with the simulator Python and auto-nav on PYTHONPATH.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from PIL import Image

from rover_nav.mission import Mission


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-seconds", type=float, default=1400.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    images = output / "rgb_inputs"
    images.mkdir(parents=True, exist_ok=True)
    mission = Mission(args.seed, sensing="rgb", capture_dir=output / "captures")
    (output / "camera_calibration.json").write_text(json.dumps(mission.robot.calibration, indent=2))
    original_read = mission.robot.rgb.read
    last_stamp, frame_count = None, 0
    manifest = (output / "rgb_inputs.jsonl").open("w")

    def recorded_read():
        nonlocal last_stamp, frame_count
        packet = original_read()
        if packet.timestamp != last_stamp:
            name = f"rgb_inputs/{frame_count:06d}.png"
            Image.fromarray(packet.pixels).save(output / name)
            # Independent observer: actual pose is written only to disk.
            truth = asdict(mission.robot.pose())
            manifest.write(json.dumps({"image":name, "timestamp":packet.timestamp,
                "evaluator_true_pose":truth}) + "\n")
            frame_count += 1
            if frame_count % 50 == 0:
                manifest.flush()
            last_stamp = packet.timestamp
        return packet

    mission.robot.rgb.read = recorded_read
    last_progress = -10.
    try:
        try:
            while not mission.finished and mission.robot.data.time < args.max_seconds:
                mission.tick()
                if mission.robot.data.time - last_progress >= 10.:
                    last_progress = float(mission.robot.data.time)
                    progress = {"time":last_progress, "state":mission.nav.status,
                        "reason":mission.nav.reason, "estimate":asdict(mission.latest_observation.pose),
                        "truth":asdict(mission.robot.pose()), "rooms":mission.rooms,
                        "collisions":mission.robot.collision_count,
                        "position_error_m":mission.localization_errors[-1][0],
                        "perception":mission.robot.perception.status()}
                    (output / "progress.json").write_text(json.dumps(progress, indent=2))
                    print(json.dumps(progress), flush=True)
        except KeyboardInterrupt:
            mission.finished, mission.failure = True, "operator_interrupted"
        if not mission.finished:
            mission.finished, mission.failure = True, "test_time_limit"
        mission.robot.drive(0., 0.)
        mission.robot.advance(.5)
        report = mission.save(output)
        (output / "rgb_recording.json").write_text(json.dumps({"frames":frame_count,
            "input_contract":"RGB pixels and timestamp only",
            "truth_role":"Independent evaluator file only; never a controller input",
            "calibration":mission.robot.calibration}, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        return 0 if report["success"] else 1
    finally:
        manifest.close()
        mission.close()


if __name__ == "__main__":
    raise SystemExit(main())
