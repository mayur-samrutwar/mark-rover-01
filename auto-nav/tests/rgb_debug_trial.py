"""Save the first RGB navigation stop for offline image/map diagnosis."""
import json
import sys
from pathlib import Path

from rover_nav.mission import Mission

output = Path(sys.argv[1])
mission = Mission(seed=7, capture_dir=output / "captures", sensing="rgb")
try:
    while not mission.finished and mission.robot.data.time < 180:
        mission.tick()
        if mission.nav.status == "paused" or mission.nav.reason in {
            "local_obstacle_stop", "braking_path_unknown", "insufficient_footprint_clearance"
        }:
            break
    print(json.dumps(mission.save(output), indent=2), flush=True)
finally:
    mission.close()
