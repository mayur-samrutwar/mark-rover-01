> **This project is in progress.**

# Mark Rover 01

An indoor rover project working toward autonomous exploration, navigation, and 3D reconstruction of its surroundings.

The goal is to let the rover move through an area, estimate its position, plan routes, and avoid obstacles while gathering observations of the environment. Those observations will support building a 3D representation of the space, including Gaussian splatting for scene reconstruction and visualization.

Manual Wi-Fi control is working. Autonomous navigation and 3D reconstruction are the next development stages.

## Project structure

| Folder | Purpose | Status |
| --- | --- | --- |
| [`wifi-control/`](wifi-control/) | Manually drive the rover over a local Wi-Fi network using keyboard or on-screen controls. Includes the ESP32 firmware and browser controller. | Working |
| [`auto-nav/`](auto-nav/) | Develop autonomous navigation: localization, mapping, route planning, obstacle avoidance, and motion control. | Planned |
| [`reconstruction/`](reconstruction/) | Develop the environment-reconstruction pipeline: observation capture, camera-pose alignment, and 3D scene reconstruction, including Gaussian splatting. | Planned |

Navigation and reconstruction are separate development areas: navigation determines where the rover can move, while reconstruction builds a representation of what the rover observes. The eventual goal is to connect them in a shared exploration workflow.

## Manual Wi-Fi control

1. Power the ESP32 and connect it to the same home network as the laptop.
2. Run [`wifi-control/Start Control.command`](wifi-control/Start%20Control.command), or run `python3 wifi-control/rover_control/serve.py` from this repository.
3. Open **http://127.0.0.1:8765/** and connect.
4. Hold WASD, the arrow keys, or an on-screen direction to move. Release to stop. Use Space or Escape for an emergency stop.

See the [controller setup](wifi-control/README.md) for firmware, network configuration, and operating details. Network credentials remain local and are excluded from Git.
