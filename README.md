> **This project is in progress.**

# Mark Rover 01

Mark Rover 01 is a mobile robotics project for autonomous indoor navigation and 3D scene reconstruction. The objective is to navigate an indoor environment, acquire observations with estimated camera poses, and reconstruct the observed scene using 3D Gaussian splatting.

## Scope

The project comprises three components:

| Directory | Function | Status |
| --- | --- | --- |
| [`wifi-control/`](wifi-control/) | Manual control over a local Wi-Fi network; ESP32 firmware and browser interface. | Implemented and tested on the rover |
| [`auto-nav/`](auto-nav/) | Localization, mapping, path planning, obstacle avoidance, and autonomous motion control. | Not yet implemented |
| [`reconstruction/`](reconstruction/) | Data acquisition, camera-pose estimation, and 3D scene reconstruction using Gaussian splatting. | Not yet implemented |

The navigation component will determine the rover's motion. The reconstruction component will process captured observations and camera poses to produce a 3D scene representation. Their interfaces, sensor requirements, and evaluation methods remain to be defined.

## Current implementation

The rover uses an ESP32-C3 and two L298N motor drivers to control four motors. A browser interface supports manual driving over Wi-Fi. Releasing input stops the rover; a firmware watchdog stops and disarms the motor-control session if valid commands are not received within 350 ms.

Autonomous navigation, perception hardware integration, and scene reconstruction have not been implemented. No autonomous-navigation or reconstruction results are reported at this stage.

## Running the manual controller

From the repository root:

```sh
python3 wifi-control/rover_control/serve.py
```

Open **http://127.0.0.1:8765/** on the laptop. The ESP32 and laptop must be on the same local network. Use WASD, the arrow keys, or the on-screen controls; Space or Escape stops and locks the controls.

See [`wifi-control/README.md`](wifi-control/README.md) for firmware and network setup. Wi-Fi credentials are stored locally and excluded from version control.
