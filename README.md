> **This project is in progress.**

# Mark Rover 01

Mark Rover 01 is a mobile robotics project for autonomous indoor navigation and 3D scene reconstruction. The objective is to acquire observations of indoor environments autonomously and reconstruct textured surfaces with validated collision geometry for mobile-robot simulation. The current acquisition stage uses manual driving and a phone browser.

## Scope

The project comprises three components:

| Directory | Function | Status |
| --- | --- | --- |
| [`wifi-control/`](wifi-control/) | Manual Wi-Fi control, ESP32 firmware, and paired laptop/phone image acquisition. | Driving tested on the rover; phone RGB acquisition and live video verified |
| [`auto-nav/`](auto-nav/) | Localization, mapping, path planning, obstacle avoidance, and autonomous motion control. | Not yet implemented |
| [`reconstruction/`](reconstruction/) | Grouped image datasets, surface reconstruction, texturing, collision assets and evaluation. | Image acquisition implemented; reconstruction not yet implemented |

The navigation component will determine the rover's motion. The reconstruction component will process captured observations and camera poses to produce a 3D scene representation. The current browser acquisition stores RGB images and capture metadata; it does not record camera poses, calibrated intrinsics or depth. The proposed autonomous sensing and reconstruction interfaces remain under development.

## Current implementation

The rover uses an ESP32-C3 and two L298N motor drivers to control four motors. A browser interface supports manual driving over Wi-Fi. Releasing input stops the rover; a firmware watchdog stops and disarms the motor-control session if valid commands are not received within 350 ms.

The laptop page also supports a paired phone camera, direct WebRTC video preview, individual photo capture, position groups and ZIP export. Images and JSON manifests are stored locally under the ignored `reconstruction/captures/` directory without a database. A physical S24 Ultra trial verified 4000 × 3000 JPEG uploads and approximately 26–30 fps live preview on the current local network. Automated tests cover storage, retries, capture cancellation and video reconnection.

Autonomous navigation, ARCore integration, and scene reconstruction have not been implemented. No autonomous-navigation or reconstruction results are reported at this stage.

## Running the manual controller

From the repository root:

```sh
python3 wifi-control/rover_control/serve.py
```

Open **http://127.0.0.1:8765/** on the laptop. The ESP32 and laptop must be on the same local network. Use WASD, the arrow keys, or the on-screen controls; Space or Escape stops and locks the controls.

See [`wifi-control/README.md`](wifi-control/README.md) for firmware and network setup. Wi-Fi credentials are stored locally and excluded from version control.

See [`reconstruction/capture/README.md`](reconstruction/capture/README.md) for phone HTTPS setup, capture workflow, storage format and validation limits.
