> **This project is in progress.**

# Mark Rover 01

Mark Rover 01 is a mobile robotics project for autonomous indoor navigation and 3D scene reconstruction. The objective is to acquire observations of indoor environments autonomously and reconstruct textured surfaces with validated collision geometry for mobile-robot simulation. The current acquisition stage uses manual driving and a phone browser.

## Scope

The project comprises three components:

| Directory | Function | Status |
| --- | --- | --- |
| [`wifi-control/`](wifi-control/) | Manual Wi-Fi control, ESP32 firmware, and paired laptop/phone image acquisition. | Driving tested on the rover; phone RGB acquisition and live video verified |
| [`auto-nav/`](auto-nav/) | Localization, mapping, path planning, obstacle avoidance, and autonomous motion control. | IsaacProtocol simulation implemented and tested; hardware autonomy pending |
| [`reconstruction/`](reconstruction/) | Grouped image datasets, surface reconstruction, texturing, collision assets and evaluation. | Acquisition and local reconstruction implemented; initial room comparison complete, geometric validation pending |

The navigation component will determine the rover's motion. The reconstruction component will process captured observations and camera poses to produce a 3D scene representation. The current browser acquisition stores RGB images and capture metadata; it does not record camera poses, calibrated intrinsics or depth. The proposed autonomous sensing and reconstruction interfaces remain under development.

## Current implementation

The rover uses an ESP32-C3 and two L298N motor drivers to control four motors. A browser interface supports manual driving over Wi-Fi. Releasing input stops the rover; a firmware watchdog stops and disarms the motor-control session if valid commands are not received within 350 ms.

The laptop page also supports a paired phone camera, direct WebRTC video preview, individual photo capture, position groups and ZIP export. Images and JSON manifests are stored locally under the ignored `reconstruction/captures/` directory without a database. A physical S24 Ultra trial verified 4000 × 3000 JPEG uploads and approximately 26–30 fps live preview on the current local network. Automated tests cover storage, retries, capture cancellation and video reconnection.

The reconstruction module now provides a separate local job service and browser viewer for Apple photogrammetry and COLMAP/OpenMVS mesh reconstruction. A same-image room comparison selected Apple photogrammetry as the current default on supported Macs; surface accuracy and apartment coverage remain unverified. Native tool verification and fixture tests are distinct from evaluation on apartment captures. Autonomous navigation has passed three furnished-room simulation trials using the original rover/phone model; physical ARCore and ESP32 integration remain pending. See [`auto-nav/`](auto-nav/) for the running simulation and measured results.

## Open the project

Run **Start Rover.command** or, from the repository root:

```sh
.venv-reconstruction/bin/python rover.py start --open
```

Always open [http://rover.localhost:1355/](http://rover.localhost:1355/). Control is the home page; click **Reconstruction** for saved scans and models under the same address. The project uses [Vercel Portless](https://github.com/vercel-labs/portless) with the existing local proxy on port 1355.

The launcher installs a macOS user LaunchAgent. It starts at login, keeps control and reconstruction running, and restarts either process if it exits. Closing the launcher terminal does not stop the project. Existing compatible services are reused. Logs and the persistent pairing code live in the ignored `.rover-runtime/` directory. Startup does not acquire a motor session or issue drive commands.

`wifi-control/Start Control.command` and `reconstruction/Start Reconstruction.command` open this same project entry point. The phone still uses the HTTPS camera link displayed on Control: `.localhost` on the phone refers to the phone itself, so use the displayed LAN link and pairing code.

```sh
.venv-reconstruction/bin/python rover.py status
# Stop the supervised services and remove login startup:
.venv-reconstruction/bin/python rover.py stop
```

Internal loopback ports 8765 and 8768 remain implementation details. The phone camera and certificate setup retain ports 8443 and 8766. Separate navigation, playback, trial-viewing and floor-calibration servers have been removed; offline algorithms and stored evidence remain under `auto-nav/`.

See [wifi-control/README.md](wifi-control/README.md) for firmware and driving, [reconstruction/capture/README.md](reconstruction/capture/README.md) for phone setup, and [reconstruction/README.md](reconstruction/README.md) for processing and model limitations.
