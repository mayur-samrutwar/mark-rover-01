# Manual Wi-Fi control and phone acquisition

Run **Start Control.command**, leave its terminal open and visit **http://127.0.0.1:8765/** on the laptop. The ESP32 and laptop use the same home network. The phone camera connects separately through the HTTPS address shown in the operator page.

## Driving

W/S or up/down arrows drive forward/reverse. A/D or left/right arrows turn left/right. Combine keys for arcs. Release input to stop. Space/Escape stops and locks the controls; Resume clears previous input. Drive strength starts at 50%.

The browser translates the installed driver channel order so A/D turn the physical rover in the intended direction. Motor rotation and speed are not measured. The ESP32 stops and disarms after 350 ms without valid commands. Idle status checks are separate from the driving heartbeat; idle recovery clears held input and does not arm the motors.

## Phone capture

The operator page adds a live phone preview, Capture, position groups and ZIP export. Start a new scan, capture overlapping photographs while rotating manually, finish the position, then drive to another position. Each acknowledged JPEG is saved immediately; finishing a position marks the group ready. Capture stops and temporarily disables driving. It never starts a motor or a rotation.

The phone service uses local HTTPS and a pairing code. Android needs a one-time installation of the laptop's public development CA certificate before its camera page can open securely. Setup instructions and the public certificate are served separately; private keys are never exposed. See [`../reconstruction/capture/README.md`](../reconstruction/capture/README.md).

Images and JSON manifests are stored under the repository's ignored `reconstruction/captures/` directory. This is an RGB acquisition tool; no database, cloud upload, ARCore tracking or reconstruction engine is included.

## Server

The operator interface and motor relay bind only to `127.0.0.1:8765`. The separate LAN HTTPS service on `8443` exposes phone acquisition routes only; the setup service on `8766` exposes instructions and the public certificate. Phone clients cannot access the motor relay.

If the ESP32 changes address, run:

```sh
python3 rover_control/serve.py --rover http://NEW_IP
```

Run `python3 rover_control/serve.py --no-capture` for the control-only interface. The firmware served directly by the ESP32 does not receive these laptop-side changes until a future firmware asset rebuild and USB upload. No firmware upload is required for this feature.

## Files

- `rover_control/`: operator and phone pages, local relay, capture store, TLS setup and tests.
- `firmware/rover_wifi_control/`: existing ESP32 firmware.
- `firmware/rover_wifi_control/wifi_credentials.h`: private configuration excluded from Git.

The capture receiver requires Pillow; see `rover_control/requirements-capture.txt`. Capture data and local TLS certificates are excluded from version control.
