# Manual Wi-Fi control and phone acquisition

Run **Start Rover.command** from the repository root (the existing **Start Control.command** also opens it). Always visit [http://rover.localhost:1355/](http://rover.localhost:1355/) on the laptop. The startup service keeps control and reconstruction running independently of the launcher terminal and restarts them after exit. The ESP32 and laptop use the same home network. The phone camera uses the HTTPS LAN address shown on Control.

## Driving

W/S or up/down arrows drive forward/reverse. A/D or left/right arrows turn left/right. Combine keys for arcs. Release input to stop. Space/Escape stops and locks the controls; Resume clears previous input. Drive strength starts at 50%.

The browser translates physical wheel commands to the current installed wiring by negating both channels without swapping sides. This corrects the reported reversed W/S directions while retaining the existing A/D commands. The Python hardware transport uses the same conversion. The user confirmed W moves forward; both left wheels still fail reverse with the wheels raised at 90%, which remains a separate hardware diagnosis. Motor rotation and speed are not measured. The ESP32 stops and disarms after 350 ms without valid commands. The manual slider allows 0–100%; calibration and autonomous limits are configured separately.

A failed driving request clears held input and the current motor session. The page checks reachability with read-only idle status requests; a healthy response restores idle controls. Driving then requires a fresh direction press and a new session. Recovery never replays the previous direction. Space/Escape and the Stop button remain locked until **Resume**; a late health response cannot unlock them. Releasing a direction during a connection attempt also prevents that direction from running after the handshake.

## Phone capture

The operator page adds a live phone preview, Capture, position groups and ZIP export. Start a new scan, capture overlapping photographs while rotating manually, finish the position, then drive to another position. Each acknowledged JPEG is saved immediately; finishing a position marks the group ready. Capture stops and temporarily disables driving. It never starts a motor or a rotation.

The phone service uses local HTTPS and a pairing code. Android needs a one-time installation of the laptop's public development CA certificate before its camera page can open securely. Setup instructions and the public certificate are served separately; private keys are never exposed. See [`../reconstruction/capture/README.md`](../reconstruction/capture/README.md).

Images and JSON manifests are stored under the repository's ignored `reconstruction/captures/` directory. This is an RGB acquisition tool; no database, cloud upload, ARCore tracking or reconstruction engine is included.

## Server

The operator interface and motor relay bind only to `127.0.0.1:8765`. The separate LAN HTTPS service on `8443` exposes phone acquisition routes only; the setup service on `8766` exposes instructions and the public certificate. Phone clients cannot access the motor relay or its diagnostics.

The current default rover target is `http://192.168.1.5`. The relay uses this explicitly configured HTTP(S) origin directly, with no environment proxy, redirect following, automatic address changes or retries. URLs containing credentials, a path, query or fragment are rejected.

If the ESP32 changes address, run:

```sh
python3 rover_control/serve.py --rover http://NEW_IP
```

Run `python3 rover_control/serve.py --no-capture` for the control-only interface. Laptop relay changes take effect after restarting the server. Updating the controller assets served directly by the ESP32 requires an asset rebuild and firmware upload; the current recovery UI was included in the verified USB upload on 2026-10-28.

Restart the `serve.py` process after changing relay code or the target address. The separate `auto-nav` monitor and phone-camera service do not need a restart for a manual relay update.

## Connection diagnostics

Open these on the laptop:

- [`/api/relay/status`](http://127.0.0.1:8765/api/relay/status) reads cached diagnostics without contacting the rover.
- [`/api/relay/probe`](http://127.0.0.1:8765/api/relay/probe) makes exactly one read-only `GET /api/info`. It does not acquire a session, send a motor command or forward a session token.

Diagnostics retain at most 64 events, including the configured target, elapsed time, deadline, upstream HTTP status, result code, delivery uncertainty and recent failure count. They distinguish timeouts, refused connections, DNS failures, unreachable addresses, resets, HTTP errors and invalid responses. Headers, command bodies, credentials and session tokens are excluded. Firmware HTTP 401/409 responses retain their status so an expired or refused lease can be identified.

Each relay request makes one upstream attempt. Its total deadline includes the control upload and upstream exchange:

| Request | Relay deadline |
| --- | --- |
| Motor command or STOP | 200 ms |
| Session acquisition or read-only info | 1.2 s |

These precede the browser's 300 ms driving and 2 s connection/idle deadlines. STOP can run while another upstream request is blocked. A failed or timed-out POST has **unknown delivery**; the ESP32's independent 350 ms lease remains responsible for stopping if STOP cannot reach it. Relay diagnostics identify request failures; they do not measure rover motion or establish that a Wi-Fi fault is resolved.

## Files

- `rover_control/`: operator and phone pages, local relay, capture store, TLS setup and tests.
- `firmware/rover_wifi_control/`: existing ESP32 firmware.
- `firmware/rover_wifi_control/wifi_credentials.h`: private configuration excluded from Git.

The capture receiver requires Pillow; see `rover_control/requirements-capture.txt`. Capture data and local TLS certificates are excluded from version control.
