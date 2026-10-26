# Manual image acquisition

This component records grouped photographs from a phone mounted on the rover. The laptop keeps its existing manual driving interface and receives observations over the home network. Images and metadata are stored as ordinary files; no database or external service is required.

This is an acquisition tool. It does not reconstruct a scene, estimate camera poses, provide depth, or drive the rover autonomously.

## Roles

| Surface | Role |
|---|---|
| Laptop operator page | Drive the rover, pair a phone, view its camera preview, capture observations and manage position groups. |
| Phone capture page | Request permission for the rear camera, display its preview and upload requested photographs. |
| Laptop server | Coordinate capture requests, receive images and maintain session and group manifests on disk. |

The phone and laptop must be on the same local network. Keep the phone page open and its screen awake during acquisition. Moving either page into the background can suspend camera or network activity.

## Start

The capture receiver requires Pillow (`python3 -m pip install -r wifi-control/rover_control/requirements-capture.txt`) and the locally installed `mkcert` tool.

Run the existing **Start Control.command**, or start the server explicitly:

```sh
python3 wifi-control/rover_control/serve.py
```

Open **http://127.0.0.1:8765/** on the laptop. The operator interface remains local to this computer. Its motor commands continue through the existing ESP32 Wi-Fi relay; the phone camera connection does not control motors.

Phone camera access requires a secure browser context. The server prepares a local HTTPS certificate using the laptop's existing `mkcert` certificate authority. It does not install certificates or change trust settings automatically.

If address discovery chooses the wrong network interface, specify the laptop's LAN address:

```sh
python3 wifi-control/rover_control/serve.py --capture-host 192.168.1.25
```

Relevant options:

| Option | Default | Purpose |
|---|---|---|
| `--port` | `8765` | Laptop operator page on loopback. |
| `--rover` | `http://192.168.1.6` | Existing ESP32 address. |
| `--capture-host` | Detected LAN IPv4 address | Address placed in phone links and the HTTPS certificate. |
| `--https-port` | `8443` | Phone camera and capture service over HTTPS. |
| `--setup-port` | `8766` | Local-network setup page and public CA download. |
| `--no-capture` | Off | Run the existing motor controller without the capture service. |

Certificate or capture startup errors appear in the server terminal without preventing manual rover control. Capture panels appear when the phone service is available.

## Connect the phone

1. On the laptop operator page, obtain the phone setup link and current pairing link/code.
2. On the phone, open **http://LAPTOP_ADDRESS:8766/**. This bootstrap page serves setup information and the public CA certificate only.
3. Download `mark-rover-capture-ca.crt`. In Android Settings, search for **Install a certificate**, choose **CA certificate**, and select that file. Android displays its own confirmation before adding the CA. The precise Settings path depends on the Android version.
4. Return to Chrome and open the pairing link, shaped like **https://LAPTOP_ADDRESS:8443/phone/?code=123456**. Use the current code from the laptop rather than this example.
5. Allow camera access, select the rear camera if necessary, and keep the page visible. Confirm that the laptop receives its preview before beginning a scan.

The downloaded certificate is the public root certificate of the local development CA created by `mkcert`. Installing it lets Chrome verify this laptop's HTTPS certificate; it grants trust to certificates signed by that CA. The CA private key stays in the laptop's existing mkcert directory and is never copied or served. The HTTPS server private key also stays on the laptop. Remove the installed CA from Android's user credentials when it is no longer needed.

Local certificates are kept in the ignored `.capture-tls/` directory at the repository root. The helper includes the selected LAN host, `localhost` and `127.0.0.1` in the server certificate. It reuses a valid certificate and renews when the host changes or expiry is within seven days. A changed laptop address requires using the updated phone link. The existing CA normally does not need to be installed again.

For certificate diagnostics only:

```sh
python3 wifi-control/rover_control/capture_tls.py --host 192.168.1.25
```

This prints the selected host and public certificate paths. It never changes trust settings.

## Record a room

1. Start a capture session and a position group while the rover is stopped.
2. Check the preview and capture one sharp photograph.
3. Turn the rover manually in small increments, releasing the direction input and allowing the image to settle before each photograph.
4. Capture roughly 10–15 overlapping views for the group. Confirm each upload before finishing it.
5. Finish the group, drive to a different position, stop, and begin the next group.
6. Repeat, including views that overlap with the preceding position. Finish the final position when acquisition is complete, then export the scan. New scan preserves earlier scans on disk.

Rotation supplies directional coverage. Translation between positions is necessary to observe surfaces from different locations and support geometric reconstruction. Include a known physical measurement in the scene so a later image-only reconstruction can be checked and scaled. Capture visible structure and texture rather than relying only on blank walls. Avoid changing camera zoom between observations.

Capturing an image never initiates a turn or other motor movement. The existing controls, release-to-stop behaviour and motor watchdog remain responsible for manual driving.

## Live preview and original photographs

The laptop receives live video directly from the paired phone through WebRTC. The local Python server exchanges the connection descriptions over the existing operator and authenticated phone endpoints. It does not relay or store the video. No database, public signalling service, STUN server or TURN relay is required for the direct connection on a compatible local network.

Live preview and saved photographs use separate paths. Preview uses a bounded video stream; Capture still requests an original JPEG through the existing image upload transaction. WebRTC does not replace the recorded dataset with video frames. A phone's native still-photo operation can briefly interrupt its video stream.

If the browser cannot establish direct video, or received frames stop advancing, the interface falls back to the lower-rate JPEG preview and retries the video connection. A frozen image is labelled as paused rather than presented as live. Preview reconnects do not issue motor commands or release a capture lock.

Keep both pages visible. Browser background suspension, weak Wi-Fi, guest-network isolation, host-candidate restrictions or local firewall rules can prevent or interrupt direct video. The JPEG fallback is intentionally less smooth than live video. Real phone performance must be checked on the target network; local browser tests do not establish a phone frame rate or end-to-end latency.

## Files and metadata

The server stores acquisitions under the repository's ignored `reconstruction/captures/` directory:

```text
reconstruction/captures/
  SESSION_ID/
    session.json
    position-001/
      manifest.json
      0001_CAPTURE_ID.jpg
      0001_CAPTURE_ID.json
      0001_CAPTURE_ID.thumb.jpg
    position-002/
      ...
```

Session and group manifests describe the acquisition and its photographs. Keep these records with the images when transferring a dataset. Each capture must be acknowledged by the server before it is considered saved. Closing or reloading a page during an upload can interrupt that observation.

No database is used. The directory tree is the dataset, while JSON manifests provide its index. Recordings are not committed to Git.

This first version captures RGB images and browser-provided acquisition metadata. It does not record ARCore camera poses, calibrated camera intrinsics, IMU samples, metric depth or depth confidence. A later photogrammetry stage can estimate camera poses from overlapping images; the uploaded photographs alone do not establish metric scale or a simulation-ready environment.

## Network and privacy

The phone interface is accessible on the LAN, paired with the code shown locally on the laptop. Treat that code and the pairing link as local access credentials. The phone endpoint is for camera acquisition, and motor-control endpoints remain restricted to the laptop's loopback interface.

Images stay on the phone/laptop network and are saved on the laptop. There is no cloud upload, external signalling service or account requirement. The setup HTTP service only supplies instructions and the public certificate. Original photographs and fallback previews use HTTPS; direct video uses WebRTC transport encryption.

If the phone cannot open the setup page, check that both devices share the same network and that guest-network or access-point isolation is disabled. Check the server output and macOS firewall permission for the Python server. If camera access fails, confirm that the installed CA matches this laptop, the phone uses the current HTTPS link, and Chrome has camera permission. Certificate error bypass pages are not a substitute for a trusted HTTPS connection.

## Validation status

Automated server and browser checks verify request handling, pairing, group storage and camera behaviour with simulated input. The WebRTC integration test sends an animated canvas through two real browser peer connections, checks that decoded frames advance, preserves a separate original photograph, and exercises reload, camera restart, JPEG fallback and recovery. It uses temporary HTTP/TLS servers and intercepts all rover requests; it cannot move the hardware. A physical S24 Ultra trial verified local certificate setup, camera permission, 4000 × 3000 JPEG uploads and direct video at approximately 26–30 displayed frames per second on the current LAN. Video resumed after a full-resolution still capture. These observations do not establish worst-case latency, performance on other networks, background reliability or reconstruction quality. No reconstructed scene or autonomous navigation result is claimed by this component.
