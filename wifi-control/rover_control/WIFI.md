# Home Wi-Fi control

The ESP32 joins the same home Wi-Fi as your laptop and serves this controller directly. Its USB power can come from the laptop, a suitable phone USB output, or the verified power-bank port; the phone camera connects separately over Wi-Fi. The existing power bank supplies the drivers. The 5V motor-driver rail must not be connected to ESP32 VBUS/5V while USB powers it. Keep the existing common ground and verified driver wiring.

1. Create `firmware/rover_wifi_control/wifi_credentials.h` from the example header and enter your network name and password. The real header is ignored by Git. The ESP32-C3 needs the network's 2.4GHz service; the laptop can use either band on the same LAN.
2. Run `python3 rover_control/build_wifi_assets.py` to embed the HTML/CSS/JS in flash.
3. Upload `firmware/rover_wifi_control/rover_wifi_control.ino` as **ESP32C3 Dev Module**, **USB CDC On Boot Enabled**, using Arduino-ESP32 3.x.
4. Open Serial Monitor at 115200 and send `INFO` to see the assigned IP address. The network password is never printed. The board also advertises `http://rover.local/` through mDNS; use the numeric IP if that name doesn't resolve.
5. For the current white controller, run `python3 rover_control/serve.py` and open `http://127.0.0.1:8765/` on the laptop. Its current default target is `http://192.168.1.5`; pass `--rover http://NEW_IP` if the board's reported address changes. Click **Connect**, and hold WASD or the on-screen buttons. The page can also be embedded in the ESP32 by rebuilding assets and uploading the firmware again.
6. After verifying Wi-Fi, move the ESP32's USB power to the verified power-bank port or to a phone that supports supplying USB power (OTG/reverse charging as appropriate). The ESP32 reboots stopped and rejoins home Wi-Fi. The updated laptop controller restores idle controls automatically once the same configured address responds with a disarmed, zero-output state. A changed IP requires updating `--rover`.

Start with all wheels raised. W/S drive forward/reverse, A/D pivot, combined keys arc, no key stops, Space/Escape latches stop. Default PWM is 128/255. Actual wheel speed/position is not measured.

The ESP32's independent motor task checks Wi-Fi and a 350ms command lease every 5ms. HTTP requests cannot prevent that task from stopping the motors. After lease expiry, the session is disarmed; delayed commands cannot restart motion. Each connection has a random token and strictly increasing sequence. A second controller is refused while the first session is active. Stopping/resuming clears all held input and requires a fresh control session. Direction reversals coast for 40ms. This is local HTTP control on the home network, not an internet-facing service; the firmware does not expose any Wi-Fi credentials through its web API.

The original USB controller remains at `/usb.html` on the laptop development server, with its original USB firmware in `firmware/rover_keyboard_control`.

This board/router combination uses 802.11b/g mode and 8.5dBm transmit power. The default 802.11n mode repeatedly lost beacons during the Wi-Fi-only diagnostic. This compatibility setting changes radio modes, not the network password or WPA2 authentication. `INFO` includes the current RSSI and the most recent disconnect reason; the latter is historical and can remain nonzero after reconnection.

While idle, the page leaves the motor session disarmed and checks network reachability once per second with a 2-second request timeout. A newly held direction establishes a fresh session; releasing input sends STOP and disarms it again. Held input is checked again after the handshake, so releasing during connection cannot replay an old movement. Driving retains the 100ms heartbeat, 300ms request deadline, and the ESP32's independent 350ms motor lease. Idle status delays therefore do not trip the driving timeout.

## Laptop relay and browser recovery

A failed driving request clears held input and the current motor session. Healthy read-only status polling restores idle controls; a fresh direction press establishes the next session. Recovery never restores the previous movement. The operator's Stop/Space/Escape latch remains locked until **Resume**, including when an older idle response arrives late. Responses from a disconnected or superseded connection attempt cannot change the current controls.

The laptop relay connects directly to its explicitly configured HTTP(S) origin. It ignores environment proxies, refuses redirects and never retries or replays a motor request. Target URLs cannot contain credentials, a path, query or fragment. Control upload and upstream exchange share one total deadline: **200ms for commands and STOP**, **1.2s for session acquisition and info**. STOP runs independently of other blocked requests. Upstream responses are size-limited and checked for the rover protocol; HTTP 401/409 control errors preserve their status.

The custom TCP socket explicitly enables `TCP_NODELAY` before connecting, as the
standard Python HTTP client normally does. Without this, separate header/body
writes delayed small POSTs by about 200ms. After the correction, ten real
zero-output commands all acknowledged within 24ms; a subsequent 90% raised-wheel
input produced command acknowledgements within 20ms and a confirmed STOP.

For local diagnostics, open `http://127.0.0.1:8765/api/relay/status`. This GET reads a cached history of at most 64 events without contacting the rover. `http://127.0.0.1:8765/api/relay/probe` performs exactly one `GET /api/info`, with no session or motor command. Both routes are available only on the laptop's operator surface; phone and setup services cannot use them.

The history shows the target, elapsed time and deadline, upstream HTTP status, controlled error code and failure count. Codes distinguish timeout, connection refusal, DNS failure, unreachable network/address, connection reset, HTTP error and invalid response. Tokens, headers and raw command bodies are not retained. A failed or timed-out POST is marked **unknown delivery**; it does not prove that the command was never received. The firmware's independent 350ms lease remains active if STOP cannot be delivered.

Restart `serve.py` to load laptop relay changes, retaining the intended `--rover` address. This does not require restarting the separate `auto-nav` camera service or uploading firmware. Updating assets served directly by the ESP32 requires rebuilding and uploading firmware; that was completed for the current recovery UI on 2026-10-28.

## Firmware recovery and diagnostics

The stock HTTP parser's abandoned-request stall was also reproduced on this
board: a closed, incomplete GET delayed the next valid request by 5009ms while
ping remained healthy. The uploaded firmware now uses a repository-local
bounded admission layer. It accepts one complete, validated request within
150ms (headers at most2048 bytes; body at most1024), then parses an immutable,
socket-free snapshot. Empty/partial clients cannot enter the stock five-second
parser wait. Transfer encoding, multipart bodies and pipelined requests are
not part of this rover protocol and are rejected. Motor pins and the independent
350ms lease are unchanged. On-device regressions for an abandoned request line,
abandoned body and idle connection completed the next healthy request in
112–115ms. The fix passed host tests,211 internal checks, sanitizers and source
review before the verified USB upload.

Admission consumes and assembles incoming chunks without blocking; it does not
assume headers and body arrive in one packet. The host regression limits each
receive to seven bytes. On the real ESP32, normal direct and relayed sessions,
a request with its body delayed30ms, and ten zero-output commands all passed.
The command round trips stayed below25ms in that check.

The recovery changes below were compiled for ESP32-C3 with USB CDC enabled and
uploaded over USB on 2026-10-28; flash verification passed. The board rejoined
at `192.168.1.5` about 1.4 seconds after boot. During the upload, the actual
controller showed Reconnecting and then recovered to Connected / Stopped
without a new direction press or movement replay. This verifies restart
recovery on laptop USB; it does not establish power-bank or motor-load stability.

The later shared-bank check rejoined successfully, but the controller then
reported a fresh `power_on` reset during the user's 100% driving inputs. The
user confirmed they had not touched power or RESET. This points to the
controller supply/reset path under motor load; it does not identify the exact
bad cable, port, wiring point or component. With the ESP32 on laptop USB and
the motors on the bank, a brief 90% raised-wheel test acknowledged three
commands and STOP, with the same boot ID throughout. Keep this electrical
fault separate from browser recovery and the resolved relay latency defect.

The source retains the motor pins, direction polarities, PWM behavior and 350ms
lease. DISCONNECTED, LOST_IP and STA_STOP latch a stop generation: a quick
disconnect/reconnect between motor-task ticks cannot preserve the previous
armed session. Reconnection never arms a motor session.

The normal Arduino automatic recovery remains enabled. A fallback covers
disconnect reasons that the core does not automatically retry. It waits 30s
after connection progress and spaces repeated attempts by 30s, 60s, then at most
120s. It calls the driver's connect/start functions without calling disconnect
or `WiFi.reconnect()`. An associated station is always left alone, including
while DHCP is pending beyond 30s; this fallback does **not** repair stuck DHCP.
Late association/IP events cancel a reserved attempt. Failed mDNS starts are
freed and retried with bounded backoff, without accumulating services.

The read-only `GET /api/info` response preserves its original motor state and
adds a `network` object. USB `INFO` prints that same network object. It includes
current IP, Wi-Fi status and RSSI (null when not associated), a per-boot ID,
uptime, reset reason, connection/event counters and timestamps, initial setup
results, fallback attempt results, and mDNS state. `disconnectReason` is
explicitly historical and can remain nonzero while connected. No credentials
or motor-session tokens are included. USB output uses a bounded nonblocking
queue so a diagnostic record larger than the CDC buffer is transmitted in full
without delaying control; complete JSON and STOP acknowledgement were checked
on the connected board.

Run the pure host policy tests without connecting hardware:

```sh
python3 -m unittest wifi-control/rover_control/test_firmware_network.py -v
```

They cover progress/DHCP grace, capped retry backoff, late events, long quiet
connections and millisecond wrap, mDNS retry, and disconnect/reconnect lease
invalidation. An access-point outage and prolonged motor-load stability on an
independent mobile controller supply still require physical checks. A successful
USB restart or brief raised-wheel pulse does not prove those cases.
