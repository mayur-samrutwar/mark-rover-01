# Home Wi-Fi control

The ESP32 joins the same home Wi-Fi as your laptop and serves this controller directly. The phone supplies USB power to the ESP32 only; the existing power bank supplies the drivers. The 5V motor-driver rail must not be connected to ESP32 VBUS/5V while USB powers it. Keep the existing common ground and verified driver wiring.

1. Create `firmware/rover_wifi_control/wifi_credentials.h` from the example header and enter your network name and password. The real header is ignored by Git. The ESP32-C3 needs the network's 2.4GHz service; the laptop can use either band on the same LAN.
2. Run `python3 rover_control/build_wifi_assets.py` to embed the HTML/CSS/JS in flash.
3. Upload `firmware/rover_wifi_control/rover_wifi_control.ino` as **ESP32C3 Dev Module**, **USB CDC On Boot Enabled**, using Arduino-ESP32 3.x.
4. Open Serial Monitor at 115200 and send `INFO` to see the assigned IP address. The network password is never printed. The board also advertises `http://rover.local/` through mDNS; use the numeric IP if that name doesn't resolve.
5. For the current white controller, run `python3 rover_control/serve.py` and open `http://127.0.0.1:8765/` on the laptop. It relays commands over home Wi-Fi to the rover at `192.168.1.6`; pass `--rover http://NEW_IP` if that address changes. Click **Connect rover**, and hold WASD or the on-screen buttons. The page can also be embedded in the ESP32 by rebuilding assets and uploading the firmware again.
6. After verifying Wi-Fi, move the ESP32's USB cable from the laptop to the phone. The phone must support supplying USB power (OTG/reverse charging as appropriate). The ESP32 reboots stopped, rejoins home Wi-Fi, and serves the same page. Reload and reconnect if its IP changes.

Start with all wheels raised. W/S drive forward/reverse, A/D pivot, combined keys arc, no key stops, Space/Escape latches stop. Default PWM is 128/255. Actual wheel speed/position is not measured.

The ESP32's independent motor task checks Wi-Fi and a 350ms command lease every 5ms. HTTP requests cannot prevent that task from stopping the motors. After lease expiry, the session is disarmed; delayed commands cannot restart motion. Each connection has a random token and strictly increasing sequence. A second controller is refused while the first session is active. Stopping/resuming clears all held input and requires a fresh control session. Direction reversals coast for 40ms. This is local HTTP control on the home network, not an internet-facing service; the firmware does not expose any Wi-Fi credentials through its web API.

The original USB controller remains at `/usb.html` on the laptop development server, with its original USB firmware in `firmware/rover_keyboard_control`.

This board/router combination uses 802.11b/g mode and 8.5dBm transmit power. The default 802.11n mode repeatedly lost beacons during the Wi-Fi-only diagnostic. This compatibility setting changes radio modes, not the network password or WPA2 authentication. `INFO` includes the current RSSI and the most recent disconnect reason; the latter is historical and can remain nonzero after reconnection.

While idle, the page leaves the motor session disarmed and checks network reachability once per second with a 2-second request timeout. A newly held direction establishes a fresh session; releasing input sends STOP and disarms it again. Held input is checked again after the handshake, so releasing during connection cannot replay an old movement. Driving retains the 100ms heartbeat, 300ms request deadline, and the ESP32's independent 350ms motor lease. Idle status delays therefore do not trip the driving timeout.
