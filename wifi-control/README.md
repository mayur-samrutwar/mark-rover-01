# Mark 01 · Wi-Fi control

Double-click **Start Control.command**, leave its terminal running, and open **http://127.0.0.1:8765/** on this laptop. Click Connect rover. The ESP32 is powered by your phone and joins the same home network; the motor drivers keep their existing power bank supply.

W/S or up/down arrows: forward/reverse. A/D or left/right arrows: left/right. Combine keys for arcs. Release all keys to stop. Space/Escape: stop and lock. Resume clears old input. Drive strength starts at 50%.

The page contains only four centered arrow buttons, a drive-strength slider, and a bottom connection bar. It has no branding, diagram, heading, tagline, or helper copy. The square footer icon stops and locks; the play icon resumes after a stop. The browser translates the installed driver channel order so A/D now turn the correct way; W/S are unchanged. The diagram uses those same physical wheel coordinates. Motor rotation and speed are not measured.

The server binds only to this laptop (127.0.0.1). It relays commands to http://192.168.1.6 over Wi-Fi and never generates movement commands itself. If the rover gets a new address, run `python3 rover_control/serve.py --rover http://NEW_IP` from this folder. Only use one controller page at a time. The ESP32 stops and disarms after 350 ms without valid commands; the browser locks if it loses acknowledgements.

## Files

- `rover_control/`: page, scripts, local Wi-Fi relay, and isolated browser tests.
- `firmware/rover_wifi_control/`: working ESP32 firmware and freshly generated white-page assets for a future USB upload. The existing running firmware already supports this laptop controller; no upload is needed to use it.
- `firmware/rover_wifi_control/wifi_credentials.h`: private local network configuration; excluded by .gitignore.

The older page at the ESP32's own address stays unchanged until a firmware upload. Use the localhost address above for this corrected version.

Idle connection checks are separate from driving: the motor session is disarmed while idle and a fresh session starts for a newly held direction. Release sends STOP. Idle polling tolerates 2-second status delays; the driving heartbeat and ESP32 350ms stop timeout are unchanged.

Idle health checks retry automatically. Three consecutive failures show Reconnecting and disable direction input; a successful status check restores the connection without arming a motor session. Held inputs are cleared, and manual emergency stops always require Resume.
