# Real-rover adaptation

The hardware adapters reuse the frozen RGB perception, visual odometry, occupancy mapping and frontier planner. They do not instantiate a simulated mission or use a room layout. Hardware deployment is still in bring-up: successful simulation and offline tests do not establish physical calibration or collision-free real operation.

The user has secured the rover's phone and battery, identified the active rear lens as ultrawide, and updated its approximate height to **24.5 cm**. The earlier 22 cm estimate is superseded. The checkerboard captures have an unverified intrinsic fit; rigid mount geometry, floor-distance acceptance, timing, motor response and coast stopping remain incomplete. The calibration and autonomous ceiling is **80% PWM**, which means **204/255 on each physical wheel**, including turning. The separate manual Wi-Fi controller allows **0–100%** at the user's request. PWM percentage is not a percentage of measured linear speed.

The user reported that W drove backward, S drove forward, and A/D turned in their intended directions. Both real adapters now encode physical `(left, right)` as wire `(-left, -right)` without exchanging sides. This reverses the straight commands while preserving the reported pivot commands; physical confirmation is pending. Both left wheels reportedly struggle in reverse even at full manual PWM. Diagnose that response with the wheels securely raised before relying on motion calibration or autonomy.

## Local services

The separate hardware navigation console and its phone/setup servers have been removed. Use `wifi-control/rover_control/serve.py` for manual driving and phone capture. The hardware runtime, camera ingestion, motor transport, offline calibration and analysis modules remain for development; there is no browser launch or live autonomous hardware service in this module.

The disconnected bench-pulse and timing-target tools have also been removed. Offline camera calibration and motion-analysis utilities remain available for existing evidence.

## Calibration and staged testing

1. Keep the mount and battery rigid. Measure the full swept footprint including the phone, battery, wiring and overhangs. Confirm that the rear lens sees textured floor in its initial calibration patch.
2. Pin the actual rear lens, full source resolution, orientation, zoom and processing settings. Collect diverse stationary images of a known-size checkerboard using a capture adapter that preserves the required source metadata. These saves use the existing raw phone JPEG; they do not request motion or another exposure. Solve lens intrinsics/distortion for those exact settings and validate on held-out images and known floor distances.
3. Measure camera height/offset/attitude relative to the rover. The camera profile uses base X forward, Y left, Z up; optical X right, Y down, Z forward. A profile remains unverified until scale and reprojection checks pass. Never copy simulated camera parameters.
4. Measure image timing under load. Browser frame callbacks describe presentation, not guaranteed sensor exposure. The adapter uses a challenge and a separately measured pipeline-delay bound to derive a conservative acquisition-time lower bound. It rejects stale, repeated, reordered or changed-source images and resets map and estimator together. Unknown timing cannot authorize motion. The [official callback specification](https://wicg.github.io/video-rvfc/) explains the presentation timestamp and lack of strict callback timing guarantees.
5. After a camera-only push/return test and floor-obstacle checks, test the firmware stop/watchdog and steering at low power under supervision. Establish motor deadband, four forward/reverse wheel speed/PWM tables and effective skid-steer track. Use known physical distances to validate RGB scale; motor acknowledgements are applied PWM, not odometry.
6. Measure worst-case coast stopping, response delay, speed/yaw error and lateral slip with the final battery/load on the test surface. Stop sets zero PWM and coasts; simulated deceleration is not a hardware rating. Increase test power in stages only while the measured stopping envelope fits the clear test area. Do not jump to 80% from an uncalibrated state.
7. Enter the measured physical/motor/dynamics/timing profile and camera profile with measurement records. A future hardware integration must provide explicit operator arming and retain the configured per-wheel PWM ceiling of 204.

The real adapter replaces the simulator's braking model with measured reaction/deceleration bounds and adds image age, speed error, yaw error and a distance margin. Each command proof covers the maximum allowed image age. An independent motor authority expires stale vision/controller proofs and operator heartbeats; tracking loss, changed camera session, overspeed, reverse/lateral slip or transport errors latch a stop. STOP cancels pending arming and stale acknowledgements cannot restore authority. The installed ESP32's independent 350 ms watchdog remains the final lease. A lost STOP acknowledgement is not proof that a STOP packet arrived.

The vision model still assumes a static flat textured floor with distinguishable obstacles. It has not been validated for stairs/drop-offs, glass, cables, floor-textured obstacles, overhangs, moving people, ramps or arbitrary surfaces. Keep the first trials in a clear bounded area with the operator at the power switch.

## Offline verification

```sh
auto-nav/run.sh test -q
node --test auto-nav/tests/test_hardware_phone.mjs auto-nav/tests/test_hardware_operator.mjs
```

These tests use fake transports, images, clocks and in-process HTTP clients; they never contact a physical rover. Constructor, status, preview, reset and monitor mode cannot acquire a motor session. Calibration examples are deliberately incomplete and unverified. Physical timing, motion, scale and stopping tests must be recorded separately when the assembled rover and phone are connected.
