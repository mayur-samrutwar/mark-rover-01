# Physical camera calibration

The offline utility uses saved phone JPEGs and physical measurements. It does not contact the rover, move motors, or copy simulation parameters. Its intrinsic output is always **unverified**, with no mount transform and no validated camera timing. These files alone cannot enable autonomous motion.

The former browser target, timing-marker page and floor/mount helper server have been removed. The offline fitting utilities below still operate on saved images and measurements; new inputs must include their required metadata.

Keep the rear lens, source resolution, orientation, zoom, focus and acquisition settings fixed. Collect 20–30 sharp images of a flat printed chessboard with its entire border visible. Move the target through the centre, sides, top and bottom of the image; vary its distance and tilt it in several directions. At least 15 distinct usable views are required. Copies and nearly identical target poses do not count. Keep the phone mount fixed while moving the target.

Measure a printed square's actual side in metres. `--columns` and `--rows` count **internal corners**, so a board with 10 by 7 squares has 9 by 6 internal corners. Each raw JPEG must have a same-stem JSON file containing its frame metadata: `source_width`, `source_height`, `width`, `height`, `settings` or `source_settings`, `rotation_degrees: 0`, `mirrored: false`, and `crop: "none"`. A `metadata` wrapper is also accepted. Use the original uploaded JPEG before host undistortion. A screenshot, cropped export or already corrected image is unsuitable.

From the repository root, replace the uppercase placeholders with your saved folder and measured values:

```sh
env PYTHONPATH=auto-nav/.deps:auto-nav \
  ../isaacprotocol/simulation/venv/bin/python -m rover_nav.hardware_calibration intrinsics \
  CAPTURE_FOLDER --columns 9 --rows 6 --square-size-m MEASURED_SQUARE_SIDE_M \
  --camera-id PHONE_AND_REAR_LENS --output auto-nav/config/camera-intrinsics.unverified.json
```

The fit uses four out of every five distinct usable views. The fifth is held out: its pose is estimated with the fitted intrinsics, and its reprojection residual is reported without changing those intrinsics. Inspect `intrinsics_report`, including per-image errors, exclusions and `quality_blockers`. Errors are in saved-JPEG pixels. The returned `K` is converted to full source-image pixels with the same pixel-centre convention used by ingestion; distortion coefficients remain in the normalized camera model. Changed source geometry, lens, zoom, focus, modes or pinned processing settings fail the fit rather than combining different camera modes.

Automatic illumination is handled separately: only when `exposure_mode` is explicitly `continuous`, changing `exposure_time` and `exposure_compensation` are omitted from the profile's pinned settings. Only when `white_balance_mode` is explicitly `continuous`, changing `color_temperature` is omitted. The modes themselves remain pinned; manual or unknown modes retain those values. Reported frame rate and processing settings remain pinned. Focus mode and any reported focus distance stay strict even with continuous autofocus, because focusing can change camera intrinsics. This normalization does not certify autofocus, image stabilization, or unreported processing changes; those still require physical checks on the actual phone.

Measure the lens optical centre relative to the rover's base origin at floor level: forward offset, left offset and height in metres. Measure the optical orientation as yaw, downward pitch and roll. Base axes are forward/left/up; image optical axes are right/down/forward. Positive yaw points left, positive pitch points down, and roll is right-handed about the initial forward axis before pitch and yaw. The helper uses `Rz(yaw) Ry(pitch) Rx(roll) Roptical`. Every value, including any zero, must be supplied explicitly:

```sh
env PYTHONPATH=auto-nav/.deps:auto-nav \
  ../isaacprotocol/simulation/venv/bin/python -m rover_nav.hardware_calibration mount \
  --height-m HEIGHT_M --forward-m FORWARD_M --left-m LEFT_M \
  --pitch-down-deg PITCH_DOWN_DEG --yaw-deg YAW_DEG --roll-deg ROLL_DEG \
  --output auto-nav/config/camera-mount.unverified.json
```

The helper writes only `T_base_camera`. Copy it into the intrinsic profile, retaining `verified: false`. Check the mounted camera against held-out physical target images and known floor points across the actual field of view, including near the rover. A good reprojection residual does not by itself establish accurate floor distance, a rigid mount, or usable obstacle perception.

Camera verification, a measured sensor-to-browser latency bound, and physical timing validation remain separate steps. Wheel speed/PWM tables, footprint, steering, coast stopping distance and manual-stop tests are also required by the hardware preflight. Never mark these measurements complete based on the simulator or the utility's synthetic tests. Existing output files are preserved unless `--force` is explicitly supplied.

## Floor and mount geometry

The browser helper has been removed. `rover_nav/hardware_floor_calibration.py` retains the offline geometry solver and its tests. Existing floor evidence and reports remain under `artifacts/hardware-calibration/floor/`. A new capture/annotation adapter is needed to collect additional evidence for this solver. No runtime profile is updated automatically, and stored candidate results do not establish physical validation.
