# Autonomous navigation

**Current v6 RGB-only clutter seeds 7, 19 and 42 passed full three-room acceptance** in IsaacProtocol's MuJoCo runtime. Default seed 7 reached terminal completion with **37.940 m** traveled in **1127.0 simulated seconds**, **zero obstacle contacts**, **100% observed reachable-floor coverage**, and all **300 requested RGB captures** saved. The [independent acceptance audit](artifacts/validation-v6/acceptance-summary.json) passed all 25 checks for each seed against the [frozen nine-source manifest](artifacts/validation-v6/source-sha256.json). The [strict RGB closed-door negative also passed](artifacts/rgb-closed-door-final-v6/acceptance-summary.json). All three runs match the frozen implementation. Physical-phone/ESP32 autonomy remains a later integration milestone.

The [seed-19 v6 report](artifacts/rgb-acceptance-seed-19-v6/report.json) records **35.519 m** in **1167.2 s**, zero contacts, **98.8880% reachable-floor coverage**, **324/324 captures**, 27 surveys and 43 recent-RGB corroboration checks. Its position RMSE/max/final error was **0.14434/0.22276/0.20718 m**, larger than seed 7; final yaw error was **−0.3753°**. All three accepted runs use the same frozen sources.

The [seed-42 v6 report](artifacts/rgb-acceptance-seed-42-v6/report.json) records **34.605 m** in **1098.9 s**, zero contacts, **98.2074% reachable-floor coverage**, **300/300 captures**, 25 surveys and 27 recent-RGB corroboration checks. Position RMSE/max/final error was **0.30655/0.55583/0.29152 m** and final yaw error **−3.1764°**. Global position drift reached **0.556 m**: the tracker uses local RGB registrations without global bundle adjustment. These simulation passes establish the tested calibrated flat-floor behavior, not hardware autonomy or navigation across general surfaces.

The [complete validation summary](artifacts/validation-v6/validation-summary.json) links the three mission audits, camera-loss and braking trials, closed-door evidence, evaluator checks and frozen source files.

## Local services

The standalone navigation, playback, trial-viewing, calibration and second phone-camera servers have been removed. Navigation algorithms, offline calibration utilities, CLI simulation and saved experiment evidence remain available. The playback recorder and disconnected bench/timing tools have also been removed; existing recordings are retained as data.

Use the [manual controller](http://rover.localhost:1355/) for driving and phone captures and [reconstruction](http://rover.localhost:1355/reconstruction/) for processing and viewing scenes. Pair the phone through the HTTPS camera link shown on the manual controller.

## Working simulation

Both modes use the existing 53-part Mark Rover, all four original wheel velocity drives and collision geometry. A generated L-shaped apartment has three furnished rooms, offset open doorways, table/chair legs, cabinets, sofas and seeded random boxes. The robot moves through wheel actuators and contact physics. For RGB navigation the whole phone cradle assembly is physically pitched 25° downward; no camera is detached from the phone or moved independently of its mount.

| Mode | Inputs used to navigate | Validation status |
| --- | --- | --- |
| `rgb` | RGB pixels, image timestamps and fixed camera/rover calibration. Motion and floor boundaries are estimated from images. No runtime depth, true pose, IMU or encoder measurements enter perception. | Current v6 seeds 7/19/42 passed: 3/3 rooms, zero contacts, at least 98.2074% reachable-floor coverage and all requested captures. Strict closed-door negative passed. |
| `ideal` | Exact simulated pose and perspective ray-traced depth. Camera images are acquired separately. | Three existing clutter seeds reached all rooms without obstacle contact. |

Both modes feed occupancy mapping, conservative footprint inflation, frontier selection, A*, path following, capture sweeps and stop gates. RGB mapping integrates disjoint confirmed floor segments while preserving their gaps, unseen near-floor intervals and occluded space as unknown. Only the rover's existing support footprint is initialized as a prior. Room names, obstacle coordinates, true trajectory and contact measurements belong to the physics engine, independent evaluator and observer dashboard; the RGB navigator receives none of them.

```sh
# Uses the existing sibling isaacprotocol/simulation/venv runtime.
../isaacprotocol/simulation/venv/bin/python auto-nav/setup_vision.py
# Later, verify the installed local package without downloading:
../isaacprotocol/simulation/venv/bin/python auto-nav/setup_vision.py --check
auto-nav/run.sh simulate --sensing rgb --seed 7 --capture --output auto-nav/artifacts/rgb-seed-7
auto-nav/run.sh simulate --sensing ideal --seed 7 --output auto-nav/artifacts/ideal-seed-7
auto-nav/run.sh test -q
```

Run one full rendered physics mission at a time on this Mac.

Recheck the saved evidence without launching physics:

```sh
PYTHONPATH=auto-nav/.deps:auto-nav ../isaacprotocol/simulation/venv/bin/python \
  auto-nav/artifacts/validation-v6/verify_rgb_acceptance.py \
  auto-nav/artifacts/rgb-acceptance-seed-7-v6 \
  auto-nav/artifacts/rgb-acceptance-seed-19-v6 \
  auto-nav/artifacts/rgb-acceptance-seed-42-v6 \
  --output auto-nav/artifacts/validation-v6/rechecked-seeds-7-19-42.json
```

The verifier checks strict physical room entries and their first times, terminal completion, zero contacts, coverage consistency, RGB metadata/checksums and all nine frozen source hashes. The v6 [unit suite passed 285 tests](artifacts/pytest-final-v6.txt), and [four physical fault profiles passed](artifacts/validation-v6/physical-faults-summary.json) with the same source manifest. These are separate from the full mission result.

The strict v6 closed-door trial completed accessible exploration in **691.6 s** over **21.7704 m**, with **zero contacts**, **99.7138% accessible-floor coverage** and **192 saved RGB captures**. Living room and study were entered; the closed bedroom remained unvisited. Navigation ended `complete` without failure while all-room mission success correctly stayed false. Its [11 negative-test criteria](artifacts/rgb-closed-door-final-v6/acceptance-summary.json) and [independent negative verification](artifacts/rgb-closed-door-final-v6/independent-negative-verification.json) passed with unchanged frozen sources.

The historical ideal-sensing acceptance runs (seeds 7, 19 and 42) entered all three rooms, reached terminal exploration completion and recorded zero obstacle-contact events. Seed 7 saved 240 stopped RGB-D frames with intrinsics, true optical poses and checksums. A blocked-bedroom-door baseline trial left the bedroom unvisited and failed all-room acceptance. **Those results are not RGB-only results.** See [SIMULATION.md](SIMULATION.md) for the baseline evidence and [RGB_NAVIGATION.md](RGB_NAVIGATION.md) for the image-only architecture, calibration, data boundary and unresolved validation work.

The current [RGB seed-7 v6 report](artifacts/rgb-acceptance-seed-7-v6/report.json) records position RMSE/max/final error **0.05136/0.09107/0.01974 m**, final yaw error **−0.3363°**, 25 surveys and 20 stopping checks using bounded recent RGB corroboration. Localization errors are evaluator measurements and never correct the controller.

The historical [RGB seed-19 v4 pass](artifacts/rgb-acceptance-seed-19-v4/report.json) traveled 31.804 m, observed 96.853% reachable floor and saved 276 captures in 1003.7 simulated seconds. Its position RMSE/max/final error was 0.07765/0.16471/0.06581 m. It remains a pass for its recorded v4 hashes, alongside the current v6 seed-7, seed-19 and seed-42 results.

Earlier RGB trials remain useful failure evidence: [rgb-trial7k-viewpoints](artifacts/rgb-trial7k-viewpoints/report.json) reached all rooms over 20.291 m without contact but failed with tracking reset, 87.29% coverage and maximum position error 1.757 m. [Seed-7 v1](artifacts/rgb-acceptance-seed-7-v1/report.json) improved coverage to 96.97% and RMSE/max position error to 0.133/0.226 m but stalled with `recovery_budget_exhausted`. Those two reports remain failed acceptance results.

The current image pipeline uses separate masks for strong fresh corners and broader floor-patch support, validated persistent tracks, 9×9 optical-flow windows with two pyramid levels, visually estimated camera attitude and ranked keyframe revisits. RGB surveys target 0.30 rad/s and mapping uses a 5 cm grid. Translation requires historical clearance over the full stopping path and image evidence for each leading stopping direction. A bounded recent RGB observation can confirm the same probe point only when the current adjacent bearings independently show farther floor. All three accepted clutter seeds exercised these v6 changes.

RGB navigation assumes a rigid calibrated camera over a flat, textured floor whose appearance can be learned from the initial image. It estimates metric scale from the known mount height. This is a restricted floor-perception model, not general monocular depth or a guarantee for floor-textured objects, glass, overhangs, ramps or drop-offs. Neither sensing mode validates physical autonomous driving.

## Hardware roadmap

The [real-rover adapter modules](HARDWARE.md) remain available for offline development. The separate hardware console and phone server have been removed. Physical camera, motor, timing and braking calibration remain pending.

The remaining sections retain the proposed S24 Ultra/ESP32 integration plan. ARCore, IMU and optional phone depth in that plan are future hardware sensing choices; they are not inputs to the current RGB-only simulation mode.

## Objective and module boundary

Explore reachable indoor floor space while selecting positions from which to capture overlapping observations. At each capture station, stop translation, acquire a 360-degree image sequence, and continue exploration. Maintain continuous perception during travel and rotation.

Navigation owns localization, a geometric navigation map, obstacle avoidance, station selection, motion control, and capture scheduling. Reconstruction consumes the recorded observations asynchronously. Navigation must run with the reconstruction server absent; it must not depend on a trained Gaussian scene for movement decisions.

## Research basis

[ActiveSplat](https://arxiv.org/html/2410.21955v2) combines a Gaussian map with a Voronoi graph, separates position selection from viewing-direction selection, and prioritizes local exploration before changing regions. It starts with a physical 360-degree survey; the panoramas used for candidate scoring are rendered from its map. We retain the separation of movement and observation decisions, but propose geometric coverage as the navigation objective. The [official implementation](https://github.com/Li-Yuetao/ActiveSplat) requires Ubuntu/CUDA and is a reference rather than a deployment target for this Mac.

## Proposed compute allocation

| Component | Responsibility |
| --- | --- |
| Android application | Camera and IMU acquisition; ARCore pose and supported depth; timestamped capture; local obstacle and tracking checks; motion-command validation. |
| Laptop navigation server | Navigation-map integration, frontier extraction, path planning, capture-station selection, coverage accounting, mission state, and logging. |
| ESP32 | Motor PWM, direction control, command sequencing, and the existing independent 350ms command watchdog. |
| Reconstruction server, later | Read capture records and generate a 3D scene. It does not own motor commands. |

The inspected laptop is an Apple M3 Pro with 18 GB memory. A lightweight map and graph/grid planner are the initial implementation choice; throughput still needs measurement. Running the complete planner on Android remains possible, but moves debugging, storage, and thermal load onto the capture device. Streaming raw camera/IMU data to the laptop and building a new visual-inertial tracker is a higher-complexity alternative.

### Android capability gate

The phone is a Samsung Galaxy S24 Ultra. Google's [supported-device list](https://developers.google.com/ar/devices/) lists this model with Depth API support. Check ARCore installation, selected-camera support, and actual pose/depth delivery at runtime; certification does not establish obstacle-detection performance on this rover.

The supplied [INVENTO motor-kit listing](https://www.amazon.in/dp/B0845S7ZFJ) describes four DC gearmotors and does not specify encoders. The existing wiring exposes two motor-power terminals per motor and no encoder-feedback connections. Treat the current rover as having no wheel-encoder feedback. Motor internals are not independently verified. Use observed camera/IMU motion for the initial closed-loop control experiments. Measure the assembled footprint rather than adopting inconsistent chassis dimensions from the listing.

ARCore combines image features with IMU measurements to estimate camera motion. Its [camera API](https://developers.google.com/ar/reference/java/com/google/ar/core/Camera) provides camera pose, image intrinsics, and tracking state. Pose data are usable only while tracking is valid. The proposed starting point is a native Android application, with pose tracking on the phone and planning on the laptop.

On supported phones, [Raw Depth](https://developers.google.com/ar/develop/java/depth/raw-depth) supplies sparse depth and a confidence image without requiring a dedicated depth sensor. Missing or low-confidence measurements remain unknown space. Check depth timestamps: a newly delivered frame may contain reprojected older depth rather than new observations. Depth availability is not a guarantee of collision detection.

If metric depth is unavailable, a plain camera stream or sparse feature map is insufficient evidence for general obstacle clearance. Evaluate another sensing route before enabling autonomous translation. [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) is a possible research alternative for visual-inertial tracking, but introduces calibration and synchronization work and does not by itself provide a dense obstacle map.

ARCore can adjust its world coordinates as tracking improves. Its [pose documentation](https://developers.google.com/ar/reference/java/com/google/ar/core/Pose) requires persistent locations to be anchored or expressed relative to appropriate anchors. The navigation map must account for frame revisions, pose jumps, and relocalization. A single uncorrected global trajectory is not sufficient evidence for apartment-scale consistency.

## Mission sequence

```text
PREFLIGHT → INITIAL_SURVEY → UPDATE_MAP → SELECT_STATION → FOLLOW_PATH
                              ↑                              ↓
                       RECORD_AND_UPLOAD ← CAPTURE_SWEEP ← STOP_AND_SETTLE

Any motion state → PAUSED on invalid tracking, stale observations, blocked clearance,
                   expired commands, or operator stop.
```

1. **Preflight.** Verify pairing, tracking, depth support and freshness, robot dimensions, camera-to-base calibration, battery state, and command ownership. Start remains disabled until the required checks pass.
2. **Initial survey.** Observe the surroundings and establish the initial map. A sweep is not proof of free space. If depth needs initialization movement, request a supervised movement in an already verified clear area rather than moving blindly.
3. **Select a station.** Choose a reachable viewpoint that reveals unknown space or improves capture coverage.
4. **Follow the path.** Use fresh pose feedback and continuously updated obstacle observations. Turn toward a direction before translating into it; do not assume the previous panorama still describes a moving environment.
5. **Stop and capture.** Rotate through measured headings, stop rotation and allow the camera to settle at each exposure, then validate the capture. Check the swept footprint during rotation.
6. **Record and continue.** Save observations locally, queue uploads, update coverage, and choose the next station. Reconstruction processing does not block this loop. Storage exhaustion or an unbounded upload backlog must pause acquisition rather than discard required data silently.

These are proposed behaviors, not implemented guarantees.

## Navigation map and exploration policy

Maintain a height-aware obstacle representation and a conservative 2D planning grid. Classify cells as free, occupied, or unknown. Obstacle evidence must cover the robot's body and mounted phone, while traversability also requires floor support. Inflating obstacles by the footprint and an uncertainty allowance must account for corner sweep when turning.

Use [frontier-based exploration](https://biorobotics.ri.cmu.edu/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf) as the first baseline: frontiers separate observed free space from unknown space. Cluster candidate frontiers and choose goal positions within verified free space, not inside unobserved cells. Use A* for paths on the inflated grid. Do not clear unknown regions merely because no obstacle points were returned.

Rank candidate stations using normalized estimates of newly observable area and missing viewing coverage, penalized by path length, low clearance, poor tracking conditions, and repeated failed visits. Weights are experimental parameters. Initially prefer nearby goals in the current region; add room/doorway topology if repeated inter-room travel becomes a measured problem.

Maintain capture coverage separately from traversed floor area. Passing through a cell does not establish that nearby surfaces have been photographed from useful viewpoints. A later reconstruction service may provide optional recapture suggestions, but is not required for this geometric baseline.

Terminate when no reachable frontier or feasible coverage-improving station remains, subject to mission limits. Report reachable coverage and blocked, unknown, or inaccessible regions. Closed doors, occluded surfaces, and unseen rooms preclude a claim of complete apartment coverage.

## Capture policy

A 10–15-image sweep is an initial target, not a fixed guarantee of overlap. Measure camera field of view from calibrated intrinsics. With 15 evenly spaced images, the heading increment is 24 degrees. Under an approximate angular-overlap model, a 60-degree horizontal field of view then gives 60% overlap; 12 images give 30-degree increments and 50% overlap. Check actual image correspondences and scene visibility during evaluation.

Use measured, unwrapped camera/base yaw to complete a sweep. PWM duration is not an angle measurement. Correct for the fixed camera-to-base transform and validate the existing physical left/right channel mapping. Skid steering, motor deadband, unequal wheel speeds, and surface changes require feedback and calibration.

Capture overlapping views from different positions as well as each rotation sequence. [COLMAP's capture guidance](https://colmap.github.io/tutorial.html) explicitly recommends viewpoint translation and high overlap. A rotation sequence about an approximately fixed optical center offers weak triangulation geometry. Record selected sharp frames between stations to maintain cross-station correspondences. Fixed camera height and pitch also limit vertical and occluded-surface coverage.

Keep tracking active during acquisition. High-resolution still capture must use a camera configuration compatible with tracking; [ARCore shared camera access](https://developers.google.com/ar/develop/java/camera-sharing) is relevant when integrating Camera2. Do not switch between unrelated camera applications and assume pose continuity.

## Control and stop behavior

Use one command authority. The proposed Android gateway owns the ESP32 control session; the laptop sends short-lived motion targets. The gateway checks tracking, sensor age, local clearance, and command age before renewing motor commands. Manual takeover stops autonomy and transfers ownership explicitly.

Keep the existing ESP32 watchdog. An additional perception watchdog is necessary: continuing to send commands while phone observations are stale must not count as healthy operation. Local stop checks should run independently of global planning and photo uploads. [Nav2's Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/) provides a useful architectural reference for this separation; adopting the full ROS stack is not a prerequisite.

Neither commanded PWM nor an acknowledgement measures distance traveled. Calibrate motion against observed pose, measure stopping distance and network latency, and choose speed limits from those measurements. Encoders would improve feedback if available; valid camera/IMU tracking can be evaluated first.

A single forward-looking phone has blind regions. Cables, glass, thin legs, near-field obstacles, drop-offs, and objects entering from the side require explicit tests. The first operating domain is a supervised, static, flat-floor room with known boundaries. Broader apartment operation follows demonstrated sensing coverage, not a software-only assumption.

## Data contract with reconstruction

The navigation-side capture record should contain:

- Session, station, and frame identifiers; capture timestamps from a documented clock domain.
- Original image and checksum; resolution, lens identifier, and exposure metadata.
- Camera intrinsics, distortion model, and fixed camera-to-base calibration.
- Estimated camera pose, reference-frame identifier, tracking state, and pose revision history.
- Optional depth, confidence, and their actual observation timestamps.
- Sweep heading and image-quality measurements.

Pose estimates are inputs that reconstruction may later refine. Preserve raw observations and revisions. Use resumable uploads with acknowledgements and local retention until successful receipt. Keep control/telemetry traffic separate from bulk image transfers. The reconstruction module owns scene fitting and export; navigation owns the capture queue and movement decisions.

## First hardware implementation milestones

1. **Phone feasibility logger.** Record synchronized RGB, pose, tracking state, depth/confidence where available, and timestamps. No autonomous motion.
2. **Manual mapping trial.** Drive with the existing controller while logging. Measure drift on a return path, observation age, depth coverage at robot height, turning response, and stopping distance.
3. **Single-room navigation.** Implement map integration, a selected waypoint, local stop checks, and fault-injection tests for tracking/network loss.
4. **Exploration and capture.** Add frontier selection, capture-station coverage, measured-heading sweeps, and cross-station keyframes. Demonstrate operation with reconstruction disabled.
5. **Multi-room evaluation.** Address drift/relocalization, doorways, blocked goals, revisit control, completion reporting, and mission recovery.

The first hardware implementation target is an S24 Ultra data-logging application and a laptop receiver, followed by a manually driven mapping trial. The implemented simulation controller does not connect to physical motors, and this roadmap does not start a hardware motor test.
