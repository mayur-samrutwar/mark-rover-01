# Autonomous navigation

Status: architecture proposal. Autonomous motion has not been implemented or tested.

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

## First implementation milestones

1. **Phone feasibility logger.** Record synchronized RGB, pose, tracking state, depth/confidence where available, and timestamps. No autonomous motion.
2. **Manual mapping trial.** Drive with the existing controller while logging. Measure drift on a return path, observation age, depth coverage at robot height, turning response, and stopping distance.
3. **Single-room navigation.** Implement map integration, a selected waypoint, local stop checks, and fault-injection tests for tracking/network loss.
4. **Exploration and capture.** Add frontier selection, capture-station coverage, measured-heading sweeps, and cross-station keyframes. Demonstrate operation with reconstruction disabled.
5. **Multi-room evaluation.** Address drift/relocalization, doorways, blocked goals, revisit control, completion reporting, and mission recovery.

The first implementation target is an S24 Ultra data-logging application and a laptop receiver, followed by a manually driven mapping trial. No navigation controller, reconstruction process, or motor test is started by this proposal.
