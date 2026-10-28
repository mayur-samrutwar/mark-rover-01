# RGB-only navigation experiment

**Current v6 RGB-only clutter seeds 7, 19 and 42 passed full acceptance** in IsaacProtocol's MuJoCo runtime. Default seed 7 physically entered all three rooms and reached terminal `complete`, with **37.940 m** traveled in **1127.0 simulated seconds**, **zero contacts**, **100% observed reachable-floor coverage**, and **300/300 RGB captures**. Position RMSE/max/final error was **0.05136/0.09107/0.01974 m**, with final yaw error **−0.3363°**. See [the report](artifacts/rgb-acceptance-seed-7-v6/report.json) and [independent 25-check acceptance audit](artifacts/validation-v6/acceptance-summary.json).

All three accepted runs match the [frozen nine-source v6 manifest](artifacts/validation-v6/source-sha256.json) and exercised disjoint floor intervals, the buffered braking gate and bounded recent-image corroboration. Default seed 7 completed 25 surveys and recorded 20 stopping checks using recent RGB evidence. The [285-test suite](artifacts/pytest-final-v6.txt) and [four physical fault profiles](artifacts/validation-v6/physical-faults-summary.json) also passed for this source. The [strict v6 RGB closed-door negative also passed](artifacts/rgb-closed-door-final-v6/acceptance-summary.json). These are three full clutter-seed acceptance results within the calibrated flat-floor domain below.

The [current seed-19 v6 report](artifacts/rgb-acceptance-seed-19-v6/report.json) records **35.519 m**, **1167.2 simulated seconds**, zero contacts, **98.8880% reachable-floor coverage**, **324/324 captures**, 27 surveys and 43 recent-RGB corroboration checks. Its position RMSE/max/final error was **0.14434/0.22276/0.20718 m**, larger than seed 7, with final yaw error **−0.3753°**. The combined [independent audit](artifacts/validation-v6/acceptance-summary.json) passed all 25 checks for each accepted seed against the same frozen source manifest.

The [current seed-42 v6 report](artifacts/rgb-acceptance-seed-42-v6/report.json) records **34.605 m** in **1098.9 simulated seconds**, zero contacts, **98.2074% reachable-floor coverage**, **300/300 captures**, 25 surveys and 27 recent-RGB corroboration checks. Position RMSE/max/final error was **0.30655/0.55583/0.29152 m**, with final yaw error **−3.1764°**. Global drift therefore reached **0.556 m**. Local RGB registrations do not provide global bundle-adjusted SLAM; these simulation successes do not establish hardware autonomy or navigation across general surfaces. The rigid calibration, known camera height and flat textured-floor priors remain required.

The historical [RGB seed-19 v4 pass](artifacts/rgb-acceptance-seed-19-v4/report.json) traveled 31.8036 m in 1003.7 simulated seconds, with zero contacts, 96.8531% reachable-floor coverage and 276/276 captures. Position RMSE/max/final error was 0.07765/0.16471/0.06581 m. Its hashes predate the current method; it remains a pass for that recorded implementation. Historical ideal-pose/depth results in [SIMULATION.md](SIMULATION.md) remain a separate baseline.

Earlier RGB failures remain part of the evidence: [rgb-trial7k-viewpoints](artifacts/rgb-trial7k-viewpoints/report.json) reached all rooms over 20.291 m without contact, then failed with tracking reset, 87.29% coverage and large drift. [Seed-7 v1](artifacts/rgb-acceptance-seed-7-v1/report.json) reached all rooms over 22.436 m without contact and 96.97% coverage but ended `stalled` with `recovery_budget_exhausted`. The later v4 pass does not change those outcomes.

## What crosses the sensor boundary

`RGBTransport` produces `RGBPacket(pixels, timestamp)`. `CameraPerception` also receives fixed camera intrinsics and a rigid camera-to-base calibration. That is the entire observation input: no rendered depth, simulator pose, body transforms, geometry IDs, room metadata, IMU, wheel encoder values or commanded-motion odometry is supplied to visual perception.

The physics engine needs the world to render images and simulate contacts. An independent evaluator reads true pose, contact forces and room geometry to score the trial and stop it on failure. The dashboard also shows that observer information. The evaluator's fixed local-to-world alignment is used only for plots, room/coverage/error measurement and display; it never corrects visual estimates or guides the controller.

The known mount height and robot dimensions are metric priors. Therefore “RGB only” means no additional **runtime measurements**, not a self-calibrating monocular system with no geometric assumptions.

## Physical camera and scene

The saved rover MJCF remains unchanged on disk. `vision_scene.py` rotates the complete phone cradle subtree down 25°, including its phone, pads, lenses and collision shapes. All 53 authored parts, original masses/inertias and four wheel drives remain. The root plus those parts plus MuJoCo's world total 55 bodies.

Static XML transforms produce the following calibration at 320×240 pixels:

```text
fx = fy = 207.8460969 pixels; cx = 159.5; cy = 119.5
camera origin in base (forward, left, up):
    (0.0077107307, 0.0200000000, 0.3001021137) metres
optical forward in base:
    (cos(25°), 0, -sin(25°))
```

`T_base_camera` maps OpenCV coordinates (+X right, +Y down, +Z forward) into base coordinates (+X forward, +Y left, +Z up). The base floor is z=0. The free root's world position/orientation is explicitly excluded from calibration. A scene-design test, separate from navigation, verifies that real bottom central camera rays reach floor approximately 0.220 m from the base without hiding self geometry. This connects visible floor to the rover's existing footprint; it does not assume an additional clear starting disk.

Generated RGB scenes place the rover at world `[1.2, 1.3, 0]` to leave clearance for its initial visible-floor calibration patch and configured planning footprint. The historical ideal baseline starts at `[1, 1, 0]`; that location is only about 0.340 m from a cabinet corner, below the RGB grid's 0.385 m clearance threshold. The RGB placement has about 0.674 m of obstacle clearance in the reference scene. This is scene setup, not localization input: the visual estimator still starts at local `[0, 0, 0]` and receives no spawn coordinates. Explicitly supplied custom worlds retain their supplied placement.

The configured 0.25 m circular footprint is a preflight placement prior, not image evidence. An offline audit of all 53 collision meshes bounds their exact planar radius for every wheel angle at 0.212794 m; a more conservative bounding-box disc is 0.214311 m. Thus the configured circle encloses all collision parts at every planar yaw, with at least 35.7 mm radial allowance. This does not cover tipping, deformation or mount changes.

The floor uses an embedded random-speckle texture with a 16 m repeat period, ordinary ambient/diffuse room lighting and a non-emissive material. There are no fiducials, labels or encoded world coordinates. The perception algorithm is not given its texture, palette or scene seed. It learns an appearance model from a central initial-image floor patch, approximately 0.3–0.7 m from the camera.

## Image processing and control

1. **Learn and check floor appearance.** Initial image chromaticity, exposure, texture and feature checks establish a bounded prior. Later frames combine color/brightness evidence, two-axis texture variation and connected regions. Metric texture-support uncertainty is removed before space is called free.
2. **Estimate metric motion.** Seed fresh corners only inside `feature_mask`, which requires strong direct two-axis texture. Use the broader connected `tracking_mask` for patch support and validating existing tracks. Optical flow uses a 9×9 window and `maxLevel=2`, with forward/backward consistency checks. Existing geometrically validated tracks may continue when too few fresh corners are available. Floor landmarks are seeded on z=0 from the current estimated camera pose; homography consensus and regularized planar-landmark PnP estimate the full camera attitude/position, using fixed mount-height and near-level priors. The accepted visual estimate then updates this exposure's floor projection, rather than assuming the chassis never rocks.
3. **Register visual revisits.** RGB descriptor keyframes store estimated floor landmarks, never true coordinates. Candidate ranking favors a nearby, similarly directed view from a previous rotation before older, farther history. Matches, reprojection error and pose-change gates must pass before a bounded correction is accepted. A failed frame never advances the accepted reference clock; old landmark seeding alone does not count as a tracking outage.
4. **Return conservative visible intervals.** The floor sensor supplies an ordered union of disjoint near/far free intervals for each original bearing, plus separate positive appearance-boundary contacts. Each confirmed segment is bounded by actual image support and retreated for projection uncertainty. Missing texture leaves a gap unknown; independently confirmed floor can resume farther along that bearing. Every segment stops before the first positive occluder, preserving its shadow. The mapper integrates the segments separately and never their convex hull. The original near/far arrays retain only the first segment for compatibility; the explicit `free_intervals` union is authoritative, including empty unions. All three accepted v6 clutter-seed missions used this representation. Gaps and space behind positive obstacles remain unmeasured.
5. **Explore through the existing navigator.** Initialize only the configured rover support footprint, then integrate image-derived evidence into a 5 cm occupancy grid. Inflate for the footprint/margin and the cell half diagonal, select reachable frontiers, plan with A*, follow using visual pose, and schedule stopped captures. RGB survey angular targets are 0.30 rad/s. Viewpoint selection prefers more room for surveys while retaining the same hard footprint clearance. Allocation bounds are fixed independently of room geometry.
6. **Stop or withhold evidence when it fails.** Invalid/stale image tracking and rejected motion fits stop commands; the actuator command-age watchdog remains separate. Uncertain floor contributes no new free interval. Visual tracking can remain valid using persistent landmarks even while new floor evidence is unavailable, so the console reports localization and floor confidence separately. Translation requires both historical clearance over the full stopping sweep and current floor corroboration in its leading directions, described below. A short visual outage can recover only by matching retained accepted image history. Expired accepted history requires resetting the visual origin and map rather than using a truth/commanded-odometry fallback.

The shared navigation observation uses the words “pose” and “ranges,” but in RGB mode those fields are **estimated from RGB and the floor assumption**. They are not privileged simulator measurements or generic monocular depth predictions.

The brightness envelope is learned from the initial calibration image, rather than a scene palette or a fixed wall color. Its normalized upper ratio is `max(1.5, 1.5 × initial_floor_p99 / initial_floor_median)`. A later surface above the current textured-floor median times that ratio, plus four 8-bit intensity levels, supplies positive appearance evidence. Exposure rescaling requires at least 100 well-exposed, color-matched seed pixels with strong two-axis texture. Smooth wall shading cannot update the floor median. Darkness remains unknown, and independently visible positive contacts survive even when the same bearing has no usable free interval. Appearance overlap can still leave a wall unknown; the fresh stopping-direction check is necessary in that case.

### Fresh stopping-direction corroboration

The RGB stopping check considers the requested and previous linear speed, straight travel, requested/previous turning, a 0.35 s reaction bound, provisional simulated braking deceleration and a 6 cm final distance buffer. Positive appearance contacts can stop any intersecting footprint sweep. The historical map must still contain the full physical stopping tube, including near-floor space outside the current image.

Historical free cells alone can become unsafe after accumulated localization error. A later seed-42 v2 trial reached every room but collided with a wall while its current floor mask correctly left that wall unknown. Its old map contained falsely free wall cells. Therefore the current controller additionally requires a real floor confirmation point in each leading stopping direction. The band begins at the physical modeled stop cap, before the additional 6 cm distance buffer, and ends at the buffered cap plus one grid-cell diagonal: another 7.071 cm on the 5 cm map. The whole buffered stopping tube must still be historically clear. This observation-confirmation band allows independently seen near floor to corroborate low-speed stopping while preserving the existing maximum probe distance. It authorizes no additional travel and clears no uncertain prefix or intervening gap.

Direct confirmation requires genuine nonzero free intervals at both adjacent original sensor bearings containing the probe's actual radius. Their disjoint unions are intersected without extending endpoints or joining gaps; invalid bearings, isolated rays and angular holes supply no sector. Farther floor requires the additional recent-image proof below. The check accounts for the camera's calibrated lateral/forward offset and every braking model. A stationary rover may rotate on its historical verified footprint without fresh forward floor; previous translation still requires braking corroboration. The saved seed-42 RGB/mask/map collision regression stops with `insufficient_current_forward_floor`.

The current v6 controller also keeps at most 64 immutable accepted RGB observations, bounded to five seconds and 0.4 m of base displacement. If the current image cannot directly confirm the probe, its original adjacent bearings must independently show farther floor in that direction, and a recent image must directly contain the **same world-space probe point** in its own calibrated adjacent-bearing interval intersection. Segment/annulus clipping enforces the same point in both views. This fallback never changes the occupancy grid, fills an interval gap or overrides the full braking-tube and current-positive-contact checks. History is cleared after tracking/clock/pose faults, an observation gap over 0.5 s or a change away from RGB sensing. The current observation is remembered only after its command decision.

This change addresses a measured v5 failure: [the default seed-7 serial trial](artifacts/rgb-acceptance-seed-7-v5-serial/report.json) stopped at the first doorway after 453.6 s with zero contacts and maximum position error 0.0585 m. It remained a failed mission. [Saved-image reconstruction](artifacts/rgb-recent-proof-seed7-v5/same-point-proof.json) found a directly observed floor point in a 3.7-second-old image, with only 0.0177 m of intervening estimated translation, that the current texture-uncertain near interval omitted. The [independent offline gate audit](artifacts/rgb-recent-proof-seed7-v5/v6-gate-validation.json) verifies this same-point proof while rejecting stale/distant history, radial gaps, current walls and non-overlapping probe points. The accepted v6 seed-7, seed-19 and seed-42 runs then exercised the method through all three rooms.

An actual seed-42 v4 doorway exposure at 408.5 s provides centre-ray intervals `[0.225, 0.325]` and `[0.55, 2.5]` m, separated by a texture-uncertain gap. With an independently clear historical-map fixture, the updated gate selects 0.0625 m/s and rejects 0.125–0.25 m/s. Mapping the centre ray retains the gap at 0.45 m as unknown. This regression checks the real image representation and stopping logic; it does not establish actual route feasibility or full-mission acceptance of the updated controller.

The [scripted v5 doorway diagnostic](artifacts/rgb-doorway-progress-final-v5/report.json), with every historical map cell deliberately marked free, advanced only 9.3 mm and failed its progress target while retaining tracking and zero contacts. That fault removes real frontiers, so its later final survey does not establish normal exploration behavior. The failed diagnostic remains recorded separately from full missions using their ordinary observed maps.

## Run and inspect

Use explicit mode names and separate output directories:

```sh
# One-time project-local OpenCV setup; leaves the sibling runtime unchanged.
../isaacprotocol/simulation/venv/bin/python auto-nav/setup_vision.py
# Offline verification after setup:
../isaacprotocol/simulation/venv/bin/python auto-nav/setup_vision.py --check
auto-nav/run.sh simulate --sensing rgb --seed 7 --capture --output auto-nav/artifacts/rgb-seed-7
auto-nav/run.sh test -q
# Recheck saved evidence against the frozen v6 source bundle; no physics:
PYTHONPATH=auto-nav/.deps:auto-nav ../isaacprotocol/simulation/venv/bin/python \
  auto-nav/artifacts/validation-v6/verify_rgb_acceptance.py \
  auto-nav/artifacts/rgb-acceptance-seed-7-v6 \
  auto-nav/artifacts/rgb-acceptance-seed-19-v6 \
  auto-nav/artifacts/rgb-acceptance-seed-42-v6 \
  --output auto-nav/artifacts/validation-v6/rechecked-seeds-7-19-42.json
# Independent geometry audit: static model only, no rendering or motion.
PYTHONPATH=auto-nav/.deps:auto-nav ../isaacprotocol/simulation/venv/bin/python auto-nav/tests/test_static_geometry_audit.py
```

The launcher uses the sibling IsaacProtocol Python runtime and prepends `auto-nav/.deps` for the locally installed OpenCV package. Set `NAV_PYTHON` and `ISAAC_PROTOCOL_ROOT` when those locations differ. RGB rendering on macOS needs the desktop graphics connection. This runs IsaacProtocol's MuJoCo backend, not NVIDIA Isaac Sim.

Run full rendered physics trials serially on this Mac. A replay dashboard can remain open while one mission runs; do not start another live mission from it. OS `SIGSTOP` cannot safely suspend in-flight renderer futures, whose timeouts use wall-clock time. Use application pause/braking for interactive runs and let final CLI acceptance trials finish one at a time.

The console labels the sensing mode and displays actual mounted-camera RGB, visual tracking/inlier diagnostics and local estimated pose. Its 3D world, measured trajectory, room labels and “evaluator pose” are observer data. A successful-looking baseline replay is not evidence that the RGB mission passed.

Each new/reset console mission receives a unique `artifacts/live-{sensing}-seed-{seed}-{uuid8}/` directory. Captures and the final report stay together within that run; another attempt with the same seed cannot overwrite them. CLI output remains controlled by `--output`. Reports retain source hashes and planner configuration for comparison with the method that actually produced the evidence.

RGB output includes `report.json`, `trajectory.json`, `localization_errors.json`, `occupancy.npy`, `last_camera.png`, `last_floor_mask.png`, `camera_calibration.json` and `final_state.json`. Requested captures contain RGB PNGs and metadata with fixed calibration, estimated base pose, tracking and image checksums. They contain no depth buffer or true optical transform. `report.localization_evaluation` is explicitly evaluator-only.

## Acceptance and unresolved limits

A full mission must independently demonstrate all three physical room entries, terminal exploration completion, no obstacle contact, at least 95% observed reachable-floor coverage, no failure and complete requested captures. A timer, an apparently filled map or a controller `complete` state alone does not satisfy acceptance. Multiple clutter seeds, closed-door negatives and image-loss trials are required alongside the data-boundary and perception tests.

The [portable verifier](artifacts/validation-v6/verify_rgb_acceptance.py) checks the report against strict trajectory entry/first-entry times, the saved scene's three non-collinear rooms, coverage counts, capture events and image timestamps, RGB PNG dimensions/checksums, fixed calibration, and all nine source hashes. Its adjacent `source/` and [manifest](artifacts/validation-v6/source-sha256.json) make the bundle reusable outside the checkout. Full all-room acceptance covers all three planned v6 clutter seeds: 7, 19 and 42.

The strict v6 closed-door negative passed all [11 criteria](artifacts/rgb-closed-door-final-v6/acceptance-summary.json): navigation completed the accessible living room and study without failure, leaving the blocked bedroom unvisited and all-room success false. It traveled **21.7704 m** in **691.6 simulated seconds**, with **zero contacts**, **99.7138% accessible reachable-floor coverage** and **192 RGB captures**. The [independent negative audit](artifacts/rgb-closed-door-final-v6/independent-negative-verification.json) found exactly the four expected all-room/bedroom failures in the strict positive verifier; its remaining **21 source, capture, trajectory, coverage and completion checks passed**.

The current model assumes a rigid camera over a stationary, approximately level floor, adequate light and floor texture, and obstacles distinguishable from the learned floor. A box printed with the same floor texture can defeat appearance-based clearance. Glass, cables, drop-offs, overhangs, changing illumination, camera motion out of the floor plane and dynamic obstacles need broader sensing or geometric inference and independent validation. RGB keyframe registration limits some drift but is not a globally optimized 3D SLAM system; large corrections are not a substitute for a consistent map.

The test suite includes synthetic motion/photometry cases and actual rendered RGB fixtures, wall/oblique-wall regressions, loss/recovery tests and truth-poisoning checks. Tests of camera placement may use simulator rays as an offline oracle; those rays never enter the RGB mission's perception path. Report the outcomes of full physical simulation trials separately from these unit/regression checks, including failed runs.

The reproducible [static geometry audit](tests/test_static_geometry_audit.py) writes [its local JSON evidence](artifacts/static-geometry-audit.json). It also checks the actual 1.08 m collision-post door openings at five grid alignments. With 10 cm of unknown space along each post, the 5 cm RGB grid retains a crossing path after the 0.385355 m center-clearance threshold. These three static regression tests pass without physics steps or GL. They establish footprint/door feasibility only; they do not validate RGB wall detection, drift or mission completion.
