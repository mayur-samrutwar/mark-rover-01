# IsaacProtocol autonomous exploration

## Current validation status

**Current v6 RGB clutter seeds 7, 19 and 42 passed full mission acceptance.** Default seed 7 recorded all three physical room entries, terminal `complete`, **37.940 m** traveled in **1127.0 simulated seconds**, **zero contacts**, **100% observed reachable-floor coverage**, and **300/300 requested RGB captures**. The [independent saved-evidence auditor](artifacts/validation-v6/acceptance-summary.json) passed all 25 checks for each seed against the [frozen nine-source manifest](artifacts/validation-v6/source-sha256.json). The [strict v6 RGB closed-door negative](artifacts/rgb-closed-door-final-v6/acceptance-summary.json) also passed. All three full missions used the same frozen implementation.

There are two explicitly separate sensing modes. `ideal` is the historical exact-pose/depth baseline. `rgb` estimates motion and visible floor boundaries from the phone's rendered RGB images and fixed calibration, without runtime pose, depth, IMU or encoder measurements. The accepted v6 default run includes disjoint floor intervals and bounded recent-image stopping corroboration. Unit tests, short motion trials, fault-injection tests and room visitation alone do not establish full acceptance.

The former browser dashboard has been removed. Saved simulation evidence and command-line execution remain available.

## Current v6 RGB acceptance: three clutter seeds

The [seed-7 v6 report](artifacts/rgb-acceptance-seed-7-v6/report.json) records position RMSE/max/final error **0.05136/0.09107/0.01974 m** and final yaw error **−0.3363°**. It completed 25 surveys and recorded 20 stopping checks using bounded recent RGB corroboration. The evaluator first recorded study entry at 381.3 s and bedroom entry at 666.5 s. True poses and room geometry score the run; they are withheld from image perception and navigation.

[Seed 7](artifacts/rgb-acceptance-seed-7-v6/report.json), [seed 19](artifacts/rgb-acceptance-seed-19-v6/report.json) and [seed 42](artifacts/rgb-acceptance-seed-42-v6/report.json) reached terminal `complete`, physically entered all three rooms and passed all 25 independent checks against the same nine frozen source hashes. Localization error varies substantially between accepted runs.

| Current v6 measurement | Seed 7, default | Seed 19 | Seed 42 |
| --- | --- | --- | --- |
| Full acceptance | **PASS** | **PASS** | **PASS** |
| Distance | 37.940 m | 35.519 m | 34.605 m |
| Simulated duration | 1127.0 s | 1167.2 s | 1098.9 s |
| Obstacle contacts | 0 | 0 | 0 |
| Observed reachable floor | 100% | 98.8880% | 98.2074% |
| RGB captures / surveys | 300 / 25 | 324 / 27 | 300 / 25 |
| Recent-RGB corroboration checks | 20 | 43 | 27 |
| Position RMSE / maximum / final | 0.05136 / 0.09107 / 0.01974 m | 0.14434 / 0.22276 / 0.20718 m | 0.30655 / 0.55583 / 0.29152 m |
| Final yaw error | −0.3363° | −0.3753° | −3.1764° |

Global position drift reached **0.556 m in seed 42**, whose final consecutive failed-goal count was zero. The tracker uses local RGB registrations without global bundle adjustment. Successful exploration and local collision avoidance in these simulations do not establish globally consistent SLAM, hardware autonomy or navigation across general surfaces. Fixed intrinsics, rigid camera calibration, known mount height and a static flat textured floor remain operating priors.

The frozen v6 [unit suite passed 285 tests](artifacts/pytest-final-v6.txt). All [four physical fault profiles](artifacts/validation-v6/physical-faults-summary.json) passed with the same nine source hashes: normal positive-contact forward stopping, forward stopping with positive hits suppressed, camera dropout stopping and short image-loss recovery. These checks complement all three full RGB missions and the strict closed-door negative below.

## Current v6 closed-door negative

The [strict RGB closed-door trial](artifacts/rgb-closed-door-final-v6/acceptance-summary.json) passed all **11 negative-test criteria** with the same nine frozen source hashes. It completed accessible exploration in **691.6 simulated seconds**, traveled **21.7704 m**, recorded **zero contacts**, observed **99.7138% of accessible reachable floor** and saved **192 RGB captures**. The living room and study were physically entered; the bedroom behind the closed door remained unvisited. Navigation reached `complete` with `failure: null`, while all-room `success` and `all_rooms_visited` correctly remained false.

The [independent negative verification](artifacts/rgb-closed-door-final-v6/independent-negative-verification.json) reused the strict positive auditor and found exactly the four expected failed criteria: reported all-room success, all three physical rooms entered, bedroom trajectory entry and bedroom first-entry time. Its other **21 checks passed**, including source hashes, RGB captures, trajectory, coverage and terminal completion. This establishes the closed-door negative separately from the successful open-door default mission.

## Historical v4 RGB acceptance pass

The [seed-19 v4 report](artifacts/rgb-acceptance-seed-19-v4/report.json) independently records `success: true`, terminal `complete`, all three rooms physically entered and no failure. It traveled **31.8036 m** in **1003.7 simulated seconds**, recorded **zero obstacle contacts**, observed **96.8531%** of reachable floor and saved all **276 requested RGB captures**. Position RMSE/max/final error was **0.07765/0.16471/0.06581 m**. Final yaw error was −2.498°.

This is an RGB-only acceptance result, distinct from the ideal-sensing baselines below. Its source hashes predate the current v6 interval-union and recent-image corroboration changes. The three v6 results above validate their shared frozen implementation. The strict v6 closed-door negative is recorded above.

## Earlier RGB milestones — failed acceptance

The earlier [rgb-trial7k-viewpoints](artifacts/rgb-trial7k-viewpoints/report.json) and newer [rgb-acceptance-seed-7-v1](artifacts/rgb-acceptance-seed-7-v1/report.json) both use RGB-only perception and preserve source hashes. Their independent results are:

| Measurement | Earlier RGB trial | Seed-7 v1 |
| --- | --- | --- |
| Physically visited rooms | 3/3 | 3/3 |
| Distance | 20.291 m | 22.436 m |
| Obstacle contacts | 0 | 0 |
| Simulated duration | 561.9 s | 1052.7 s |
| RGB captures | 288, complete | 288, complete |
| Observed reachable-floor coverage | 87.29% | 96.97% |
| Position RMSE | 1.029 m | 0.133 m |
| Maximum / final position error | 1.757 m / 1.015 m | 0.226 m / 0.162 m |
| Final yaw error | −20.02° | −3.88° |
| Terminal state / reason | `visual_tracking_reset_required` | `stalled` / `recovery_budget_exhausted` |
| Overall acceptance | **FAIL** | **FAIL** |

Neither of these two earlier controller runs reached successful exploration completion. Seed-7 v1 satisfies the room/contact/coverage portions of acceptance but exhausts recovery rather than finishing. The later seed-19 v4 pass is recorded separately above. Source hashes in each report identify the exact implementation used by that run; later fixes do not alter an earlier report's outcome.

## Verified ideal-sensing baseline

The original saved 53-part rover, four wheel velocity actuators and mounted phone camera run in IsaacProtocol's actual MuJoCo runtime. In the baseline, exact simulator pose and forward depth rays drive navigation. The planner receives no room goals, floorplan, obstacle list or precomputed route. Its initial map extent is an allocation bound, not a measured floorplan. This still relies on privileged simulated sensing and is not camera-only autonomy.

The 60 m² L-shaped world has a 5×4 m living room, 5×4 m study and 5×4 m bedroom. Its two offset 1.2 m door openings require non-collinear travel and turns. It contains 51 static collision boxes: walls, furniture components and six random clutter boxes. Seed changes the clutter while retaining the room arrangement and a traversable circulation route.

The 1.2 m door descriptors include the partition end caps: actual collision-post clearance is 1.08 m. The RGB mode uses a 5 cm grid and reserves the cell half diagonal in addition to the 0.25 m footprint, 0.08 m safety margin and 0.02 m clearance allowance, for a 0.385355 m EDT threshold. Offline geometry tests confirm a crossing path even with 10 cm unknown strips beside each post; this is geometric feasibility, not RGB mission acceptance.

| Ideal-sensing clutter seed | Rooms physically entered | Distance | Simulated duration | Obstacle contacts | Observed reachable floor |
| --- | --- | --- | --- | --- | --- |
| 7, RGB-D acquisition enabled | 3/3 | 30.08 m | 482.0 s | 0 | 97.50% |
| 19 | 3/3 | 44.44 m | 567.4 s | 0 | 98.67% |
| 42 | 3/3 | 46.93 m | 606.7 s | 0 | 98.30% |

All three ideal-sensing runs reached the controller's terminal `complete` state. The reference run recorded 20 measured-heading surveys and 240 RGB-D frames (12 exposures per sweep). All image/depth pairs passed SHA-256 and dimensions checks. A baseline negative trial physically closes the bedroom door: exploration finishes in the accessible rooms, the bedroom remains unvisited, and all-room acceptance is false. None of these numbers is evidence of RGB-only mission completion.

Reports and evidence are local and ignored by Git:

- `artifacts/acceptance-seed-{7,19,42}/report.json`: measured acceptance results.
- `trajectory.json`, `trajectory.png`: actual sampled pose history and its map plot.
- `world.json`, `occupancy.npy`, `final_state.json`: scene, final sensor map and display snapshot.
- `artifacts/acceptance-seed-7/captures/`: original mounted-camera PNGs, float32 depth and per-frame metadata.
- `artifacts/blocked-door/`: negative-trial evidence.

Each room visit requires the actual base to enter at least 0.45 m inside that room's bounding edges. Coverage compares observed cells against an independently generated, inflated ground-truth floor grid connected to the spawn. It is observed reachable **floor** coverage, not photographed-surface coverage or a complete apartment reconstruction. Acceptance requires all room visits, terminal completion, no collisions, no failure, at least 95% observed reachable floor, and all requested capture records saved. Contacts with static obstacles over 0.05 N are counted; wheel-floor support is excluded. Final braking contacts also disqualify success.

## Run locally

From the Mark Rover repository:

```sh
../isaacprotocol/simulation/venv/bin/python auto-nav/setup_vision.py
../isaacprotocol/simulation/venv/bin/python auto-nav/setup_vision.py --check
auto-nav/run.sh simulate --sensing rgb --seed 7 --capture --output auto-nav/artifacts/rgb-seed-7
# Reproduce the historical ideal-sensing baseline separately:
auto-nav/run.sh simulate --sensing ideal --seed 7 --capture --output auto-nav/artifacts/ideal-seed-7
auto-nav/run.sh simulate --sensing ideal --seed 19 --output auto-nav/artifacts/ideal-seed-19
auto-nav/run.sh simulate --sensing ideal --seed 42 --output auto-nav/artifacts/ideal-seed-42
auto-nav/run.sh test -q
# Historical ideal-sensing closed-door trial; current RGB v6 evidence is linked above:
PYTHONPATH=auto-nav ../isaacprotocol/simulation/venv/bin/python auto-nav/tests/closed_door_trial.py
# Verify saved evidence and the frozen nine-source manifest without physics:
PYTHONPATH=auto-nav/.deps:auto-nav ../isaacprotocol/simulation/venv/bin/python \
  auto-nav/artifacts/validation-v6/verify_rgb_acceptance.py \
  auto-nav/artifacts/rgb-acceptance-seed-7-v6 \
  auto-nav/artifacts/rgb-acceptance-seed-19-v6 \
  auto-nav/artifacts/rgb-acceptance-seed-42-v6 \
  --output auto-nav/artifacts/validation-v6/rechecked-seeds-7-19-42.json
```

On macOS, rendering the actual camera needs a normal desktop graphics connection. A restricted sandbox can fail with `invalid CoreGraphics connection`; physics-only tests do not require camera rendering. The reference acquisition run was executed with desktop graphics access.

Run one full rendered physics mission at a time on this Mac; renderer work uses wall-clock futures, so avoid OS process suspension during rendering.

`run.sh` defaults to the existing sibling `isaacprotocol/simulation/venv/bin/python`. Set `ISAAC_PROTOCOL_ROOT` and `NAV_PYTHON` if those checkouts/runtimes are elsewhere. The adapter imports IsaacProtocol's own simulator, camera runtime and geometry extraction; it opens no physical rover connection. No NVIDIA Isaac Sim installation is involved: IsaacProtocol here uses MuJoCo 3.5.0.

Select `--sensing` explicitly for reproducible CLI runs: `simulate` defaults to `rgb`; reproducing the historical baseline requires `--sensing ideal`. CLI runs use the explicitly supplied `--output` directory.

The tested seed-7 world was also saved in the running IsaacProtocol environment catalog as `world_2fa5751ac91843b58f8791e705ca7942`, named **Mark Rover · three furnished rooms · seed 7**. CLI missions use their own isolated simulator session.

## Implementation

| File | Responsibility |
| --- | --- |
| `rover_nav/navigation.py` | Timestamped observations, occupancy evidence, unknown-space/obstacle inflation, reachable frontier clustering, A*, turn-before-translation tracking, measured yaw sweeps, capture scheduling, bounded replanning and stop gates. |
| `rover_nav/isaac.py` | Original rover loading, wheel mixing, actual contact/pose reads, mounted perspective ray observations and original RGB-D camera capture. |
| `rover_nav/vision_scene.py` | Physical phone-cradle pitch, XML-derived fixed camera calibration, unencoded floor texture and ordinary scene lighting. |
| `rover_nav/vision_isaac.py`, `vision.py` | RGB-only transport packet, camera perception integration and RGB captures; no truth/depth payload crosses into perception. |
| `rover_nav/visual_odometry.py` | Separate corner/support masks, validated persistent optical-flow tracks, regularized planar-landmark camera PnP, ranked RGB keyframes and gated registration. |
| `rover_nav/floor_vision.py` | Initial-image floor appearance learning, texture checks and conservative visible-floor intervals. |
| `rover_nav/world.py` | Reproducible world descriptor and static MJCF geometry; no planner dependency. |
| `rover_nav/mission.py` | Closed-loop mission, settled captures, physical room-entry checks, independent coverage and result artifacts. |
| `assets/` | Original CAD snapshot, unchanged exported rover MJCF, provenance/hash and repeatable Isaac CAD exporter. |

`Navigator.update(Observation)` emits linear/angular velocity. Observation contains a base-pose estimate, relative bearings, measured/estimated interval endpoints, observation time, tracking state and camera optical origin. RGB supplies an immutable ordered union of disjoint `free_intervals` per original bearing and separates positive obstacle contacts from guaranteed-free evidence. Empty unions remain unknown; missing unions alone fall back to the legacy first near/far pair. The mapper integrates each segment separately, clipping before positive contacts without filling texture gaps or occluded shadows. Its ranges are inverse-perspective floor estimates, not rendered depth. `Navigator.check(now)` returns zero on stale/invalid clocks. The simulated motor layer enforces a separate 350 ms command-age stop. The runner samples at 10 Hz and integrates 2 ms MuJoCo physics steps. Motor commands are velocity actuator targets; root positions are never scripted or teleported during a mission.

RGB translation requires historical clearance for the full buffered braking tube and image evidence corroborating each leading stopping direction. Normally two adjacent original bearings' current unions contain an actual confirmation point from the physical modeled stop cap through the extra 6 cm buffered cap plus one cell diagonal. A bounded recent-image fallback may prove that same world point in its original calibrated sensor frame, while current adjacent bearings independently show farther floor in that direction. It retains at most 64 immutable observations, no older than five seconds or farther than 0.4 m, and clears them after tracking/clock/pose faults, an observation gap over 0.5 s or leaving RGB sensing. No free endpoint, angular gap or unobserved prefix is extended; the maximum probe distance and historical braking buffer remain unchanged. The fixed offset from base to camera is included. In-place rotation without previous translation remains possible on a known-clear circular footprint. See [the RGB stopping-check description](RGB_NAVIGATION.md#fresh-stopping-direction-corroboration), its saved seed-42 collision regression and the real doorway interval test. The v6 seed-7, seed-19 and seed-42 full missions and strict closed-door negative passed with this method.

The skid-steer mixer has a documented simulation feedforward factor, calibrated from the original model's measured turning response. It preserves the original effort, friction, inertia and actuator speed limits (8 rad/s). The outer loop continually corrects using observed heading.

In `ideal` mode, the forward ray sensor samples the mounted camera's 4:3 perspective frustum and casts against the actual world. Self geometry is excluded from these baseline depth rays without changing contact masks. Occluders, including raised furniture, are projected into the 2D navigation map. Baseline RGB-D capture files contain intrinsics, true optical transforms, timestamps and checksums.

In `rgb` mode, the renderer returns RGB pixels and the image clock only. The complete phone cradle is physically pitched down 25°, retaining all attached parts and collision geometry. Static calibration defines the 320×240 camera's intrinsics and camera-to-base transform. Metric scale comes from the known mount height over a flat floor; visual pose begins at a local origin, independently of world spawn. The ordinary floor texture contains random speckles, with no labels or coordinate encoding. The classifier learns its appearance from the initial image. Captures contain RGB, fixed calibration, estimated pose and tracking state; they contain no rendered depth or true camera pose.

The current optical-flow settings are 9×9 windows and `maxLevel=2`. Fresh corners and descriptors require direct two-axis texture evidence; existing validated tracks may continue over the broader floor-patch support mask without requiring 12 newly detected corners in every exposure. Homography consensus and planar-landmark PnP estimate camera position/attitude with known-height and near-level priors. Only visually accepted observations advance the reference clock. Keyframe candidates prioritize a nearby view from an earlier rotation before spending the bounded matching budget on farther history. RGB survey angular targets are 0.30 rad/s, and roomier survey viewpoints are preferred within the existing hard footprint constraints.

Generated RGB worlds use spawn `[1.2, 1.3, 0]` rather than the ideal baseline's `[1, 1, 0]`, providing clearance for the initial floor patch and configured footprint. The RGB estimator still initializes at local `[0, 0, 0]`; the spawn belongs only to scene setup and evaluator/display alignment. The [static footprint/door audit](tests/test_static_geometry_audit.py) and [local JSON evidence](artifacts/static-geometry-audit.json) preserve the model-derived clearance proof without rendering or moving the rover.

The console provides `/api/scene`, `/api/status`, `/api/replay`, `/api/camera`, and POST `/api/start`, `/api/pause`, `/api/reset`. Start/reset requests accept `sensing: "rgb"` or `"ideal"`. Isaac's underlying API is mounted at `/isaac/`. The server binds loopback and requires local-owner requests for mutations. Start launches a fresh run from a saved result; pause clears requested commands, lets the simulated rover brake for 0.5 s, then zeros navigator command telemetry. Static mesh data is served with `/api/scene` rather than repeated in every status poll; status uses a direct JSON response. The mounted-camera inset shows an actual render or saved capture. Its observer view is separate from the RGB-only controller input boundary.

## Validation and boundaries

The baseline tests cover occluded shadows and invalid rays, sensor extrinsics, footprint inflation, A*, frontiers, capture sweeps, stop gates, actuator physics, contact detection and independent evaluation. RGB regressions additionally exercise fixed calibration, actual RGB fixtures, visual odometry, image-only transport, inaccessible truth/depth objects, floor-boundary uncertainty, blind intervals and frame loss. The import/data-boundary tests actively make simulator pose/ray reads fail while replaying RGB packets. They establish an input boundary, not a complete multi-room performance result.

The RGB classifier is a restricted flat-floor appearance model. Floor-textured obstacles, transparent surfaces, overhangs, low contrast, deep shadows, slopes, drop-offs and moving objects remain limitations or unsupported conditions. Optical-flow/keyframe estimates can drift; localization errors are measured using truth exclusively in the evaluator. The code has no ARCore application, physical motor gateway or validated real-world latency/stopping model. Neither baseline success nor RGB unit-test success establishes hardware safety, complete surface capture coverage or general apartment autonomy. Unknown space can remain behind furniture or outside the reachable component; terminal exploration completion must still pass the independent room-entry/contact/coverage criteria.

The saved-result page shows the measured final scene and trajectory; it is not a time-scrubbable animation. Start runs the actual closed-loop physics again.
