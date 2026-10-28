# Scene reconstruction

Status: experimental. The separate research scripts, manual structure assembly and feature-guided refinement CLI have been removed. Saved captures, runs and experimental outputs remain on disk. The current room model contains substantial local shape errors, including bent furniture, a bowed floor and large missing surfaces. Native photogrammetry is more continuous than the original CPU mesh, but the result is not an accurate or simulation-ready reconstruction.

The earlier manually annotated furniture fits are archived as scene-specific experiments and excluded from automatic viewer selection. They do not improve the general reconstruction method. The general pipeline evaluation (historical experiment) records tests on all 19 original photos: confidence calibration produced only a small CPU improvement, while confidence-weighted meshing degraded furniture. Neither result establishes a better replacement for native reconstruction.

The general-method search (historical experiment) adds camera-import checks, independent CPU stereo, MPS learned-geometry comparisons and normal-guided depth integration. None established a better complete room model; numerical gains that introduced surface artifacts were rejected.

This module turns saved phone captures into textured surface geometry. It is independent of rover control and navigation. Processing and storage remain on the laptop.

## Start

From the repository root:

```sh
python3 -m venv .venv-reconstruction
.venv-reconstruction/bin/python -m pip install -r reconstruction/requirements.txt
python3 reconstruction/toolchain/setup_photogrammetry.py
.venv-reconstruction/bin/python rover.py start --open
```

The native setup compiles the included Swift helper and checks whether this Mac supports `PhotogrammetrySession`. It requires the macOS Command Line Tools. It does not install COLMAP or OpenMVS. The separate [toolchain instructions](toolchain/README.md) describe that optional backend.

Open [http://rover.localhost:1355/](http://rover.localhost:1355/) on the laptop and click **Reconstruction**. Camera capture, controls and models share this entry point. Continue using the HTTPS phone camera link shown on Control; reconstruction reads its saved scans directly.

Choose a scan, method and quality. On a supported Mac, the page defaults to Apple photogrammetry at Full quality. Jobs run one at a time. Cancellation terminates the active processing or export process group. Retry verifies the immutable input snapshot and reuses a completed native model when its fingerprint and required outputs still match. An interrupted native solve may need to run again.

## Methods

### Apple photogrammetry

Original JPEG bytes and EXIF metadata are preserved. Numbered working copies retain capture order. New native sessions use normal feature sensitivity, unordered sampling and object masking disabled, following the same-image object-shape comparison. Both profile settings are recorded in the job and cache fingerprint. Full requests raw geometry; Draft requests the framework's smaller full-detail mesh. Native progress, omitted samples and automatic downsampling are recorded.

The installed macOS 14.4 SDK does not expose `ignoreBoundingBox`. The helper records that this setting was not applied. Its absence, the scene boundaries and any resulting truncation must be considered when assessing a room model.

All native exports retain the same Apple Y-up coordinate frame. Camera centres and orientations come from that session, with the encoded-image axes converted using preserved EXIF orientation. Estimated camera intrinsics are unavailable through this SDK. Capture viewpoints therefore use a display-only 60-degree vertical field of view, rather than claiming exact pixel alignment.

### COLMAP / OpenMVS

This backend normalises orientation in separate working images, estimates camera poses and sparse structure with COLMAP, checks registration, undistorts images and reconstructs dense surfaces with OpenMVS. Draft requests Metal when available. Full uses CPU PatchMatch because it recovered more coherent surfaces in the first room comparison. This is a dataset-specific choice. The active backend must be confirmed by its log.

The original captures and earlier runs remain unchanged. Each run records its engine identities, configuration, input checksums and outputs.

## Viewing and exports

The viewer supports orbit, pan, zoom, capture viewpoints, textured surfaces, geometry and wireframe modes. Native Overview uses the file's declared Y-up axis. The classic backend estimates display-up from a consensus of camera image-up directions. Display controls do not alter mesh coordinates. Raw and metric models use camera positions in their corresponding coordinate frame.

Exports include a standalone textured `scene.glb`, `surface.ply`, a simplified `collision.obj` candidate, camera records, provenance and an archive containing the OBJ and its texture dependencies. Baked photographic colours use an explicit diffuse, nonmetallic display material. This corrects glTF's default metallic appearance; it does not estimate physical reflectance or repair geometry.

Scale is unverified for both methods. Native file unit declarations do not establish measured dimensions. **Set scale** creates separate metric exports from two selected points and their measured separation. One distance fixes uniform scale; it does not validate shape distortion. Floor alignment and collision accuracy remain unverified.

Processing completion is distinct from reconstruction quality. The topology report measures connectivity after welding coincident positions for analysis. Texture seams do not count as separate pieces. A connected mesh can still contain gaps, false closures or warped objects. Neither connectivity nor triangle count measures recovered apartment coverage.

## Inspecting the model

The viewpoint menu stores camera positions within one reconstructed 3D scene. Choosing a camera position moves the viewer; it does not replace the scene with a source photograph. Camera field of view remains approximate when native intrinsics are unavailable.

Click **Walk** to explore from the current viewpoint. With the viewer focused, **W/A/S/D** move across the scene, **Q/E** move down/up, **Shift** increases speed, and dragging looks around. **Escape** exits Walk. Movement stops on key release, loss of focus or tab hiding. Camera travel follows the scene up axis and changes no model coordinates or exports. Walk is free camera inspection without gravity or collision; it is not a physics simulation.

## Conservative gap completion

**Fill gaps** creates a separate completed variant of the selected raw or metric model. The original reconstruction remains available in the model menu. **Completed · inferred** shows the original mesh with added surfaces; **Inferred surfaces only** isolates the additions. The status line reports filled boundaries and openings preserved.

The first completion method triangulates supported, closed, approximately horizontal planar boundaries. It preserves vertical openings and boundaries that fail the geometric checks. It does not close the entire room with a convex hull or invent unseen furniture. Boundary colour samples provide a display approximation rather than recovered photographic texture. Inferred surfaces remain distinguishable in separate mesh nodes and exports.

Versioned outputs live under `exports/completed/<fingerprint>/` or `exports/scaled/completed/<fingerprint>/`: `scene-completed.glb`, `surface-completed.ply`, `inferred-surfaces.ply`, `inferred.glb` and `completion.json`. The report records source hashes, accepted and rejected loops, added geometry and topology before and after completion. These exports keep the coordinate frame of their source model and use the matching camera records.

On the same 19-image room scan, the first completed variant adds 254 triangles across three bounded horizontal openings: two lower openings and one at the ceiling. Open boundary edges decrease from 304 to 44, with no added nonmanifold edges. The remaining vertical opening is preserved. Original geometry and embedded texture buffer bytes are retained exactly; added surfaces use flat estimated colours, not synthesized photographic texture. These topology checks do not establish physical accuracy.

Scale calibration and completion cannot run concurrently for the same reconstruction. Recalibrating scale invalidates the previous metric completion; raw completion remains available. Filling gaps does not establish geometric truth, collision safety or simulation readiness. Large unsupported gaps may remain open.

## Same-image evaluation

The first room comparison used the same 19 original photographs throughout.

| Configuration | Result |
| --- | --- |
| Original Metal depth pipeline | 350,877 fused points; large wall and floor gaps |
| CPU depth, 2400-pixel limit | 1,042,052 fused points; refined mesh with 321,556 faces and 1,107 connected pieces |
| CPU depth, 1600-pixel control | Similar sparse-anchor precision, reduced floor and wall support; not selected as an improvement |
| CPU, more iterations and geometric checks | 383,556 faces and 1,688 connected pieces; additional compute did not resolve the blank floor and walls |
| Apple raw photogrammetry | All 19 cameras registered; 432,785 faces, one connected component and 251 boundary edges; visibly more continuous room surfaces |

Apple's initial four-output benchmark completed in 226.2 seconds. That timing includes raw/full OBJ and USDZ requests; it is not a raw-only service latency measurement. Native and COLMAP camera frames were independently compared before cross-frame inspection. Production native exports use their own consistent frame.

A subsequent end-to-end run through the webpage and production worker used the same 19 originals and a single raw OBJ request. The native solve took 99.2 seconds, followed by 18.1 seconds of export. It registered all 19 cameras and produced 420,596 faces in one connected component, with 304 boundary edges and no detected nonmanifold edges. Original-image hashes, mesh and camera frames, texture dependencies and archive integrity were checked independently. These checks establish export consistency, not complete or metrically accurate room recovery; ceiling holes and local surface distortions remain.

A larger pose-conditioned depth experiment also ran on MPS with all 19 images and higher-resolution overlap groups. It recovered useful planar hypotheses but remained less accurate on established feature tracks and produced fragmented direct surfaces. It is retained for research and is not the default reconstruction method. Manually bounded floor and wall completion variants are labelled inferred and unvalidated.

## Storage and verification

`captures/` holds original scans; `runs/` holds immutable job snapshots, logs and exports. Native working data is under each run's `native/`; classic working data uses `prepared/`, `sfm/`, `dense/` and `openmvs/`. These directories, local tool builds and experimental predictions are ignored by Git.

The service binds to `127.0.0.1` and has no motor-control endpoints. No database service or cloud processing is required. The classic backend uses an internal SQLite working file. Viewer dependencies are vendored locally.

```sh
PYTHONDONTWRITEBYTECODE=1 .venv-reconstruction/bin/python -m unittest discover -s reconstruction -p 'test_*.py' -v
node --test reconstruction/web/orientation.test.mjs
```

Fixture tests cover snapshots, method dispatch, cancellation, recovery, coordinate frames, material exports, scale calibration and separate gap-completion exports. Completion checks include source preservation, zero-fill reports, unsafe paths, persistence, export locking and recalibration invalidation. Browser checks cover actual model loading, capture viewpoints, metric camera positions, raw and metric completion variants, inferred-only viewing, manual selection during polling, mobile layout and local resource loading. Native engine benchmarks are separate from fixture tests. No test commands the rover to move.

### Object-shape refinement

A same-image comparison selected unordered/normal raw reconstruction followed by a bounded correction fitted to separate training feature tracks. The refined viewer run retains the original run, a before-correction model and separate inferred gap fills. Median/p90 surface disagreement on 964 disjoint validation features decreased from 4.981%/8.043% for the original model to 0.967%/2.998%. This is agreement with a same-image SfM reference, not measured physical accuracy. Thin and reflective objects retain artifacts. New webpage jobs use the selected native profile; the experimental feature-guided calibration CLI has been removed.

### Surface-quality audit

Off-angle inspection of the proportion-corrected run exposed severe local geometry errors that the sparse-feature score did not measure. A structural shell candidate and three desk-panel repairs were tested separately; none was promoted as an improved production model. The webpage now describes gap filling as partial and distinguishes unfilled boundaries from intentionally preserved architectural openings. The existing photos and geometry are retained.
