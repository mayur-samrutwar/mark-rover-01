# Reconstruction toolchain

The module supports Apple photogrammetry and an independent COLMAP/OpenMVS backend. Both run locally. The installation targets macOS on Apple Silicon.

## Apple photogrammetry

```sh
python3 reconstruction/toolchain/setup_photogrammetry.py
```

This compiles the included `photogrammetry.swift` with the installed Apple Command Line Tools and RealityKit SDK. It downloads no packages and writes the helper, module cache, build log and capability manifest under `.reconstruction-tools/`. The page exposes this method only when the helper's runtime support probe succeeds.

Full quality requests raw OBJ geometry; Draft requests the smaller full-detail mesh. Each session also exports its own camera poses, bounds and sparse points. Normal feature sensitivity, unordered sampling and masking disabled match the selected object-shape comparison. The helper also accepts `--sensitivity high` and `--ordering sequential` for controlled comparisons; both settings are fingerprinted by the driver. The installed macOS 14.4 SDK lacks `ignoreBoundingBox`; the helper records that it is not applied. Native framework scheduling is internal, so environment thread limits are not a framework thread-count guarantee.

The initial 19-image room benchmark completed on this M3 Pro and retained all 19 cameras. That actual reconstruction is separate from fixture tests. `--probe-only` checks support without starting a reconstruction. Toolchain availability does not certify scene coverage, shape accuracy or metric scale.

The standalone helper is `.reconstruction-tools/bin/PhotogrammetryReconstruct`. Its source, compiler, SDK path, binary hash and support result are recorded in `.reconstruction-tools/photogrammetry-toolchain.json`.

## COLMAP / OpenMVS

COLMAP estimates camera calibration and sparse structure. OpenMVS estimates depth, reconstructs and refines surfaces, and textures the mesh. This backend is optional when using Apple photogrammetry.

### Install

Command Line Tools and Homebrew must already be available. From the repository
root:

```sh
python3 reconstruction/toolchain/setup_macos.py --jobs 2
```

The script installs missing Homebrew dependencies, verifies the official COLMAP
archive against its published SHA-256, and builds the pinned OpenMVS source.
Use `--skip-brew` when dependencies are already installed. Build concurrency is
limited to 1–4 jobs; the default is two to reduce memory pressure on an 18 GB Mac.
Full Xcode is not required: the Metal shader is embedded and compiled by the
Metal runtime.

Executables and build artifacts are stored in the ignored
`.reconstruction-tools/` directory. No Python package files are overwritten to
resolve Homebrew linking conflicts. CMake uses the separate `opencv@4` keg directly; OpenCV 5 introduces datatype
specializations incompatible with this pinned OpenMVS source.

### Versions and scope

`pins.json` records exact source revisions and the COLMAP archive checksum.
COLMAP 4.2.1 is installed from its official ARM64 release. OpenMVS is a pinned
post-2.4.0 development revision because its Metal backend is newer than the
2.4.0 release. The OpenMVS version banner therefore still identifies 2.4.0;
the Git revision in `.reconstruction-tools/toolchain.json` disambiguates it.
Homebrew dependency versions are recorded in that local installation manifest;
Homebrew packages are not independently frozen by this repository.

`openmvs-headless.patch` changes CMake configuration to exclude the unused native SfM
library, alternate importers, video extraction and GUI applications. COLMAP
supplies camera poses. The patch leaves OpenMVS's dense reconstruction, meshing,
refinement, texturing and Metal algorithm sources unchanged. Both this patch
and `halfmesh-headless.patch` restrict OpenCV linkage to the required modules,
avoiding unrelated GUI, video and machine-learning libraries at process startup.

The installed command paths are:

```text
.reconstruction-tools/bin/colmap
.reconstruction-tools/bin/InterfaceCOLMAP
.reconstruction-tools/bin/DensifyPointCloud
.reconstruction-tools/bin/ReconstructMesh
.reconstruction-tools/bin/RefineMesh
.reconstruction-tools/bin/TextureMesh
.reconstruction-tools/bin/TransformScene
```

`DensifyPointCloud --gpu-device -1` requests the Metal backend on this build.
`--gpu-device cpu` or `-2` selects CPU processing. OpenMVS can fall back to CPU if
the Metal runtime is unavailable, so a compiled Metal backend does not establish
that a particular job used the GPU. The backend log must confirm activation.
COLMAP's `without GPU support` banner describes its native CUDA build capability;
it does not by itself determine whether optional ONNX models can use CoreML.

### Verify actual GPU execution

```sh
python3 reconstruction/toolchain/smoke_metal.py
```

This copies the upstream four-image test scene into an ignored test directory,
converts its two HEIC images to JPEG with macOS, preserves its known camera
parameters, and runs depth estimation and fusion. It requires an explicit Metal activation
message, a successful exit, and a nonempty output point cloud. The log and JSON
report distinguish this from merely compiling Metal support. It updates the local
manifest's `validation.metal_active` field only after a successful run.

The smoke test uses known upstream camera poses. It does not test COLMAP camera
recovery, apartment coverage, metric scale or collision accuracy. Integration
tests must evaluate those stages separately. No rover motor commands are sent.

Sources: [COLMAP release](https://github.com/colmap/colmap/releases/tag/4.2.1),
[OpenMVS revision](https://github.com/cdcseacave/openMVS/commit/610562a4f033be2aaf4f240512c93e1038186768),
[Metal implementation and upstream validation](https://github.com/cdcseacave/openMVS/pull/1279).
