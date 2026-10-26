# 3D scene reconstruction

Status: manual RGB acquisition implemented; scene reconstruction not yet implemented.

The target output is a metric, textured surface model with collision geometry suitable for mobile-robot navigation simulation. Gaussian splatting may be evaluated as an intermediate representation or appearance layer. It is not the required final scene format.

## Current acquisition

The laptop controller pairs with a phone browser, receives a low-resolution live preview and requests individual JPEG photographs. Observations are grouped by capture position and saved with JSON manifests. Finished groups remain part of a scan that can be exported as a ZIP archive. No database is used.

Implementation is shared with the existing local controller in `wifi-control/rover_control/`. See [`capture/README.md`](capture/README.md) for setup and the recording procedure. Datasets are stored under the ignored `captures/` directory and are not committed to Git.

This browser version records RGB images, timestamps, dimensions and available camera settings. It does not record ARCore poses, calibrated camera intrinsics or depth. A later image-based reconstruction stage must estimate poses and establish scale using independent measurements.

## Planned processing

- Estimate or refine camera alignment from overlapping observations at different positions.
- Reconstruct surfaces and generate a textured visual mesh.
- Prepare separate collision geometry while preserving measured openings and clearances.
- Validate dimensions, traversability, processing cost and manual repair effort.

The reconstruction backend and simulation export pipeline have not been implemented or benchmarked. Autonomous navigation remains a separate module.
