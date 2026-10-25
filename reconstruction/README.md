# 3D scene reconstruction

Status: not yet implemented.

This module will develop the pipeline for reconstructing indoor scenes from rover observations, using 3D Gaussian splatting as the intended scene representation.

## Design questions

- Camera selection, calibration, and data acquisition.
- Observation timestamps and camera-pose estimation.
- Dataset preparation and coordinate-frame alignment.
- Reconstruction, rendering, and export.
- Evaluation of visual quality, geometric consistency, and processing cost.

The camera configuration, pose-estimation method, reconstruction implementation, and evaluation protocol have not been selected. Integration with [`../auto-nav/`](../auto-nav/) will require consistent pose, calibration, and timestamp conventions.
