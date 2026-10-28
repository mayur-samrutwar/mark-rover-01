"""Experimental image-only dense floor odometry; no deployed code changes.

Replay stored RGB through FloorVision, remap to a metric floor canvas and fit
Euclidean image alignment. Ground-truth files are loaded only after all image
estimates have been computed. This is a bounded prototype, not mission logic.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image

from rover_nav.floor_vision import FloorVision


ROOT = Path(__file__).resolve().parents[1]


def wrapped(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def evaluation_summary(records, truth_records):
    accepted = [item["evaluation_only"] for item in records if item["tracking"]]
    baselines = {}
    for name in ("legacy_se2", "camera_pnp", "camera_pnp_fresh_reseed"):
        selected = [item[name] for item in truth_records if isinstance(item.get(name), dict)]
        if not selected:
            continue
        baselines[name] = {
            "max_position_error_m": max(item["position_error_m"] for item in selected),
            "final_position_error_m": selected[-1]["position_error_m"],
            "max_yaw_error_deg": max(abs(item["yaw_error_deg"]) for item in selected),
            "final_yaw_error_deg": selected[-1]["yaw_error_deg"],
        }
    return {"accepted_tracking_frames": len(accepted),
            "max_yaw_error_while_tracking_deg": max(abs(item["yaw_error_deg"]) for item in accepted),
            "last_tracked_yaw_error_deg": accepted[-1]["yaw_error_deg"],
            "stored_baselines_evaluation_only_same_frames": baselines}


class DenseFloorOdometry:
    def __init__(self, calibration, *, resolution=.01, max_forward=2., highpass=True):
        self.K = np.asarray(calibration["K"], float)
        self.T = np.asarray(calibration["T_base_camera"], float)
        self.width, self.height = calibration["width"], calibration["height"]
        self.floor = FloorVision(self.K, self.T, self.width, self.height)
        self.resolution, self.near, self.lateral = resolution, .15, 1.
        self.shape = (round(2 * self.lateral / resolution), round((max_forward - self.near) / resolution))
        yy, xx = np.indices(self.shape)
        ground = np.column_stack(((self.near + xx * resolution).ravel(),
                                  (self.lateral - yy * resolution).ravel(),
                                  np.zeros(xx.size), np.ones(xx.size)))
        camera = (ground @ np.linalg.inv(self.T).T)[:, :3]
        image = camera @ self.K.T
        pixels = image[:, :2] / image[:, 2, None]
        self.map_x = pixels[:, 0].reshape(self.shape).astype(np.float32)
        self.map_y = pixels[:, 1].reshape(self.shape).astype(np.float32)
        self.projectable = ((camera[:, 2] > 0) & (pixels[:, 0] > 3)
                            & (pixels[:, 0] < self.width - 3) & (pixels[:, 1] > 3)
                            & (pixels[:, 1] < self.height - 3)).reshape(self.shape)
        self.highpass = highpass
        self.pose = np.zeros(3)
        self.reference = None
        self.reference_mask = None
        self.reference_time = None
        self.previous_warp = np.eye(2, 3, dtype=np.float32)
        self.last_bev = None
        self.last_mask = None

    def prepare(self, rgb):
        floor = self.floor.process(rgb)
        mask = floor.tracking_mask if floor.tracking_mask is not None else floor.mask
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32) / 255.
        bev = cv2.remap(gray, self.map_x, self.map_y, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT)
        valid = cv2.remap(mask.astype(np.uint8), self.map_x, self.map_y,
                         cv2.INTER_NEAREST).astype(bool) & self.projectable
        valid = cv2.erode(valid.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        bev = cv2.GaussianBlur(bev, (3, 3), .6)
        if self.highpass:
            bev = bev - cv2.GaussianBlur(bev, (0, 0), 6.)
        self.last_bev, self.last_mask = bev, valid
        return bev, valid, floor

    def update(self, rgb, timestamp):
        bev, mask, floor = self.prepare(rgb)
        count = int(mask.sum())
        record = {"timestamp": float(timestamp), "floor_valid": bool(floor.valid),
                  "floor_reason": floor.reason, "floor_pixels_bev": count,
                  "pose": self.pose.tolist(), "tracking": False}
        if not floor.valid or count < 200:
            return {**record, "reason": "insufficient_floor"}
        if self.reference is None:
            self.reference, self.reference_mask, self.reference_time = bev, mask, timestamp
            return {**record, "reason": "initial_rgb_reference", "tracking": True}
        overlap = cv2.erode((mask & self.reference_mask).astype(np.uint8), np.ones((3, 3), np.uint8))
        if int(overlap.sum()) < 120:
            return {**record, "reason": "insufficient_overlap"}
        warp = self.previous_warp.copy()
        coefficient = None
        try:
            # Two image scales improve convergence without an external motion
            # seed. Translation scales with image size, rotation does not.
            for scale in (.5, 1.):
                template = cv2.resize(self.reference, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                current = cv2.resize(bev, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                selection = cv2.resize(overlap, (template.shape[1], template.shape[0]), interpolation=cv2.INTER_NEAREST) * 255
                scaled_warp = warp.copy()
                scaled_warp[:, 2] *= scale
                coefficient, scaled_warp = cv2.findTransformECC(template, current, scaled_warp,
                    cv2.MOTION_EUCLIDEAN, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 60, 1e-6),
                    inputMask=selection, gaussFiltSize=3)
                warp = scaled_warp.copy()
                warp[:, 2] /= scale
        except cv2.error as error:
            return {**record, "reason": "ecc_failed", "error": str(error).splitlines()[-1]}
        # Canvas columns are forward; rows are negative left. Convert the
        # template-to-input pixel transform to previous-to-current base SE(2).
        basis = np.diag([1 / self.resolution, -1 / self.resolution])
        offset = np.array([-self.near / self.resolution, self.lateral / self.resolution])
        rotation = np.linalg.inv(basis) @ warp[:, :2] @ basis
        translation = np.linalg.inv(basis) @ (warp[:, :2] @ offset + warp[:, 2] - offset)
        delta_yaw = -math.atan2(rotation[1, 0], rotation[0, 0])
        delta_xy = -rotation.T @ translation
        age = float(timestamp - self.reference_time)
        if coefficient < .6 or abs(delta_yaw) > .12 * max(1., age / .1) or np.linalg.norm(delta_xy) > .08 * max(1., age / .1):
            return {**record, "reason": "ecc_motion_gate", "correlation": coefficient,
                    "delta_yaw_deg": math.degrees(delta_yaw), "delta_xy_m": delta_xy.tolist()}
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[:2] += np.array([[c, -s], [s, c]]) @ delta_xy
        self.pose[2] += delta_yaw
        self.reference, self.reference_mask, self.reference_time = bev, mask, timestamp
        self.previous_warp = warp
        return {**record, "tracking": True, "reason": "dense_ecc_floor_alignment",
                "pose": self.pose.tolist(), "correlation": coefficient,
                "delta_yaw_deg": math.degrees(delta_yaw), "delta_xy_m": delta_xy.tolist()}


def run(dataset, destination, limit=50, *, highpass=True, resolution=.01):
    destination.mkdir(parents=True, exist_ok=True)
    calibration = json.loads((dataset / "calibration.json").read_text())
    sequence = np.load(dataset / "rgb_sequence.npz")
    estimator = DenseFloorOdometry(calibration, highpass=highpass, resolution=resolution)
    records, consecutive_failures = [], 0
    started = time.monotonic()
    for image, timestamp in zip(sequence["rgb"][:limit], sequence["timestamps"][:limit]):
        record = estimator.update(image, timestamp)
        records.append(record)
        consecutive_failures = 0 if record["tracking"] else consecutive_failures + 1
        if consecutive_failures >= 10:
            break
    # All truth is loaded after inference completes and is used for scores only.
    truth_records = json.loads((dataset / "diagnostics.json").read_text())[:len(records)]
    errors = []
    for estimate, truth in zip(records, truth_records):
        expected = np.asarray(truth["truth_local_evaluation_only"])
        pose = np.asarray(estimate["pose"])
        error = {"position_error_m": float(np.linalg.norm(pose[:2] - expected[:2])),
                 "yaw_error_deg": math.degrees(wrapped(pose[2] - expected[2]))}
        estimate["evaluation_only"] = error
        errors.append(error)
    report = {"prototype_only": True, "dataset": str(dataset), "frames_requested": limit,
              "frames_processed": len(records), "tracking_failures": sum(not record["tracking"] for record in records),
              "wall_seconds": time.monotonic() - started, "highpass": highpass,
              "bev_resolution_m": resolution,
              "online_inputs": "RGB, image timestamps and static camera K/T only; truth loaded after inference",
              "max_position_error_m": max(item["position_error_m"] for item in errors),
              "final_position_error_m": errors[-1]["position_error_m"],
              "max_yaw_error_deg": max(abs(item["yaw_error_deg"]) for item in errors),
              "final_yaw_error_deg": errors[-1]["yaw_error_deg"],
              "final_estimated_unwrapped_yaw_deg": math.degrees(records[-1]["pose"][2]),
              "failure_reasons": {reason: sum(record["reason"] == reason for record in records)
                                  for reason in sorted(set(record["reason"] for record in records))}}
    report.update(evaluation_summary(records, truth_records))
    (destination / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    (destination / "diagnostics.json").write_text(json.dumps(records, indent=2) + "\n")
    if estimator.last_bev is not None:
        display = np.clip(estimator.last_bev * (4 if highpass else 1) + (.5 if highpass else 0), 0, 1)
        Image.fromarray((display * 255).astype(np.uint8)).save(destination / "last_bev.png")
        Image.fromarray(estimator.last_mask.astype(np.uint8) * 255).save(destination / "last_bev_mask.png")
    print(json.dumps(report, indent=2), flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=ROOT / "artifacts" / "vo-near-wall")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts" / "dense-bev-prototype")
    parser.add_argument("--frames", type=int, default=50)
    parser.add_argument("--resolution", type=float, default=.01)
    parser.add_argument("--no-highpass", action="store_true")
    args = parser.parse_args()
    run(args.dataset, args.output, args.frames, highpass=not args.no_highpass, resolution=args.resolution)
