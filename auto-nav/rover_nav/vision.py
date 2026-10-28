"""RGB-only perception boundary: no simulator state or metric depth enters."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .floor_vision import FloorVision
from .navigation import Observation
from .visual_odometry import VisualOdometry


@dataclass(frozen=True)
class RGBPacket:
    pixels: np.ndarray
    timestamp: float


class CameraPerception:
    """Consume only image bytes, image time and fixed physical calibration."""
    def __init__(self, calibration):
        self.K = np.asarray(calibration["K"], dtype=float)
        self.T = np.asarray(calibration["T_base_camera"], dtype=float)
        self.width, self.height = calibration["width"], calibration["height"]
        self.floor = FloorVision(self.K, self.T, self.width, self.height)
        self.odometry = VisualOdometry(self.K, self.T, self.width, self.height)
        self.last_floor = None
        self.last_vo = None
        self.frames = 0

    def process(self, packet: RGBPacket):
        # Use the previous accepted image attitude only to propose this
        # frame's floor features. The estimator independently fits the new
        # image; mapping then uses that same frame's estimated camera pose.
        self.floor.set_camera_transform(self.odometry.T_level_camera)
        floor = self.floor.process(packet.pixels)
        features_mask = floor.tracking_mask if floor.tracking_mask is not None else floor.mask
        visual = self.odometry.update(packet.pixels, packet.timestamp,
                                      ground_mask=features_mask, feature_mask=floor.feature_mask)
        camera_transform = self.odometry.T_level_camera
        if visual.tracking and not np.allclose(camera_transform, self.floor.T, atol=1e-8, rtol=0.):
            self.floor.set_camera_transform(camera_transform)
            floor = self.floor.process(packet.pixels)
        self.last_floor, self.last_vo = floor, visual
        self.frames += 1
        pose = visual.pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        offset = np.array([[c, -s], [s, c]]) @ camera_transform[:2, 3]
        return Observation(pose=pose, angles=floor.angles, ranges=floor.ranges,
                           min_ranges=floor.min_ranges, hits=floor.hits,
                           hit_ranges=floor.hit_ranges,
                           free_intervals=floor.free_intervals,
                           max_range=self.floor.max_range, timestamp=packet.timestamp,
                           tracking=bool(visual.tracking),
                           sensor_origin=(pose.x + offset[0], pose.y + offset[1]))

    def status(self):
        result = {"source":"rgb_only", "frames":self.frames}
        if self.last_vo:
            v=self.last_vo
            result.update(tracking=v.tracking, inliers=v.inliers, features=v.features,
                          reprojection_error_px=v.reprojection_error if math.isfinite(v.reprojection_error) else None,
                          reason=v.reason, visual_relocalizations=self.odometry.relocalizations)
            result["camera_fit"]=self.odometry.last_camera_fit
        if self.last_floor:
            result.update(floor_confidence=self.last_floor.confidence,
                          floor_valid=self.last_floor.valid, floor_reason=self.last_floor.reason)
        return result
