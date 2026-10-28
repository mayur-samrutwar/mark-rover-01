"""RGB-only metric floor odometry with visually estimated camera attitude.

Initial scale and floor plane come from fixed camera calibration. Tracked
floor landmarks then constrain full camera extrinsics with planar PnP, so
chassis roll/pitch/height need not masquerade as planar robot motion. Only
RGB, exposure timestamps and fixed mount calibration enter this module.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import cv2
import numpy as np
from scipy.optimize import least_squares
from .navigation import Pose, wrap_angle


@dataclass(frozen=True)
class VOResult:
    pose: Pose
    tracking: bool
    inliers: int
    features: int
    reprojection_error: float
    reason: str


class VisualOdometry:
    """Optical flow, planar PnP and historical RGB landmark registration.

    OpenCV camera axes are right/down/forward; robot axes are forward/left/up.
    T_base_camera is fixed physical mount calibration. The initial image sets
    an arbitrary world frame with floor z=0 and measured camera height. New
    landmarks use estimated camera attitude rather than a level-chassis prior.
    Failed frames never advance pose or replace image references. Old visual
    references require reset(), which also requires resetting the caller's map.
    """
    def __init__(self, K, T_base_camera, width, height, *, max_features=500,
                 min_inliers=12, max_reference_age=1.):
        self.K = np.array(K, dtype=np.float64, copy=True)
        self.T_base_camera = np.array(T_base_camera, dtype=np.float64, copy=True)
        self.width, self.height = int(width), int(height)
        if self.K.shape != (3, 3) or self.T_base_camera.shape != (4, 4):
            raise ValueError("Expected 3x3 intrinsics and 4x4 camera-to-base calibration")
        if not np.all(np.isfinite(self.K)) or not np.all(np.isfinite(self.T_base_camera)):
            raise ValueError("Calibration must be finite")
        if self.width < 32 or self.height < 32 or self.K[0, 0] <= 0 or self.K[1, 1] <= 0:
            raise ValueError("Invalid image dimensions or focal length")
        rotation = self.T_base_camera[:3, :3]
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)
                or not np.allclose(self.T_base_camera[3], [0, 0, 0, 1])
                or self.T_base_camera[2, 3] <= 0):
            raise ValueError("Camera must be rigidly calibrated above the floor")
        self._K_inverse = np.linalg.inv(self.K)
        self._camera_base = np.linalg.inv(self.T_base_camera)
        self.max_features = max(24, int(max_features))
        self.min_inliers = max(6, int(min_inliers))
        self.max_reference_age = float(max_reference_age)
        if not math.isfinite(self.max_reference_age) or self.max_reference_age <= 0:
            raise ValueError("Reference age must be positive and finite")
        yy, xx = np.mgrid[:self.height, :self.width]
        ground = self.pixels_to_ground(np.column_stack((xx.ravel(), yy.ravel())))
        distances = np.linalg.norm(ground, axis=1).reshape(self.height, self.width)
        self._geometric_mask = (np.all(np.isfinite(ground), axis=1).reshape(
            self.height, self.width) & (distances >= .08) & (distances <= 3.5))
        self._orb = cv2.ORB_create(nfeatures=650, edgeThreshold=12, patchSize=23,
                                  fastThreshold=8, nlevels=6)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.reset()

    def reset(self):
        self.pose = Pose(0., 0., 0.)
        self._T_world_camera = self.T_base_camera.copy()
        self._reference_gray = self._reference_world = self._tracking_points = None
        self._reference_timestamp = self._accepted_timestamp = self._last_timestamp = None
        self._keyframes = []
        self._last_map_check = -math.inf
        self.relocalizations = 0
        self.last_camera_fit = None

    @property
    def T_level_camera(self):
        """Estimated camera -> yaw-level base with its origin at floor z=0.

        This includes estimated camera roll/pitch and floor height, for the
        same RGB frame's floor projection. Returned calibration is a copy.
        """
        c, s = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        world_level = np.eye(4)
        world_level[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
        world_level[:2, 3] = [self.pose.x, self.pose.y]
        return np.linalg.inv(world_level) @ self._T_world_camera

    def _ground_world(self, pixels, world_camera):
        pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
        rays = np.column_stack((pixels, np.ones(len(pixels)))) @ self._K_inverse.T
        rays = rays @ world_camera[:3, :3].T
        center = world_camera[:3, 3]
        scale = np.full(len(pixels), np.nan)
        downward = rays[:, 2] < -1e-5
        scale[downward] = -center[2] / rays[downward, 2]
        points = center + scale[:, None] * rays
        points[:, 2] = 0.
        return points

    def pixels_to_ground(self, pixels):
        """Fixed-calibration projection utility, in initial robot XY."""
        return self._ground_world(pixels, self.T_base_camera)[:, :2]

    def _project_ground(self, ground):
        points = np.column_stack((ground, np.zeros(len(ground)), np.ones(len(ground))))
        camera = (points @ self._camera_base.T)[:, :3]
        image = camera @ self.K.T
        return image[:, :2] / image[:, 2, None]

    def _base_pose(self, world_camera):
        base = world_camera @ self._camera_base
        return Pose(float(base[0, 3]), float(base[1, 3]), math.atan2(base[1, 0], base[0, 0]))

    def _mask(self, ground_mask):
        if ground_mask is None:
            mask = self._geometric_mask.copy()
            mask[:int(.4 * self.height)] = False
        else:
            ground_mask = np.asarray(ground_mask)
            if ground_mask.shape != (self.height, self.width):
                raise ValueError("Ground mask must match RGB dimensions")
            mask = self._geometric_mask & ground_mask.astype(bool)
        return cv2.erode(mask.astype(np.uint8) * 255, np.ones((5, 5), np.uint8))

    def _features(self, gray, mask):
        features = cv2.goodFeaturesToTrack(gray, maxCorners=self.max_features,
            qualityLevel=.008, minDistance=6, mask=mask, blockSize=5)
        return np.empty((0, 1, 2), np.float32) if features is None else features

    def _result(self, tracking, inliers=0, features=0, error=math.inf, reason=""):
        return VOResult(self.pose, bool(tracking), int(inliers), int(features), float(error), reason)

    def _solve_camera(self, landmarks, pixels, guess, required=None):
        required = self.min_inliers if required is None else required
        selected = np.zeros(len(landmarks), bool)
        if len(landmarks) < required:
            return None, selected, math.inf, "insufficient_floor_matches"
        _, consensus = cv2.findHomography(landmarks[:, :2], pixels, cv2.RANSAC, 1.5,
                                          maxIters=800, confidence=.995)
        if consensus is None:
            return None, selected, math.inf, "planar_consensus_failed"
        selected = consensus.ravel().astype(bool)
        if selected.sum() < required or selected.sum() < .45 * len(landmarks):
            return None, selected, math.inf, "insufficient_planar_consensus"
        points = landmarks[selected]
        if np.linalg.svd(points[:, :2] - points[:, :2].mean(axis=0), compute_uv=False)[-1] < .025:
            return None, selected, math.inf, "degenerate_floor_features"
        camera_world = np.linalg.inv(guess)
        rvec = cv2.Rodrigues(camera_world[:3, :3])[0]
        tvec = camera_world[:3, 3].reshape(3, 1).copy()
        ok, rvec, tvec = cv2.solvePnP(np.ascontiguousarray(points),
            np.ascontiguousarray(pixels[selected]), self.K, None, rvec, tvec,
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, selected, math.inf, "camera_pnp_failed"
        projected = cv2.projectPoints(landmarks, rvec, tvec, self.K, None)[0].reshape(-1, 2)
        selected &= np.linalg.norm(projected - pixels, axis=1) < 1.75
        if selected.sum() < required or selected.sum() < .45 * len(landmarks):
            return None, selected, math.inf, "insufficient_pnp_consensus"
        ok, rvec, tvec = cv2.solvePnP(np.ascontiguousarray(landmarks[selected]),
            np.ascontiguousarray(pixels[selected]), self.K, None, rvec, tvec,
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        # A short visible floor patch poorly conditions camera height versus
        # pitch. Retain the measured mounting-height prior while estimating
        # attitude: metric scale must not random-walk when new landmarks are
        # seeded. Small real height changes remain estimable from the images.
        def residual(parameters):
            rotation = cv2.Rodrigues(parameters[:3])[0]
            center = -rotation.T @ parameters[3:]
            world_base_rotation = rotation.T @ self._camera_base[:3, :3]
            projected = cv2.projectPoints(landmarks[selected], parameters[:3],
                parameters[3:], self.K, None)[0].reshape(-1, 2)
            image_error = (projected - pixels[selected]).ravel() / math.sqrt(selected.sum())
            height_prior = (center[2] - self.T_base_camera[2, 3]) / .01
            # A four-wheel platform on the supported flat floor is near level.
            # Weak attitude regularization prevents unobservable floor-normal
            # drift through successively re-seeded short visual patches. It
            # still lets image evidence estimate real chassis rocking.
            attitude_prior = world_base_rotation[2, :2] / .003
            return np.concatenate((image_error, [height_prior], attitude_prior))
        fitted = least_squares(residual, np.concatenate((rvec.ravel(), tvec.ravel())),
            max_nfev=12, ftol=1e-6, xtol=1e-6, gtol=1e-6)
        rvec, tvec = fitted.x[:3], fitted.x[3:]
        camera_world[:3, :3] = cv2.Rodrigues(rvec)[0]
        camera_world[:3, 3] = tvec.ravel()
        candidate = np.linalg.inv(camera_world)
        base = candidate @ self._camera_base
        height = self.T_base_camera[2, 3]
        self.last_camera_fit = {"camera_height_m": float(candidate[2, 3]),
            "base_height_m": float(base[2, 3]), "tilt_degrees": math.degrees(
                math.acos(float(np.clip(base[2, 2], -1., 1.))))}
        if (not ok or not np.all(np.isfinite(candidate))
                or not .65 * height < candidate[2, 3] < 1.35 * height
                or abs(base[2, 3]) > .08 or base[2, 2] < math.cos(math.radians(15))):
            return None, selected, math.inf, "implausible_camera_attitude"
        error = float(np.median(np.linalg.norm(cv2.projectPoints(landmarks[selected],
            rvec, tvec, self.K, None)[0].reshape(-1, 2) - pixels[selected], axis=1)))
        if error > 1.:
            return None, selected, error, "excessive_camera_reprojection_error"
        return candidate, selected, error, "visual_floor_pnp_tracking"

    def _registration_candidates(self, pose, timestamp):
        """Prioritize a previous rotation's local view before distant history.

        Chronological truncation can fill every matching slot with views from
        an earlier station, hiding a much closer anchor acquired on the latest
        rotation. Ranking uses estimated visual poses and exposure ages only;
        stored keyframes remain chronological for creation and eviction.
        """
        nearby = [frame for frame in self._keyframes if timestamp - frame["time"] >= 1.
            and math.dist((pose.x, pose.y), (frame["pose"].x, frame["pose"].y)) < 1.2
            and abs(wrap_angle(pose.yaw - frame["pose"].yaw)) < .65]
        def priority(frame):
            distance = math.dist((pose.x, pose.y), (frame["pose"].x, frame["pose"].y))
            angle = abs(wrap_angle(pose.yaw - frame["pose"].yaw))
            prior_rotation = distance < .15 and timestamp - frame["time"] >= 8.
            return (0 if prior_rotation else 1, angle/.3 + distance/.15, frame["time"])
        return sorted(nearby, key=priority)

    def _map_registration(self, gray, mask, candidate, timestamp):
        if timestamp - self._last_map_check < .5:
            return candidate, False
        self._last_map_check = timestamp
        keypoints, descriptors = self._orb.detectAndCompute(gray, mask)
        if descriptors is None or len(keypoints) < 24:
            return candidate, False
        pixels = np.array([point.pt for point in keypoints], dtype=np.float64)
        pose, registered = self._base_pose(candidate), False
        nearby = self._registration_candidates(pose, timestamp)
        for frame in nearby[:6]:
            pairs = self._matcher.knnMatch(descriptors, frame["descriptors"], k=2)
            good = [pair[0] for pair in pairs if len(pair) == 2
                    and pair[0].distance < .72 * pair[1].distance and pair[0].distance < 58]
            unique = {}
            for match in sorted(good, key=lambda item: item.distance):
                unique.setdefault(match.trainIdx, match)
            matches = list(unique.values())
            if len(matches) < 24:
                continue
            old = np.array([match.trainIdx for match in matches])
            new = np.array([match.queryIdx for match in matches])
            correction, selected, error, _ = self._solve_camera(
                frame["landmarks"][old], pixels[new], candidate, required=24)
            if correction is None or selected.sum() < .65 * len(matches) or error > .9:
                continue
            corrected = self._base_pose(correction)
            if (math.dist((corrected.x, corrected.y), (pose.x, pose.y)) > .12
                    or abs(wrap_angle(corrected.yaw - pose.yaw)) > .12):
                continue
            candidate, pose, registered = correction, corrected, True
            self.relocalizations += 1
            break
        if (not self._keyframes or math.dist((pose.x, pose.y),
                (self._keyframes[-1]["pose"].x, self._keyframes[-1]["pose"].y)) >= .15
                or abs(wrap_angle(pose.yaw - self._keyframes[-1]["pose"].yaw)) >= .3):
            landmarks = self._ground_world(pixels, candidate)
            finite = np.all(np.isfinite(landmarks), axis=1)
            self._keyframes.append({"pose": pose, "time": timestamp,
                "landmarks": landmarks[finite], "descriptors": descriptors[finite]})
            if len(self._keyframes) > 100:
                self._keyframes.pop(1)
        return candidate, registered

    def _seed_reference(self, gray, features, timestamp):
        landmarks = self._ground_world(features.reshape(-1, 2), self._T_world_camera)
        finite = np.all(np.isfinite(landmarks), axis=1)
        self._reference_gray = gray
        self._tracking_points = features[finite]
        self._reference_world = landmarks[finite]
        self._reference_timestamp = timestamp

    def update(self, rgb, timestamp, ground_mask=None, feature_mask=None):
        """Consume RGB and separate patch support from feature-center proof.

        ``ground_mask`` is the broad connected floor hypothesis used for LK
        patch support and validating existing tracks. ``feature_mask`` may
        restrict fresh feature/descriptor centers to directly strong texture.
        Eroding those small texture islands would remove valid corners, so
        only the broad support mask is eroded. Neither mask contains geometry
        or measurements beyond what was classified from the image itself.
        """
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            return self._result(False, reason="invalid_timestamp")
        if self._last_timestamp is not None and timestamp <= self._last_timestamp:
            return self._result(False, reason="nonmonotonic_timestamp")
        self._last_timestamp = timestamp
        rgb = np.asarray(rgb)
        if rgb.shape != (self.height, self.width, 3) or rgb.dtype != np.uint8:
            return self._result(False, reason="invalid_rgb_frame")
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        mask = self._mask(ground_mask)
        seed_mask = mask
        if feature_mask is not None:
            feature_mask = np.asarray(feature_mask)
            if feature_mask.shape != (self.height, self.width):
                raise ValueError("Feature-center mask must match RGB dimensions")
            seed_mask = mask & (feature_mask.astype(bool).astype(np.uint8) * 255)
        fresh = self._features(gray, seed_mask)
        feature_count = len(fresh)
        if self._reference_gray is None:
            if feature_count < self.min_inliers:
                return self._result(False, features=feature_count, reason="insufficient_floor_texture")
            self._seed_reference(gray, fresh, timestamp)
            self._accepted_timestamp = timestamp
            self._map_registration(gray, seed_mask, self._T_world_camera, timestamp)
            return self._result(True, feature_count, feature_count, 0., "initialized_at_arbitrary_origin")
        if not np.any(mask):
            return self._result(False, features=feature_count, reason="insufficient_floor_texture")
        # The reference image and its track locations advance after every
        # accepted exposure, even when no new landmarks can safely be seeded.
        # Landmark age alone is not a camera outage.
        if timestamp - self._accepted_timestamp > self.max_reference_age:
            return self._result(False, features=feature_count, reason="reference_too_old_reset_required")
        previous = self._tracking_points
        # A large translation-only patch spans different perspective motion
        # on a rotating floor and may include a nearby wall in coarse pyramid
        # levels. The smaller measured window retains real image support and
        # reduced systematic yaw bias on frozen actual navigation exposures.
        lk = dict(winSize=(9, 9), maxLevel=2,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 24, .01))
        current, fs, _ = cv2.calcOpticalFlowPyrLK(self._reference_gray, gray, previous, None, **lk)
        backward, bs, _ = cv2.calcOpticalFlowPyrLK(gray, self._reference_gray, current, None, **lk)
        a, b = previous.reshape(-1, 2), current.reshape(-1, 2)
        valid = (fs.ravel().astype(bool) & bs.ravel().astype(bool)
            & np.all(np.isfinite(b), axis=1) & (np.linalg.norm(a-backward.reshape(-1, 2), axis=1) < .8)
            & (b[:, 0] >= 2) & (b[:, 0] < self.width-2) & (b[:, 1] >= 2) & (b[:, 1] < self.height-2))
        indices = np.flatnonzero(valid)
        rounded = np.rint(b[indices]).astype(int)
        valid[indices] &= mask[rounded[:, 1], rounded[:, 0]] != 0
        landmarks, pixels = self._reference_world[valid], b[valid]
        candidate, selected, error, reason = self._solve_camera(landmarks, pixels, self._T_world_camera)
        inliers = int(selected.sum())
        if candidate is None:
            return self._result(False, inliers, feature_count, error, reason)
        pose = self._base_pose(candidate)
        dt = timestamp - self._accepted_timestamp
        if (math.dist((pose.x, pose.y), (self.pose.x, self.pose.y)) > .025 + dt
                or abs(wrap_angle(pose.yaw - self.pose.yaw)) > .03 + 2.5*dt):
            return self._result(False, inliers, feature_count, error, "implausible_visual_motion")
        candidate, registered = self._map_registration(gray, seed_mask, candidate, timestamp)
        self._T_world_camera = candidate
        self.pose = self._base_pose(candidate)
        self._accepted_timestamp = timestamp
        self._reference_gray = gray
        if (feature_count >= self.min_inliers
                and (registered or timestamp-self._reference_timestamp >= .4
                     or inliers < .55*len(previous) or inliers < .55*feature_count)):
            self._seed_reference(gray, fresh, timestamp)
        else:
            self._reference_world = landmarks[selected]
            self._tracking_points = pixels[selected].astype(np.float32).reshape(-1, 1, 2)
        return self._result(True, inliers, feature_count, error, reason)
