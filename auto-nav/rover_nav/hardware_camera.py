"""Paired phone RGB ingestion with explicit camera and timing uncertainty.

Browser frame callbacks report presentation, not sensor exposure. A challenge
therefore provides only a conservative lower bound, conditional on a separately
measured sensor-to-callback latency bound. No browser or network timestamp is
silently treated as an exact exposure time. Uncalibrated streams are preview
only; a source/settings change requires a new ingestion and perception session.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import io
import json
import math
import secrets
import threading
import time

import cv2
import numpy as np
from PIL import Image, UnidentifiedImageError

from .hardware_floor_calibration import floor_patch_visibility
from .vision import RGBPacket

MAX_FRAME_BYTES = 2 * 1024 * 1024


class CameraFrameError(ValueError):
    def __init__(self, code, message=None):
        self.code = code
        super().__init__(message or code.replace("_", " "))


def _number(value, name, *, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(f"{name} is outside its supported range")
    return float(value)


def _dimension(value, name, maximum=8192):
    if isinstance(value, bool) or not isinstance(value, int) or not 32 <= value <= maximum:
        raise ValueError(f"{name} must be an integer from 32 to {maximum}")
    return value


@dataclass(frozen=True)
class CameraCalibration:
    camera_id: str
    source_width: int
    source_height: int
    width: int
    height: int
    K: np.ndarray
    distortion: np.ndarray
    T_base_camera: np.ndarray
    settings_json: str
    verified: bool
    max_pipeline_age_s: float | None
    timing_validated: bool
    calibration_id: str

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or type(value.get("version")) is not int or value.get("version") != 1:
            raise ValueError("Camera calibration version 1 is required")
        camera_id = value.get("camera_id")
        if not isinstance(camera_id, str) or not 1 <= len(camera_id) <= 200:
            raise ValueError("camera_id must identify the measured phone and lens")
        sw = _dimension(value.get("source_width"), "source_width")
        sh = _dimension(value.get("source_height"), "source_height")
        width = _dimension(value.get("width"), "width", 2048)
        height = _dimension(value.get("height"), "height", 2048)
        if sw * sh > 20_000_000 or width * height > 2_000_000:
            raise ValueError("Camera image exceeds the supported pixel count")
        if width * sh != height * sw:
            raise ValueError("Output must retain the full calibrated source aspect ratio; crop is unsupported")
        if width > sw or height > sh:
            raise ValueError("Output cannot upscale the calibrated source")
        K = np.asarray(value.get("K"), dtype=float)
        T = np.asarray(value.get("T_base_camera"), dtype=float)
        distortion = np.asarray(value.get("distortion"), dtype=float)
        if (K.shape != (3, 3) or not np.isfinite(K).all() or
                not np.allclose(K[2], [0, 0, 1], atol=1e-9) or
                abs(K[1, 0]) > 1e-9 or abs(K[0, 1]) > 1e-9 or
                K[0, 0] <= 0 or K[1, 1] <= 0 or
                not 0 <= K[0, 2] < sw or not 0 <= K[1, 2] < sh):
            raise ValueError("K must be measured pinhole intrinsics in source pixels with zero skew")
        if distortion.shape not in {(4,), (5,), (8,)} or not np.isfinite(distortion).all():
            raise ValueError("Measured OpenCV distortion must have 4, 5 or 8 finite coefficients")
        if (T.shape != (4, 4) or not np.isfinite(T).all() or
                not np.allclose(T[3], [0, 0, 0, 1], atol=1e-9) or
                not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5) or
                not np.isclose(np.linalg.det(T[:3, :3]), 1., atol=1e-5) or
                not .03 <= T[2, 3] <= 2.):
            raise ValueError("T_base_camera must be a measured rigid transform above the floor")
        settings = value.get("settings")
        if (not isinstance(settings, dict) or
                not isinstance(settings.get("device_id"), str) or not settings["device_id"] or
                settings.get("width") != sw or settings.get("height") != sh):
            raise ValueError("settings must pin device_id, source width and source height")
        try:
            settings_json = json.dumps(settings, sort_keys=True, separators=(",", ":"), allow_nan=False)
        except (ValueError, TypeError) as error:
            raise ValueError("Camera settings must be finite JSON") from error
        if len(settings_json) > 4096:
            raise ValueError("Camera settings are too large")
        # group_id is document-scoped in Media Capture, so reloading the same
        # lens can change it. Retain it in raw frame diagnostics, but exclude
        # only this non-optical field from persisted equality and fingerprints.
        settings = {name: item for name, item in settings.items() if name != "group_id"}
        settings_json = json.dumps(settings, sort_keys=True, separators=(",", ":"), allow_nan=False)
        timing = value.get("timing", {})
        if not isinstance(timing, dict):
            raise ValueError("Camera timing must be an object")
        bound = timing.get("max_pipeline_age_s")
        if bound is not None:
            bound = _number(bound, "max_pipeline_age_s", minimum=.001, maximum=.25)
        verified = value.get("verified") is True
        timing_validated = timing.get("validated") is True
        if timing_validated and (bound is None or not verified):
            raise ValueError("Validated timing requires a measured bound and verified camera calibration")
        canonical = dict(value)
        canonical.pop("notes", None)
        canonical["settings"] = settings
        fingerprint = hashlib.sha256(json.dumps(canonical, sort_keys=True, allow_nan=False).encode()).hexdigest()
        for array in (K, distortion, T):
            array.setflags(write=False)
        calibration = cls(camera_id, sw, sh, width, height, K, distortion, T,
                          settings_json, verified, bound, timing_validated, fingerprint)
        # A horizontal or slightly upward optical axis can still see enough
        # floor below it. Check the actual rays delivered to FloorVision rather
        # than imposing a tilt angle. Rectification can narrow the view, so a
        # raw JPEG visibility check alone cannot establish this condition.
        final_view = calibration.perception_calibration
        patch = floor_patch_visibility(T, final_view["K"], np.zeros(5), (width, height))
        if not patch["seed_visible"]:
            raise ValueError("Camera cannot observe the required initial floor calibration patch in the final rectified view")
        return calibration

    @property
    def settings(self):
        return json.loads(self.settings_json)

    @property
    def perception_calibration(self):
        # Pixels are resized without cropping before undistortion. OpenCV's
        # remap and the perception model use this same output intrinsic matrix.
        scale = self.width / self.source_width
        K = self.K.copy()
        K[0, 0] *= scale
        K[1, 1] *= scale
        # cv2.resize uses pixel-centre coordinates.
        K[0, 2] = (K[0, 2] + .5) * scale - .5
        K[1, 2] = (K[1, 2] + .5) * scale - .5
        # A virtual pinhole view may narrow the field of view to avoid unknown
        # border pixels after undistortion. Its exact K is passed to perception.
        # This is a known optical transformation, not an unreported phone crop.
        if np.any(self.distortion):
            K, _ = cv2.getOptimalNewCameraMatrix(K, self.distortion,
                (self.width, self.height), 0., (self.width, self.height))
            source_K = self.K.copy()
            source_K[0, 0] *= scale
            source_K[1, 1] *= scale
            source_K[0, 2] = (source_K[0, 2] + .5) * scale - .5
            source_K[1, 2] = (source_K[1, 2] + .5) * scale - .5
            # Pixel-centre rounding in OpenCV's alpha=0 ROI can leave a fraction
            # of an invalid edge. Tighten the virtual field of view until every
            # delivered pixel has a real source pixel, or reject the profile.
            for _ in range(100):
                mx, my = cv2.initUndistortRectifyMap(source_K, self.distortion, None, K,
                    (self.width, self.height), cv2.CV_32FC1)
                if np.isfinite(mx).all() and np.isfinite(my).all() and mx.min() >= -1e-5 and my.min() >= -1e-5 and mx.max() <= self.width - 1 + 1e-5 and my.max() <= self.height - 1 + 1e-5:
                    break
                K[0, 0] *= 1.01
                K[1, 1] *= 1.01
            else:
                raise ValueError("Distortion profile cannot produce a valid bounded pinhole image")
        return {"width": self.width, "height": self.height, "K": K.tolist(),
                "T_base_camera": self.T_base_camera.tolist(),
                "source": "measured_phone_calibration_undistorted_full_frame"}

    def public_descriptor(self):
        return {"calibration_id": self.calibration_id, "camera_id": self.camera_id,
                "source_width": self.source_width, "source_height": self.source_height,
                "width": self.width, "height": self.height, "settings": self.settings,
                "verified": self.verified, "timing_validated": self.timing_validated,
                "max_pipeline_age_s": self.max_pipeline_age_s,
                "rotation_degrees": 0, "mirrored": False, "crop": "none",
                "undistortion": "known_pinhole_transform_with_valid_source_pixels"}


class CameraIngest:
    def __init__(self, calibration=None, *, max_frame_age_s=.35, challenge_timeout_s=.30,
                 clock=time.monotonic):
        if calibration is not None and not isinstance(calibration, CameraCalibration):
            calibration = CameraCalibration.from_dict(calibration)
        self.calibration = calibration
        self.clock = clock
        self.max_frame_age_s = _number(max_frame_age_s, "max_frame_age_s", minimum=.05, maximum=.5)
        self.challenge_timeout_s = _number(challenge_timeout_s, "challenge_timeout_s", minimum=.02, maximum=.5)
        self.lock = threading.RLock()
        self._challenge = None
        self._packet = None
        self._jpeg = None
        self._metadata = None
        self._received_at = None
        self._stream_id = None
        self._last_seq = self._last_presented = -1
        self._last_media = -1.
        self._last_timestamp = None
        self._consumed = True
        self._reset_required = False
        self._accepted = self._rejected = 0
        self._reason = "awaiting_phone_frame"
        self._map1 = self._map2 = self._valid_pixels = None
        if calibration is not None:
            K = calibration.K.copy()
            scale = calibration.width / calibration.source_width
            K[0, 0] *= scale
            K[1, 1] *= scale
            K[0, 2] = (K[0, 2] + .5) * scale - .5
            K[1, 2] = (K[1, 2] + .5) * scale - .5
            new_K = np.asarray(calibration.perception_calibration["K"])
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                K, calibration.distortion, None, new_K,
                (calibration.width, calibration.height), cv2.CV_32FC1)
            # Do not make black border pixels into false obstacle/floor evidence.
            self._valid_pixels = (np.isfinite(self._map1) & np.isfinite(self._map2) &
                                  (self._map1 >= -1e-5) & (self._map1 <= calibration.width - 1 + 1e-5) &
                                  (self._map2 >= -1e-5) & (self._map2 <= calibration.height - 1 + 1e-5))
            self._map1 = np.clip(self._map1, 0., calibration.width - 1.)
            self._map2 = np.clip(self._map2, 0., calibration.height - 1.)

    def _now(self, now):
        return _number(self.clock() if now is None else now, "host monotonic time", minimum=0.)

    def challenge(self, now=None):
        with self.lock:
            now = self._now(now)
            if self._reset_required:
                raise CameraFrameError("camera_session_reset_required")
            if self._challenge and now <= self._challenge["expires_at"]:
                return dict(self._challenge)
            self._challenge = {"challenge_id": secrets.token_hex(16), "issued_at": now,
                               "expires_at": now + self.challenge_timeout_s,
                               "calibration_id": self.calibration.calibration_id if self.calibration else None,
                               "width": self.calibration.width if self.calibration else None,
                               "height": self.calibration.height if self.calibration else None,
                               "preview_max_edge": 640}
            return dict(self._challenge)

    def _reject(self, code, *, reset=False):
        self._rejected += 1
        self._reason = code
        self._packet = None
        self._consumed = True
        self._reset_required |= reset
        raise CameraFrameError(code)

    def ingest(self, jpeg, metadata, now=None):
        with self.lock:
            now = self._now(now)
            if self._reset_required:
                self._reject("camera_session_reset_required")
            challenge = self._challenge
            # A challenge is single-use even when its upload is malformed.
            self._challenge = None
            if not isinstance(metadata, dict):
                self._reject("invalid_frame_metadata")
            try:
                canonical = json.dumps(metadata, allow_nan=False, separators=(",", ":"))
                if len(canonical) > 16384:
                    raise ValueError()
                # Own the metadata, including nested settings, instead of
                # retaining caller-mutable objects in status snapshots.
                metadata = json.loads(canonical)
            except (ValueError, TypeError, RecursionError):
                self._reject("invalid_frame_metadata")
            if not challenge or metadata.get("challenge_id") != challenge["challenge_id"]:
                self._reject("unknown_or_replayed_challenge")
            if now < challenge["issued_at"] or now > challenge["expires_at"]:
                self._reject("stale_frame_challenge")
            stream = metadata.get("stream_id")
            if not isinstance(stream, str) or not 1 <= len(stream) <= 128:
                self._reject("invalid_stream_id")
            if self._stream_id is not None and stream != self._stream_id:
                self._reject("camera_source_changed_reset_required", reset=True)
            try:
                sequence = metadata["sequence"]
                presented = metadata["presented_frames"]
                media = _number(metadata["media_time"], "media_time", minimum=0.)
                if (isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0 or
                        isinstance(presented, bool) or not isinstance(presented, int) or presented < 1):
                    raise ValueError()
                sw = _dimension(metadata["source_width"], "source_width")
                sh = _dimension(metadata["source_height"], "source_height")
                width = _dimension(metadata["width"], "width", 2048)
                height = _dimension(metadata["height"], "height", 2048)
                if width * height > 2_000_000 or width * sh != height * sw:
                    raise ValueError()
                if metadata.get("rotation_degrees") != 0 or metadata.get("mirrored") is not False or metadata.get("crop") != "none":
                    raise ValueError()
            except (KeyError, ValueError, TypeError):
                self._reject("invalid_frame_geometry_or_timing", reset=self._stream_id is not None)
            if sequence <= self._last_seq or presented <= self._last_presented or media <= self._last_media:
                self._reject("repeated_or_out_of_order_frame")
            calibration = self.calibration
            if calibration is not None:
                if (metadata.get("calibration_id") != calibration.calibration_id or
                        (sw, sh, width, height) != (calibration.source_width, calibration.source_height,
                                                  calibration.width, calibration.height)):
                    self._reject("camera_geometry_changed_reset_required", reset=True)
                settings = metadata.get("settings")
                if (not isinstance(settings, dict) or
                        {name: item for name, item in settings.items() if name != "group_id"} != calibration.settings):
                    self._reject("camera_settings_changed_reset_required", reset=True)
            elif max(width, height) > 640:
                self._reject("preview_resolution_exceeded")
            if not isinstance(jpeg, bytes) or not 0 < len(jpeg) <= MAX_FRAME_BYTES:
                self._reject("invalid_frame_size")
            try:
                picture = Image.open(io.BytesIO(jpeg))
                if picture.format != "JPEG" or picture.size != (width, height):
                    self._reject("jpeg_geometry_mismatch")
                if picture.getexif().get(274, 1) != 1:
                    self._reject("jpeg_orientation_not_canonical")
                picture.load()
                pixels = np.asarray(picture.convert("RGB"))
            except CameraFrameError:
                raise
            except (UnidentifiedImageError, OSError, ValueError, Image.DecompressionBombError):
                self._reject("invalid_jpeg")
            bound = calibration.max_pipeline_age_s if calibration else None
            # This is a conservative acquisition-time LOWER BOUND, never an
            # exact exposure. Unknown browser pipeline age permits preview only.
            timestamp = challenge["issued_at"] - bound if bound is not None else None
            if timestamp is not None and now - timestamp > self.max_frame_age_s:
                self._reject("frame_age_bound_exceeded")
            if timestamp is not None and self._last_timestamp is not None and timestamp <= self._last_timestamp:
                self._reject("nonmonotonic_host_frame_bound")
            if calibration is not None and calibration.verified and timestamp is not None:
                pixels = cv2.remap(pixels, self._map1, self._map2, interpolation=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT)
                # Out-of-image undistortion borders cannot be interpreted by
                # the frozen vision model. Reject instead of inventing pixels.
                if not bool(self._valid_pixels.all()):
                    self._reject("undistortion_requires_valid_full_frame_calibration")
                pixels = np.array(pixels, dtype=np.uint8, copy=True)
                pixels.setflags(write=False)
                self._packet = RGBPacket(pixels, timestamp)
                self._consumed = False
            else:
                self._packet = None
                self._consumed = True
            self._jpeg, self._metadata = jpeg, dict(metadata)
            self._received_at = now
            self._stream_id = stream
            self._last_seq, self._last_presented, self._last_media = sequence, presented, media
            self._last_timestamp = timestamp
            self._accepted += 1
            self._reason = ("frame_ready" if self._packet is not None else
                            "calibration_required" if calibration is None or not calibration.verified else
                            "camera_pipeline_latency_bound_required")
            return self.status(now)

    def read(self, now=None):
        with self.lock:
            now = self._now(now)
            if self._reset_required:
                raise CameraFrameError("camera_session_reset_required")
            if self._packet is None:
                raise CameraFrameError(self._reason)
            if now - self._packet.timestamp > self.max_frame_age_s or now < self._packet.timestamp:
                self._packet = None
                self._consumed = True
                self._reason = "stale_camera_frame"
                raise CameraFrameError(self._reason)
            if self._consumed:
                raise CameraFrameError("no_new_camera_frame")
            self._consumed = True
            return self._packet

    def latest_image(self):
        with self.lock:
            return self._jpeg

    def reset(self):
        """Explicitly start a camera session; caller must also reset VO/map.

        No old exposure, challenge, source generation or preview survives. This
        operation does not acquire or command rover motors.
        """
        with self.lock:
            self._challenge = self._packet = self._jpeg = self._metadata = None
            self._received_at = self._stream_id = self._last_timestamp = None
            self._last_seq = self._last_presented = -1
            self._last_media = -1.
            self._consumed = True
            self._reset_required = False
            self._reason = "awaiting_phone_frame_after_session_reset"

    def status(self, now=None):
        with self.lock:
            now = self._now(now)
            age = None if self._last_timestamp is None else now - self._last_timestamp
            preview_age = None if self._received_at is None else now - self._received_at
            calibration = self.calibration
            fresh = self._packet is not None and age is not None and 0 <= age <= self.max_frame_age_s
            return {"accepted_frames": self._accepted, "rejected_frames": self._rejected,
                    "reason": self._reason, "stream_id": self._stream_id,
                    "sequence": self._last_seq, "frame_age_upper_bound_s": age,
                    "preview_age_s": preview_age, "fresh": fresh,
                    "new_frame": fresh and not self._consumed, "reset_required": self._reset_required,
                    "calibration_verified": bool(calibration and calibration.verified),
                    "hardware_timing_validated": bool(calibration and calibration.timing_validated),
                    "live_ready": bool(fresh and calibration and calibration.verified and calibration.timing_validated and not self._reset_required),
                    "timestamp_source": "browser_challenge_lower_bound_not_exact_exposure",
                    "timestamp_bound_conditional_on_measured_pipeline_limit": True,
                    "descriptor": calibration.public_descriptor() if calibration else None,
                    "source_metadata": dict(self._metadata) if self._metadata else None}
