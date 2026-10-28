"""Conservative monocular free-floor evidence from RGB and fixed calibration.

This is an appearance-based flat-floor sensor, not general semantic depth.
Its explicit prior is a textured, approximately single-chromaticity floor with
a bounded brightness envelope learned in
the initial 0.3--0.7 m central calibration patch, with obstacles distinguishable
from that appearance. Metric scale comes only from the fixed camera height.
Uniform, badly exposed or chromatically ambiguous calibration images fail
closed. A floor-coloured *uniform* obstacle is rejected by the texture gate;
an obstacle that copies the floor's texture remains outside this sensor's
supported domain and requires temporal 3-D or another sensor to disambiguate.

Only image-observed intervals are returned. Their near endpoints are explicit
because the camera cannot see the floor immediately beneath/in front of it.
Texture interruptions separate disjoint radial intervals; their gaps remain
unknown. A positive appearance obstacle blocks all farther floor evidence on
that bearing. The output must never be interpreted as a clear ray from the
camera origin or as permission to interpolate across an unmeasured gap.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class FloorResult:
    mask: np.ndarray
    angles: np.ndarray
    ranges: np.ndarray
    min_ranges: np.ndarray
    hits: np.ndarray
    confidence: float
    valid: bool
    reason: str
    free_points: np.ndarray
    tracking_mask: np.ndarray | None = None
    hit_ranges: np.ndarray | None = None
    feature_mask: np.ndarray | None = None
    free_intervals: tuple[tuple[tuple[float, float], ...], ...] | None = None


class FloorVision:
    """Learn a bounded floor-appearance prior and project its visible boundary.

    ``T_base_camera`` maps OpenCV (right, down, forward) coordinates into base
    (forward, left, up) coordinates with z=0 at the flat floor. ``angles`` and
    distances are relative to its fixed camera XY centre, in the base frame.
    ``hits`` means independently visible positive non-floor appearance evidence,
    even when no floor interval is established. Texture absence or
    an image-edge limit only truncates free evidence and stays unknown. NaN
    ranges mean there was no usable visible floor interval. ``ranges`` ends
    guaranteed-free evidence; ``hit_ranges`` independently locates the original
    image appearance boundary. Never mark the retreated free endpoint occupied:
    its gap to a positive contact expresses measurement uncertainty.
    ``tracking_mask`` supports optical-flow patches; ``feature_mask`` selects
    directly textured feature centres and must not itself be eroded.
    ``free_intervals`` contains ordered disjoint positive-length intervals per
    bearing; empty entries mean unknown. The legacy ``min_ranges``/``ranges``
    expose only the first interval, or a zero-length positive-hit placeholder.
    """

    def __init__(self, K, T_base_camera, width: int, height: int,
                 *, max_range: float = 2.5, bearings: int = 101):
        self.K = np.array(K, dtype=float, copy=True)
        self.T = np.array(T_base_camera, dtype=float, copy=True)
        self.width, self.height = int(width), int(height)
        self.max_range = float(max_range)
        if (self.K.shape != (3, 3) or self.T.shape != (4, 4)
                or not np.all(np.isfinite(self.K)) or not np.all(np.isfinite(self.T))
                or self.width < 32 or self.height < 32 or self.K[0, 0] <= 0
                or self.K[1, 1] <= 0 or not .5 <= self.max_range <= 4.):
            raise ValueError("Invalid camera calibration or floor sensor dimensions")
        self._bearings = max(21, int(bearings))
        self.set_camera_transform(self.T)
        self.reset()

    def set_camera_transform(self, T_level_camera):
        """Update image-estimated camera tilt/height without relearning color.

        The transform maps OpenCV camera coordinates to a level frame whose
        XY origin is the estimated base position, +X is its estimated heading,
        and z=0 is the floor. It may come from image-only planar PnP; no scene
        state, inertial data or simulator transform is used by this module.
        """
        transform = np.array(T_level_camera, dtype=float, copy=True)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError("Camera transform must be a finite 4x4 matrix")
        rotation = transform[:3, :3]
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)
                or not np.allclose(transform[3], [0., 0., 0., 1.]) or transform[2, 3] <= 0):
            raise ValueError("Camera must be rigidly calibrated above the flat floor")
        if hasattr(self, "ground") and np.array_equal(transform, self.T):
            return
        self.T = transform
        self._inverse = np.linalg.inv(self.T)
        self.origin = self.T[:2, 3].copy()
        yy, xx = np.mgrid[:self.height, :self.width]
        homogeneous = np.column_stack((xx.ravel(), yy.ravel(), np.ones(xx.size)))
        rays = homogeneous @ np.linalg.inv(self.K).T @ rotation.T
        scale = np.full(xx.size, np.nan)
        downward = rays[:, 2] < -1e-5
        scale[downward] = -self.T[2, 3] / rays[downward, 2]
        self.ground = (self.origin + rays[:, :2] * scale[:, None]).reshape(
            self.height, self.width, 2)
        relative = self.ground - self.origin
        distance = np.linalg.norm(relative, axis=2)
        angle = np.arctan2(relative[:, :, 1], relative[:, :, 0])
        self._geometric = (np.isfinite(distance) & (distance <= self.max_range + .1)
                           & (distance >= .06) & (relative[:, :, 0] > 0))
        # Texture evidence is evaluated over a fixed *metric* neighbourhood.
        # Speckled carpet contains smooth gaps that occupy many near pixels;
        # a constant pixel-sized texture gate would misclassify those gaps.
        self._texture_resolution = .015
        self._texture_support_radius = .095
        self._texture_shape = (int(math.ceil(2 * (self.max_range + .15)
                                             / self._texture_resolution)),) * 2
        safe_ground = np.where(np.isfinite(relative), relative, 0.)
        self._texture_cells = np.floor((safe_ground + self.max_range + .15)
                                        / self._texture_resolution).astype(int)
        self._texture_cells = np.clip(self._texture_cells, 0, self._texture_shape[0] - 1)
        self._seed = (self._geometric & (distance >= .3) & (distance <= .7)
                      & (np.abs(angle) < .32))
        self._connection_seed = self._seed | (self._geometric & (yy >= .75 * self.height)
                                              & (distance <= .6) & (np.abs(angle) < .75))
        if self._seed.sum() < 100 and getattr(self, "_appearance", None) is None:
            raise ValueError("Camera cannot observe the required initial floor calibration patch")
        # Sample radial ground points, then project them into the actual image.
        # No scene geometry or image-derived absolute pose enters this operation.
        geometric_angles = angle[self._geometric]
        low = max(-1.2, float(np.quantile(geometric_angles, .002)))
        high = min(1.2, float(np.quantile(geometric_angles, .998)))
        self.angles = np.linspace(low, high, self._bearings)
        self._distances = np.arange(.025, self.max_range + .0125, .025)
        a, r = self.angles[:, None], self._distances[None, :]
        self._points = self.origin + np.stack((np.cos(a) * r, np.sin(a) * r), axis=2)
        points = np.column_stack((self._points.reshape(-1, 2),
                                  np.zeros(self._points.size // 2),
                                  np.ones(self._points.size // 2)))
        camera = (points @ self._inverse.T)[:, :3]
        image = camera @ self.K.T
        pixels = image[:, :2] / np.maximum(image[:, 2:3], 1e-12)
        pixels = pixels.reshape(len(self.angles), -1, 2)
        self._pixels = np.rint(pixels).astype(int)
        self._visible = ((camera[:, 2] > 0).reshape(len(self.angles), -1)
            & (pixels[:, :, 0] >= 3) & (pixels[:, :, 0] < self.width - 3)
            & (pixels[:, :, 1] >= 3) & (pixels[:, :, 1] < self.height - 3))

    def reset(self):
        self._appearance = None
        self._floor_brightness = None
        self._floor_brightness_ratio = None

    def _failure(self, reason, confidence=0., tracking_mask=None, feature_mask=None,
                 *, hits=None, hit_ranges=None):
        count = len(self.angles)
        return FloorResult(np.zeros((self.height, self.width), bool), self.angles.copy(),
            np.full(count, np.nan), np.full(count, np.nan),
            np.zeros(count, bool) if hits is None else hits.copy(),
            float(confidence), False, reason, np.empty((0, 2)),
            None if tracking_mask is None else tracking_mask.copy(),
            np.full(count, np.nan) if hit_ranges is None else hit_ranges.copy(),
            None if feature_mask is None else feature_mask.copy(),
            tuple(() for _ in range(count)))

    def _positive_contacts(self, positive_obstacle):
        """Keep visible positive evidence independently of floor connectivity.

        Texture uncertainty earlier on a bearing cannot make a later visible
        non-floor surface disappear. Mapping may have no free interval while
        the image still supplies a positive surface contact for stopping.
        """
        hits = np.zeros(len(self.angles), bool)
        hit_ranges = np.full(len(self.angles), np.nan)
        for index in range(len(self.angles)):
            visible = np.flatnonzero(self._visible[index])
            pixels = self._pixels[index, visible]
            positive = np.flatnonzero(positive_obstacle[pixels[:, 1], pixels[:, 0]])
            if len(positive):
                hits[index] = True
                hit_ranges[index] = self._distances[visible[positive[0]]]
        return hits, hit_ranges

    def _clearance_exclusions(self, uncertain_points, candidate_points, required):
        """Exactly match nearest-point erosion, filtering with safe EDT bounds.

        Each point is at most half a grid-cell diagonal from its cell centre.
        The nearest-centre distance therefore differs from the actual nearest
        point distance by at most one full diagonal. Only candidates whose
        threshold lies inside that interval need an exact nearest-point query.
        """
        if not len(candidate_points):
            return np.empty(0, bool)
        if not len(uncertain_points):
            return np.zeros(len(candidate_points), bool)
        resolution = self._texture_resolution
        shift = self.max_range + .15
        unknown_cells = np.floor((uncertain_points - self.origin + shift) / resolution).astype(int)
        candidate_cells = np.floor((candidate_points - self.origin + shift) / resolution).astype(int)
        size = np.asarray(self._texture_shape)
        # Bounds require an actual containing cell. Never clip a point into a
        # different cell when a caller supplies coordinates outside this grid.
        if (np.any(unknown_cells < 0) or np.any(unknown_cells >= size)
                or np.any(candidate_cells < 0) or np.any(candidate_cells >= size)):
            distance, _ = cKDTree(uncertain_points).query(candidate_points, workers=1)
            return distance < required
        occupied = np.zeros(self._texture_shape, bool)
        occupied[unknown_cells[:, 1], unknown_cells[:, 0]] = True
        grid_distance = ndimage.distance_transform_edt(~occupied, sampling=resolution)
        distance = grid_distance[candidate_cells[:, 1], candidate_cells[:, 0]]
        # Tiny roundoff slack sends a threshold equality to the exact path.
        error = math.sqrt(2.) * resolution + 1e-12
        lower, upper = distance - error, distance + error
        excluded = upper < required
        ambiguous = ~(excluded | (lower >= required))
        if np.any(ambiguous):
            exact, _ = cKDTree(uncertain_points).query(candidate_points[ambiguous], workers=1)
            excluded[ambiguous] = exact < required[ambiguous]
        return excluded

    @staticmethod
    def _photometry(rgb):
        floating = rgb.astype(np.float32) / 255.
        mean = cv2.GaussianBlur(floating, (5, 5), 0)
        chroma = mean[:, :, :2] / np.maximum(mean.sum(axis=2, keepdims=True), .025)
        gray = cv2.cvtColor(floating, cv2.COLOR_RGB2GRAY)
        local_mean = cv2.boxFilter(gray, -1, (11, 11))
        variance = np.maximum(0., cv2.boxFilter(gray * gray, -1, (11, 11)) - local_mean ** 2)
        # Normalization tolerates exposure changes and diffuse shadows.
        texture = np.sqrt(variance) / np.maximum(local_mean, .06)
        return gray, chroma, texture

    def process(self, rgb) -> FloorResult:
        rgb = np.asarray(rgb)
        if rgb.shape != (self.height, self.width, 3) or rgb.dtype != np.uint8:
            return self._failure("invalid_rgb_frame")
        gray, chroma, texture = self._photometry(rgb)
        if self._appearance is None:
            seed_gray, seed_chroma = gray[self._seed], chroma[self._seed]
            brightness = float(np.median(seed_gray))
            spread = np.quantile(seed_chroma, .95, axis=0) - np.quantile(seed_chroma, .05, axis=0)
            seed_texture = float(np.median(texture[self._seed]))
            features = cv2.goodFeaturesToTrack((gray * 255).astype(np.uint8), 100,
                .015, 5, mask=self._seed.astype(np.uint8) * 255)
            if brightness < .06 or brightness > .94:
                return self._failure("floor_calibration_bad_exposure")
            if features is None or len(features) < 12 or seed_texture < .018:
                return self._failure("insufficient_floor_texture")
            if np.max(spread) > .13:
                return self._failure("ambiguous_floor_chromaticity")
            centre = np.median(seed_chroma, axis=0)
            tolerance = np.maximum(.035, spread * 1.25)
            self._appearance = (centre, tolerance, max(.009, min(.035, seed_texture * .20)))
            self._floor_brightness = brightness
            # Learn the floor's upper reflectance envelope from its clear
            # calibration patch. A 50% allowance above its 99th percentile
            # tolerates sampled texture/illumination variation; four 8-bit
            # intensity levels below account for quantization at low exposure.
            # Only an independently textured floor median rescales exposure.
            self._floor_brightness_ratio = max(1.5,
                float(np.quantile(seed_gray, .99)) / brightness * 1.5)
        centre, tolerance, texture_threshold = self._appearance
        colour_match = np.all(np.abs(chroma - centre) <= tolerance, axis=2)
        well_exposed = (gray > .025) & (gray < .98)
        # Local variance alone accepts a smoothly shaded wall or its straight
        # contact edge as "texture". Require two-axis spatial variation instead:
        # the smaller structure-tensor eigenvalue is near zero for gradients
        # and isolated straight edges, but positive for ordinary carpet marks.
        local_mean = cv2.boxFilter(gray, -1, (7, 7))
        structure = cv2.cornerEigenValsAndVecs(gray, 5, 3)
        first, second = structure[:, :, 0], structure[:, :, 1]
        corner_texture = np.sqrt(np.maximum(second, 0.))
        corner_texture /= np.maximum(local_mean, .06)
        strong_texture = ((texture >= texture_threshold) & (corner_texture >= .006)
                          & (second >= .01 * np.maximum(first, 1e-12)))
        # Smooth wall shading has variance but no independent two-axis carpet
        # texture. Never let it adapt the floor brightness and erase itself
        # from positive appearance evidence on subsequent frames.
        exposed_seed = self._seed & colour_match & strong_texture & well_exposed
        if exposed_seed.sum() >= 100:
            self._floor_brightness = float(np.median(gray[exposed_seed]))
        # Deep shadows remain unknown: darkness is not positive evidence.
        brightness_limit = self._floor_brightness * self._floor_brightness_ratio + 4. / 255.
        bright_surface = gray > brightness_limit
        positive_obstacle = ~colour_match | bright_surface
        hits, hit_ranges = self._positive_contacts(positive_obstacle)
        appearance_match = self._geometric & ~positive_obstacle & well_exposed
        textured = appearance_match & strong_texture
        texture_grid = np.zeros(self._texture_shape, bool)
        cells = self._texture_cells[textured]
        if len(cells):
            texture_grid[cells[:, 1], cells[:, 0]] = True
        distance_to_texture = ndimage.distance_transform_edt(~texture_grid) * self._texture_resolution
        cells = self._texture_cells
        texture_support = (distance_to_texture[cells[:, :, 1], cells[:, :, 0]]
                            <= self._texture_support_radius)
        candidate = appearance_match & texture_support
        # A few individual texture marks/colour outliers do not puncture the
        # floor; disconnected regions and uniform obstacle interiors stay out.
        candidate = cv2.morphologyEx(candidate.astype(np.uint8), cv2.MORPH_CLOSE,
                                      np.ones((3, 3), np.uint8)).astype(bool)
        # Tracking receives hypotheses, independently checked by optical flow
        # and a rigid planar RANSAC fit. Collision mapping receives the more
        # conservative eroded evidence below. Eroding feature hypotheses by a
        # robot safety margin would discard useful, genuinely visible floor
        # corners beside furniture without improving visual motion estimation.
        _, tracking_labels = cv2.connectedComponents(candidate.astype(np.uint8), connectivity=8)
        tracking_seeds = np.unique(tracking_labels[self._connection_seed & candidate])
        tracking_seeds = tracking_seeds[tracking_seeds > 0]
        raw_floor_mask = np.isin(tracking_labels, tracking_seeds) & self._geometric
        # Patch support and feature centres serve different purposes. Fixed
        # metric islands become smaller than a feature window on distant floor.
        # Keep a broad floor ROI for optical-flow patches, but select fresh
        # feature centres only where direct strong two-axis texture is present.
        tracking_mask = raw_floor_mask
        feature_mask = raw_floor_mask & textured
        # Texture support is an isotropic expansion on the floor plane. Undo
        # that uncertainty in the same metric plane before claiming free
        # evidence. Subtracting a constant along each bearing is insufficient
        # at grazing incidence: a 7.5 cm normal expansion can become metres
        # along an almost wall-parallel ray.
        uncertain_mask = self._geometric & ~candidate
        uncertain_points = self.ground[uncertain_mask]
        candidate_pixels = np.flatnonzero(candidate & self._geometric)
        if len(uncertain_points) and len(candidate_pixels):
            ground_points = self.ground.reshape(-1, 2)[candidate_pixels]
            ranges = np.linalg.norm(ground_points - self.origin, axis=1)
            required = (self._texture_support_radius + 2 * self._texture_resolution
                        + .020 * ranges ** 2)
            excluded = self._clearance_exclusions(uncertain_points, ground_points, required)
            excluded_pixels = candidate_pixels[excluded]
            candidate.ravel()[excluded_pixels] = False
        components, labels = cv2.connectedComponents(candidate.astype(np.uint8), connectivity=8)
        seeds = np.unique(labels[self._connection_seed & candidate])
        seeds = seeds[seeds > 0]
        if not len(seeds):
            return self._failure("no_connected_floor_evidence", tracking_mask=tracking_mask,
                                 feature_mask=feature_mask, hits=hits, hit_ranges=hit_ranges)
        mask = np.isin(labels, seeds) & self._geometric
        feature_centres = (cv2.erode(tracking_mask.astype(np.uint8), np.ones((5, 5), np.uint8))
                           & feature_mask.astype(np.uint8))
        features = cv2.goodFeaturesToTrack((gray * 255).astype(np.uint8), 160, .01, 6,
                                          mask=feature_centres * 255)
        feature_count = 0 if features is None else len(features)
        if feature_count < 12 or mask.sum() < 250:
            return self._failure("insufficient_connected_floor_texture", tracking_mask=tracking_mask,
                                 feature_mask=feature_mask, hits=hits, hit_ranges=hit_ranges)
        count = len(self.angles)
        ranges, near = np.full(count, np.nan), np.full(count, np.nan)
        free_points = []
        free_intervals = []
        for index in range(count):
            visible = np.flatnonzero(self._visible[index])
            if not len(visible):
                free_intervals.append(())
                continue
            pixels = self._pixels[index, visible]
            floor = mask[pixels[:, 1], pixels[:, 0]]
            positive = positive_obstacle[pixels[:, 1], pixels[:, 0]]
            # Texture uncertainty can separate multiple independently visible
            # floor segments. Keep every segment, never their convex hull.
            # Positive appearance is different: it occludes this bearing and
            # blocks ALL farther intervals, even if floor remains visible.
            positive_indices = np.flatnonzero(positive)
            limit = int(positive_indices[0]) if len(positive_indices) else len(visible)
            established = np.flatnonzero(floor[:limit])
            splits = np.flatnonzero(np.diff(established) > 1) + 1
            segments = []
            for run in np.split(established, splits):
                if not len(run):
                    continue
                first, stop = int(run[0]), int(run[-1]) + 1
                minimum = float(self._distances[visible[first]])
                endpoint = float(self._distances[visible[stop - 1]])
                if stop < len(visible) or endpoint < self.max_range - .0125:
                    # Retreat each independent segment for pixel footprint
                    # and quantization uncertainty; never extend an endpoint.
                    endpoint = max(minimum, endpoint - (.025 + .012 * endpoint ** 2))
                selected = visible[first:stop]
                selected = selected[self._distances[selected] <= endpoint]
                if len(selected):
                    # End at an actual measured sample, with no free tail.
                    endpoint = float(self._distances[selected[-1]])
                if endpoint > minimum:
                    segments.append((minimum, endpoint))
                    free_points.append(self._points[index, selected])
            free_intervals.append(tuple(segments))
            if segments:
                near[index], ranges[index] = segments[0]
            elif hits[index]:
                # Compatibility only: no free interval is implied by this
                # zero-length placeholder; hit_ranges locates the contact.
                near[index] = self._distances[visible[0]]
                ranges[index] = near[index]
        usable = np.isfinite(ranges) & np.isfinite(near)
        confidence = min(1., feature_count / 40.) * min(1., float(mask.sum()) / 2500.)
        if usable.sum() < 10:
            return self._failure("insufficient_visible_bearings", confidence, tracking_mask, feature_mask,
                                 hits=hits, hit_ranges=hit_ranges)
        return FloorResult(mask, self.angles.copy(), ranges, near, hits,
            float(confidence), True, "appearance_bounded_flat_floor",
            np.concatenate(free_points) if free_points else np.empty((0, 2)),
            tracking_mask, hit_ranges, feature_mask, tuple(free_intervals))
