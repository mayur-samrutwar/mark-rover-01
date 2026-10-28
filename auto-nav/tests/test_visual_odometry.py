"""Synthetic image tests with known motion, independent of the simulator.

The renderer consumes test poses; VisualOdometry receives only the resulting
RGB arrays and static camera calibration. Texture is irregular, not fiducials.
"""
import inspect
import math

import cv2
import numpy as np
import pytest

from rover_nav.navigation import Pose, wrap_angle
from rover_nav.visual_odometry import VisualOdometry


WIDTH, HEIGHT = 320, 240
K = np.array([[230., 0., 159.5], [0., 230., 119.5], [0., 0., 1.]])
T = np.eye(4)
pitch = math.radians(18)
T[:3, :3] = [[0., -math.sin(pitch), math.cos(pitch)],
             [-1., 0., 0.], [0., -math.cos(pitch), -math.sin(pitch)]]
T[:3, 3] = [.055, .012, .21]


class FloorImages:
    def __init__(self):
        rng = np.random.default_rng(872)
        # 10m square natural random floor. Spatial filtering prevents aliasing.
        noise = rng.integers(0, 256, (2048, 2048), dtype=np.uint8)
        self.texture = cv2.GaussianBlur(noise, (3, 3), .65)
        self.projector = VisualOdometry(K, T, WIDTH, HEIGHT)
        yy, xx = np.mgrid[:HEIGHT, :WIDTH]
        self.ground = self.projector.pixels_to_ground(np.column_stack((xx.ravel(), yy.ravel())))
        self.valid = np.all(np.isfinite(self.ground), axis=1).reshape(HEIGHT, WIDTH)

    def frame(self, pose):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        points = self.ground @ np.array([[c, s], [-s, c]]) + [pose.x, pose.y]
        coords = (points * 204.8 + 1024.).astype(np.float32)
        coords[~np.isfinite(coords)] = -1
        gray = cv2.remap(self.texture, coords[:, 0].reshape(HEIGHT, WIDTH),
                         coords[:, 1].reshape(HEIGHT, WIDTH), cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=128)
        gray[~self.valid] = 128
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    def frame_camera(self, world_camera):
        """Independent pinhole renderer with explicit camera roll/pitch/height."""
        yy, xx = np.mgrid[:HEIGHT, :WIDTH]
        rays = np.column_stack((xx.ravel(), yy.ravel(), np.ones(WIDTH*HEIGHT))) @ np.linalg.inv(K).T
        rays = rays @ world_camera[:3, :3].T
        down = rays[:, 2] < -1e-6
        scales = np.full(len(rays), np.nan)
        scales[down] = -world_camera[2, 3] / rays[down, 2]
        points = world_camera[:3, 3] + scales[:, None] * rays
        coords = (points[:, :2] * 204.8 + 1024.).astype(np.float32)
        coords[~np.isfinite(coords)] = -1
        gray = cv2.remap(self.texture, coords[:, 0].reshape(HEIGHT, WIDTH),
            coords[:, 1].reshape(HEIGHT, WIDTH), cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=128)
        gray[~down.reshape(HEIGHT, WIDTH)] = 128
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)


@pytest.fixture(scope="module")
def floor():
    return FloorImages()


def test_calibrated_pixel_rays_have_metric_scale_and_camera_offset():
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    ground = np.array([[.2, -.1], [.5, .1], [1.1, -.3]])
    pixels = vo._project_ground(ground)
    assert np.allclose(vo.pixels_to_ground(pixels), ground, atol=1e-10)


def test_translation_is_measured_from_images_with_metric_scale(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    for index in range(31):
        expected = Pose(index * .015, index * .002, 0.)
        result = vo.update(floor.frame(expected), index * .1)
        assert result.tracking, (index, result)
    assert abs(result.pose.x - expected.x) < .012
    assert abs(result.pose.y - expected.y) < .012
    assert abs(result.pose.yaw) < .015
    assert result.inliers >= 12
    assert result.reprojection_error < 1.


def test_rotation_in_place_does_not_invent_translation(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    for index in range(101):
        expected = Pose(0., 0., index * .04)
        result = vo.update(floor.frame(expected), index * .1)
        assert result.tracking, (index, result)
    assert math.hypot(result.pose.x, result.pose.y) < .025
    assert abs(wrap_angle(result.pose.yaw - expected.yaw)) < .025


def test_combined_motion_composes_in_the_arbitrary_initial_frame(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    # Initial absolute image pose is deliberately not the estimator origin.
    start = Pose(.8, -.5, .7)
    expected = Pose(0., 0., 0.)
    for index in range(81):
        yaw = index * .012
        x, y = 1.1 * math.sin(yaw), 1.1 * (1 - math.cos(yaw))
        expected = Pose(x, y, yaw)
        c, s = math.cos(start.yaw), math.sin(start.yaw)
        image_pose = Pose(start.x + c*x - s*y, start.y + s*x + c*y, start.yaw + yaw)
        result = vo.update(floor.frame(image_pose), index * .1)
        assert result.tracking, (index, result)
    assert math.dist((result.pose.x, result.pose.y), (expected.x, expected.y)) < .025
    assert abs(wrap_angle(result.pose.yaw - expected.yaw)) < .025


def test_blank_frame_stops_and_short_loss_recovers_only_by_visual_matching(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    assert vo.update(floor.frame(Pose(0., 0., 0.)), 0.).tracking
    assert vo.update(floor.frame(Pose(.01, 0., 0.)), .1).tracking
    before = vo.pose
    failed = vo.update(np.zeros((HEIGHT, WIDTH, 3), np.uint8), .2)
    assert not failed.tracking
    assert failed.pose == before
    recovered = vo.update(floor.frame(Pose(.025, 0., .02)), .3)
    assert recovered.tracking
    assert abs(recovered.pose.x - .025) < .005
    assert abs(recovered.pose.yaw - .02) < .01


def test_corrupt_image_does_not_change_pose(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    vo.update(floor.frame(Pose(0., 0., 0.)), 0.)
    noise = np.random.default_rng(772).integers(0, 256, (HEIGHT, WIDTH, 3), np.uint8)
    failed = vo.update(noise, .1)
    assert not failed.tracking
    assert failed.pose == Pose(0., 0., 0.)


def test_supplied_floor_mask_excludes_independently_moving_obstacle(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    mask = np.ones((HEIGHT, WIDTH), bool)
    mask[70:170, 80:240] = False
    rng = np.random.default_rng(611)
    for index in range(16):
        expected = Pose(index * .012, 0., index * .008)
        image = floor.frame(expected)
        image[70:170, 80:240] = rng.integers(0, 256, (100, 160, 3), dtype=np.uint8)
        result = vo.update(image, index * .1, mask)
        assert result.tracking, (index, result)
    assert math.dist((result.pose.x, result.pose.y), (expected.x, expected.y)) < .012
    assert abs(wrap_angle(result.pose.yaw - expected.yaw)) < .015


def test_rgb_revisit_registration_bounds_closed_loop_drift(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    for index in range(501):
        yaw = index * 2 * math.pi / 500
        expected = Pose(.8 * math.sin(yaw), .8 * (1 - math.cos(yaw)), yaw)
        result = vo.update(floor.frame(expected), index * .1)
        assert result.tracking, (index, result)
    # No absolute pose was given: this is descriptor registration to old RGB
    # features whose positions were themselves estimated at the first visit.
    assert vo.relocalizations > 10
    assert math.hypot(result.pose.x, result.pose.y) < .025
    assert abs(result.pose.yaw) < .025


def test_long_loss_never_silently_resets_pose_and_can_require_explicit_reset(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    vo.update(floor.frame(Pose(0., 0., 0.)), 0.)
    vo.update(floor.frame(Pose(.02, 0., 0.)), .1)
    before = vo.pose
    result = vo.update(floor.frame(Pose(.03, 0., 0.)), 2.)
    assert not result.tracking
    assert result.pose == before
    assert result.reason == "reference_too_old_reset_required"
    vo.reset()
    assert vo.update(floor.frame(Pose(.03, 0., 0.)), 3.).pose == Pose(0., 0., 0.)


def test_timestamp_image_and_ground_mask_validation(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    image = floor.frame(Pose(0., 0., 0.))
    assert not vo.update(image, math.nan).tracking
    assert vo.update(image, 0.).tracking
    assert vo.update(image, 0.).reason == "nonmonotonic_timestamp"
    assert vo.update(image.astype(float), .1).reason == "invalid_rgb_frame"
    assert not vo.update(image, .2, np.zeros((HEIGHT, WIDTH), bool)).tracking
    with pytest.raises(ValueError):
        vo.update(image, .3, np.ones((10, 20), bool))


def test_module_has_no_simulator_or_command_input():
    import rover_nav.visual_odometry as module
    source = inspect.getsource(module)
    assert "import mujoco" not in source
    assert "import isaac" not in source
    assert list(inspect.signature(VisualOdometry.update).parameters) == [
        "self", "rgb", "timestamp", "ground_mask", "feature_mask"]


def test_camera_attitude_changes_do_not_become_large_robot_translation(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    maximum_translation = 0.
    for index in range(101):
        # A stationary base rocks beyond the tiny attitude changes measured in
        # the actual flat-floor simulator. Neither angle enters the estimator.
        roll = math.radians(.5) * math.sin(index * .09)
        pitch = math.radians(1.) * math.sin(index * .07)
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        base = np.eye(4)
        base[:3, :3] = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]]) @ np.array(
            [[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
        result = vo.update(floor.frame_camera(base @ T), index * .1)
        assert result.tracking, (index, result)
        maximum_translation = max(maximum_translation, math.hypot(result.pose.x, result.pose.y))
    assert maximum_translation < .004
    assert math.hypot(result.pose.x, result.pose.y) < .002
    assert not np.allclose(vo.T_level_camera, T, atol=1e-6)


def test_estimated_camera_to_level_transform_removes_world_yaw_and_xy_only():
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    yaw, pitch = .8, .03
    c, s, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    level = np.eye(4)
    level[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    level[:2, 3] = [1.2, -.4]
    rocking = np.eye(4)
    rocking[:3, :3] = [[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]]
    rocking[2, 3] = .002
    vo._T_world_camera = level @ rocking @ T
    vo.pose = vo._base_pose(vo._T_world_camera)
    np.testing.assert_allclose(vo.T_level_camera, rocking @ T, atol=1e-10)
    returned = vo.T_level_camera
    returned[2, 3] = -100.
    assert vo.T_level_camera[2, 3] > 0.
    vo.reset()
    np.testing.assert_allclose(vo.T_level_camera, T, atol=1e-10)


def test_reappearing_floor_features_replenish_an_accepted_but_sparse_track_set(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    image = floor.frame(Pose(0., 0., 0.))
    assert vo.update(image, 0.).tracking
    assert len(vo._tracking_points) > 100
    # Model retained tracks after partial occlusion: these remain valid and
    # spatially distributed, but a newly visible floor contains many fresh
    # features. Comparing only to the previous sparse count would keep 12
    # tracks until the timed keyframe boundary and risk starvation meanwhile.
    indices = np.linspace(0, len(vo._tracking_points)-1, vo.min_inliers, dtype=int)
    vo._tracking_points = vo._tracking_points[indices]
    vo._reference_world = vo._reference_world[indices]
    result = vo.update(image, .1)
    assert result.tracking
    assert result.inliers == vo.min_inliers
    assert result.features > 100
    assert len(vo._tracking_points) > 100
    assert vo._reference_timestamp == .1


def test_registration_candidates_do_not_starve_a_nearby_previous_rotation():
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    distant = [{"pose": Pose(.5, 0., .1), "time": float(i)} for i in range(8)]
    previous_rotation = {"pose": Pose(.03, .01, .05), "time": 25.}
    fresh_view = {"pose": Pose(0., 0., 0.), "time": 35.}
    vo._keyframes = distant + [previous_rotation, fresh_view]
    original_order = list(vo._keyframes)
    selected = vo._registration_candidates(Pose(0., 0., 0.), 36.)
    assert selected[0] is previous_rotation
    assert previous_rotation in selected[:6]
    assert fresh_view in selected[:6]
    assert vo._keyframes == original_order


def test_registration_candidate_ranking_preserves_view_and_age_safety_filters():
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    good = {"pose": Pose(.05, 0., .1), "time": 20.}
    too_new = {"pose": Pose(0., 0., 0.), "time": 35.5}
    too_far = {"pose": Pose(1.3, 0., 0.), "time": 20.}
    wrong_direction = {"pose": Pose(0., 0., 1.), "time": 20.}
    vo._keyframes = [too_new, too_far, wrong_direction, good]
    assert vo._registration_candidates(Pose(0., 0., 0.), 36.) == [good]


def test_registration_can_reach_a_matching_anchor_after_six_older_nonmatches(monkeypatch):
    from types import SimpleNamespace
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    old_descriptors = np.zeros((24, 32), np.uint8)
    close_descriptors = np.ones((24, 32), np.uint8)
    landmarks = np.column_stack((np.linspace(.3, 1., 24), np.linspace(-.2, .2, 24), np.zeros(24)))
    vo._keyframes = [{"pose": Pose(.5, 0., 0.), "time": float(i),
        "descriptors": old_descriptors, "landmarks": landmarks} for i in range(8)]
    vo._keyframes.append({"pose": Pose(.02, 0., 0.), "time": 20.,
        "descriptors": close_descriptors, "landmarks": landmarks})
    keypoints = [cv2.KeyPoint(float(30+i*5), 180., 3.) for i in range(24)]
    vo._orb = SimpleNamespace(detectAndCompute=lambda gray, mask: (keypoints, close_descriptors))
    def match(query, train, k):
        if train[0, 0] == 0:
            return []
        return [(cv2.DMatch(i, i, 1.), cv2.DMatch(i, (i+1)%24, 100.)) for i in range(24)]
    vo._matcher = SimpleNamespace(knnMatch=match)
    angle = .01
    rotation = np.eye(4)
    rotation[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
    corrected = rotation @ T
    monkeypatch.setattr(vo, "_solve_camera", lambda *args, **kwargs:
        (corrected, np.ones(24, bool), .1, "visual_floor_pnp_tracking"))
    result, registered = vo._map_registration(np.zeros((HEIGHT, WIDTH), np.uint8),
        np.ones((HEIGHT, WIDTH), np.uint8)*255, T.copy(), 36.)
    assert registered
    assert vo.relocalizations == 1
    np.testing.assert_allclose(result, corrected)


def test_tiny_strong_texture_islands_seed_features_without_eroding_patch_support(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    image = floor.frame(Pose(0., 0., 0.))
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    broad = np.ones((HEIGHT, WIDTH), bool)
    candidates = vo._features(gray, vo._mask(broad)).reshape(-1, 2)
    assert len(candidates) > 200
    first = np.zeros((HEIGHT, WIDTH), bool)
    second = np.zeros_like(first)
    for target, points in [(first, candidates[:100]), (second, candidates[200:300])]:
        for x, y in np.rint(points).astype(int):
            target[y-1:y+2, x-1:x+2] = True
    assert not np.any(cv2.erode(first.astype(np.uint8), np.ones((5, 5), np.uint8)))
    initial = vo.update(image, 0., ground_mask=broad, feature_mask=first)
    assert initial.tracking
    assert initial.features >= 80
    tracked = np.rint(vo._tracking_points.reshape(-1, 2)).astype(int)
    assert np.all(first[tracked[:, 1], tracked[:, 0]])
    # A changing strong-center classification must not invalidate real,
    # previously established floor tracks inside the broad support region.
    assert not np.any(second[tracked[:, 1], tracked[:, 0]])
    next_frame = vo.update(image, .1, ground_mask=broad, feature_mask=second)
    assert next_frame.tracking
    assert next_frame.inliers >= 80


def test_feature_center_mask_never_overrides_missing_floor_patch_support(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    image = floor.frame(Pose(0., 0., 0.))
    result = vo.update(image, 0., ground_mask=np.zeros((HEIGHT, WIDTH), bool),
                       feature_mask=np.ones((HEIGHT, WIDTH), bool))
    assert not result.tracking
    assert result.features == 0
    with pytest.raises(ValueError):
        vo.update(image, .1, feature_mask=np.ones((10, 20), bool))


def test_valid_existing_tracks_continue_when_only_eleven_fresh_corners_are_proven(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    image = floor.frame(Pose(0., 0., 0.))
    broad = np.ones((HEIGHT, WIDTH), bool)
    assert vo.update(image, 0., ground_mask=broad).tracking
    points = vo._tracking_points.reshape(-1, 2)
    limited = np.zeros((HEIGHT, WIDTH), bool)
    for x, y in np.rint(points[:11]).astype(int):
        limited[y, x] = True
    # Continue beyond maximum reference age without enough *fresh* corners:
    # the current RGB and >=12 validated old landmarks remain available.
    for index in range(1, 16):
        result = vo.update(image, index*.1, ground_mask=broad, feature_mask=limited)
        assert result.tracking, (index, result)
        assert result.features == 11
        assert result.inliers >= 100
    assert vo._reference_timestamp == 0.  # no sparse re-seeding
    assert vo._accepted_timestamp == 1.5
    before = vo.pose
    blank = vo.update(np.zeros_like(image), 1.6, ground_mask=np.zeros_like(broad),
                      feature_mask=np.zeros_like(limited))
    assert not blank.tracking
    assert blank.pose == before
    assert vo._accepted_timestamp == 1.5
    # A genuine RGB outage still expires the last accepted exposure, even
    # though those same old metric landmarks could geometrically match again.
    expired = vo.update(image, 2.6, ground_mask=broad, feature_mask=limited)
    assert not expired.tracking
    assert expired.reason == "reference_too_old_reset_required"
    assert expired.pose == before


def test_initialization_still_requires_twelve_proven_fresh_corners(floor):
    vo = VisualOdometry(K, T, WIDTH, HEIGHT)
    image = floor.frame(Pose(0., 0., 0.))
    broad = np.ones((HEIGHT, WIDTH), bool)
    candidates = vo._features(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY), vo._mask(broad))
    limited = np.zeros((HEIGHT, WIDTH), bool)
    for x, y in np.rint(candidates[:11].reshape(-1, 2)).astype(int):
        limited[y, x] = True
    result = vo.update(image, 0., ground_mask=broad, feature_mask=limited)
    assert not result.tracking
    assert result.features == 11
    assert vo._reference_gray is None
