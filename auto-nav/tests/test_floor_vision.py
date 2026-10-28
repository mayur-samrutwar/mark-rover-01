"""Image-only floor-sensor tests with independently painted obstacle pixels."""
import inspect
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from rover_nav.floor_vision import FloorVision
from rover_nav.visual_odometry import VisualOdometry


WIDTH, HEIGHT = 320, 240
K = np.array([[230., 0., 159.5], [0., 230., 119.5], [0., 0., 1.]])
T = np.eye(4)
pitch = math.radians(30)
T[:3, :3] = [[0., -math.sin(pitch), math.cos(pitch)],
             [-1., 0., 0.], [0., -math.cos(pitch), -math.sin(pitch)]]
T[:3, 3] = [.055, .012, .30]


def floor_image(exposure=1.):
    rng = np.random.default_rng(472)
    noise = cv2.GaussianBlur(rng.uniform(.4, 1.6, (HEIGHT, WIDTH)).astype(np.float32),
                            (3, 3), .55)
    colour = np.array([142., 126., 99.])
    return np.clip(noise[:, :, None] * colour * exposure, 0, 255).astype(np.uint8)


def project(points, transform=T):
    points = np.column_stack((np.asarray(points), np.ones(len(points))))
    camera = (points @ np.linalg.inv(transform).T)[:, :3]
    pixel = camera @ K.T
    return np.rint(pixel[:, :2] / pixel[:, 2:3]).astype(np.int32)


def barrier(image, distance=1., width=.5, height=.5, colour=(90, 35, 30), offset=0., *, transform=T):
    result = image.copy()
    x, y = transform[0, 3] + distance, transform[1, 3] + offset
    corners = project([[x, y-width/2, 0.], [x, y+width/2, 0.],
                       [x, y+width/2, height], [x, y-width/2, height]], transform)
    cv2.fillConvexPoly(result, corners, tuple(map(int, colour)))
    return result


def center(result):
    return int(np.argmin(np.abs(result.angles)))


def test_floor_only_has_metric_visible_intervals_and_no_nearfield_extrapolation():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    result = sensor.process(floor_image())
    assert result.valid, result.reason
    assert result.confidence > .9
    middle = center(result)
    assert result.ranges[middle] >= 2.45
    assert not result.hits[middle]
    assert .15 < result.min_ranges[middle] < .3
    radii = np.linalg.norm(result.free_points - T[:2, 3], axis=1)
    assert radii.min() >= np.nanmin(result.min_ranges) - 1e-8
    assert radii.max() <= sensor.max_range + 1e-8
    assert result.mask.dtype == bool
    assert isinstance(result.free_intervals, tuple)
    assert len(result.free_intervals) == len(result.angles)
    assert result.free_intervals[middle] == ((result.min_ranges[middle], result.ranges[middle]),)


def test_ground_contact_boundary_stops_free_evidence_and_occluded_floor():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    image = barrier(floor_image(), distance=1.)
    result = sensor.process(image)
    assert result.valid, result.reason
    middle = center(result)
    assert result.hits[middle]
    assert .80 <= result.ranges[middle] <= 1.
    relative = result.free_points - T[:2, 3]
    near_center = np.abs(np.arctan2(relative[:, 1], relative[:, 0])) < .05
    assert np.max(np.linalg.norm(relative[near_center], axis=1)) <= 1.
    # Floor can remain visible around either side without clearing its shadow.
    assert np.nanmax(result.ranges) > 1.6


def test_positive_contact_location_is_separate_from_retreated_free_endpoint():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert sensor.process(floor_image()).valid
    result = sensor.process(barrier(floor_image(), .4, .15, .5, (90, 35, 30)))
    assert result.valid
    middle = center(result)
    assert result.hits[middle]
    assert result.ranges[middle] <= .25
    assert .375 <= result.hit_ranges[middle] <= .425
    assert result.hit_ranges.shape == result.ranges.shape
    positive = result.hits & np.isfinite(result.ranges)
    assert np.isfinite(result.hit_ranges[result.hits]).all()
    assert np.isnan(result.hit_ranges[~result.hits]).all()
    assert np.all(result.hit_ranges[positive] >= result.ranges[positive])


def test_thin_furniture_leg_is_a_conservative_boundary():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    result = sensor.process(barrier(floor_image(), .85, .045, .5, (20, 100, 150)))
    assert result.valid
    middle = center(result)
    assert result.hits[middle]
    assert result.ranges[middle] < .90


def test_uniform_floor_coloured_obstacle_is_not_confident_free_space():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    image = barrier(floor_image(), 1., .6, .5, (142, 126, 99))
    result = sensor.process(image)
    assert result.valid
    middle = center(result)
    assert not result.hits[middle]  # Texture absence is unknown, not a proved solid.
    assert np.isnan(result.hit_ranges[middle])
    assert result.ranges[middle] <= 1.


def test_bright_neutral_wall_is_positive_appearance_evidence():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    result = sensor.process(barrier(floor_image(), 1., .6, .5, (235, 235, 235)))
    assert result.valid
    assert result.hits[center(result)]
    assert result.ranges[center(result)] <= 1.


def test_untextured_first_visible_patch_preserves_only_later_visible_interval():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    image = floor_image()
    image[175:] = [142, 126, 99]
    result = sensor.process(image)
    assert result.valid, result.reason
    middle = center(result)
    assert not result.hits[middle]
    assert np.isnan(result.hit_ranges[middle])
    first_visible = np.flatnonzero(sensor._visible[middle])[0]
    assert result.min_ranges[middle] > sensor._distances[first_visible] + .1
    assert result.ranges[middle] >= 2.45
    relative = result.free_points - sensor.origin
    central = np.abs(np.arctan2(relative[:, 1], relative[:, 0])) < .01
    assert np.linalg.norm(relative[central], axis=1).min() >= result.min_ranges[middle]
    assert result.tracking_mask.sum() > 1000
    assert VisualOdometry(K, T, WIDTH, HEIGHT).update(
        image, 0., result.tracking_mask, feature_mask=result.feature_mask).tracking


@pytest.mark.parametrize("uncertain_prefix", [False, True])
def test_positive_near_object_blocks_later_supported_floor_interval(uncertain_prefix):
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert sensor.process(floor_image()).valid
    # A low box leaves real textured floor visible above its image silhouette.
    # Conservative erosion removes the small near patch. That must not allow
    # interval extraction to jump over the positive box to its visible shadow.
    image = floor_image()
    if uncertain_prefix:
        image[175:] = [142, 126, 99]
    result = sensor.process(barrier(image, .3, .20, .035))
    assert result.valid
    middle = center(result)
    visible = np.flatnonzero(sensor._visible[middle])
    pixels = sensor._pixels[middle, visible]
    supported = result.mask[pixels[:, 1], pixels[:, 0]]
    assert np.any(supported & (sensor._distances[visible] > .5))
    # Unknown texture before the object cannot suppress its independently
    # visible positive contact, nor permit floor behind it to clear its shadow.
    assert result.hits[middle]
    assert .275 <= result.hit_ranges[middle] <= .325
    assert result.ranges[middle] == result.min_ranges[middle]
    assert result.free_intervals[middle] == ()
    relative = result.free_points - sensor.origin
    central = np.abs(np.arctan2(relative[:, 1], relative[:, 0])) < .01
    assert not central.any()


def test_positive_obstacle_at_first_visible_sample_has_zero_free_interval():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert sensor.process(floor_image()).valid
    result = sensor.process(barrier(floor_image(), .25, .15, .5, (90, 35, 30), .15))
    assert result.valid
    assert result.ranges.shape == result.min_ranges.shape == result.hits.shape == result.angles.shape
    finite = np.isfinite(result.ranges) & np.isfinite(result.min_ranges)
    assert np.all(result.ranges[finite] >= result.min_ranges[finite])
    assert np.all(result.ranges[finite] <= sensor.max_range)
    zero_length_hits = finite & result.hits & (result.ranges == result.min_ranges)
    assert zero_length_hits.sum() > 10
    assert np.all(result.hit_ranges[zero_length_hits] >= result.min_ranges[zero_length_hits])


@pytest.mark.parametrize("brightness", [0, 100, 255])
def test_blank_or_uniform_calibration_stops(brightness):
    result = FloorVision(K, T, WIDTH, HEIGHT).process(
        np.full((HEIGHT, WIDTH, 3), brightness, np.uint8))
    assert not result.valid
    assert not result.mask.any()
    assert len(result.free_points) == 0
    assert np.isnan(result.ranges).all()


def test_exposure_change_preserves_floor_prior_and_obstacle_boundary():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    initial = sensor.process(floor_image())
    assert initial.valid
    darker = sensor.process(barrier(floor_image(.45), 1., colour=(40, 15, 12)))
    assert darker.valid, darker.reason
    middle = center(darker)
    assert .8 <= darker.ranges[middle] <= 1.
    assert darker.hits[middle]


def test_invalid_frame_and_calibration_fail_closed():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert not sensor.process(np.zeros((HEIGHT, WIDTH, 3))).valid
    assert not sensor.process(np.zeros((10, 10, 3), np.uint8)).valid
    with pytest.raises(ValueError):
        FloorVision(K, np.eye(4), WIDTH, HEIGHT)


def test_module_has_no_scene_or_privileged_sensor_dependency():
    import rover_nav.floor_vision as module
    source = inspect.getsource(module)
    assert "import mujoco" not in source
    assert "import isaac" not in source
    assert "import world" not in source
    assert list(inspect.signature(FloorVision.process).parameters) == ["self", "rgb"]


def test_rendered_speckled_floor_gaps_do_not_become_false_nearby_obstacles():
    # This saved RGB exposure is only an image fixture. The estimator is given
    # no fixture depth, scene geometry, object labels or simulation state.
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    rgb = cv2.cvtColor(cv2.imread(str(fixtures / "floor_rgb_initial.png")), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    result = sensor.process(rgb)
    assert result.valid, result.reason
    assert result.ranges[center(result)] >= 2.45
    assert np.mean(result.mask[sensor._seed]) > .99
    assert result.mask[137, 160]  # Smooth carpet gap, previously a false obstacle.
    assert not result.mask[60, 270]  # Visible solid furniture remains excluded.
    darker = sensor.process(np.rint(rgb.astype(float) * .65).astype(np.uint8))
    assert darker.valid, darker.reason
    assert darker.ranges[center(darker)] >= 2.4


def test_rendered_uniform_wall_never_clears_floor_behind_its_ground_contact():
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_bright_start.png")).valid
    result = sensor.process(read("floor_rgb_facing_wall.png"))
    assert result.valid, result.reason
    assert result.tracking_mask.shape == result.mask.shape
    assert result.tracking_mask.dtype == bool
    assert result.tracking_mask.sum() > result.mask.sum()
    # Fixture evaluation label only: at this frozen test pose the front wall
    # plane is 0.54 m forward of the base. That label is never given to sensor.
    # Test every bearing, especially FOV-clipped edge rays that previously
    # cleared 0.64 m forward even though the central ray stopped correctly.
    forward_endpoints = sensor.origin[0] + result.ranges * np.cos(result.angles)
    assert np.nanmax(forward_endpoints) <= .54
    assert result.free_points[:, 0].max() <= .54
    assert .35 < result.ranges[center(result)] <= .54


def test_smooth_single_axis_wall_shading_is_not_floor_texture():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert sensor.process(floor_image()).valid
    image = floor_image()
    painted = barrier(image, 1., .65, .5, (1, 1, 1))
    face = np.all(painted == [1, 1, 1], axis=2)
    shade = np.linspace(.7, 1.3, HEIGHT)[:, None, None]
    shaded = np.broadcast_to(shade * np.array([142., 126., 99.]), image.shape)
    image[face] = np.clip(shaded[face], 0, 255).astype(np.uint8)
    result = sensor.process(image)
    assert result.valid
    assert result.ranges[center(result)] <= 1.


@pytest.mark.parametrize("image_name,base_x,yaw", [
    ("floor_rgb_wall_oblique2.png", .6, 2.),
    ("floor_rgb_wall_oblique24.png", .6, 2.4),
    ("floor_rgb_wall_near.png", .3, 1.78),
    ("floor_rgb_wall_grazing.png", .55, math.pi / 2),
    ("floor_rgb_wall_grazing_near.png", .4, 1.9),
])
def test_oblique_wall_never_gets_free_evidence_across_its_plane(image_name, base_x, yaw):
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_bright_start.png")).valid
    result = sensor.process(read(image_name))
    assert result.valid, result.reason
    assert len(result.free_points) > 100
    # The test evaluator rotates the returned points into the fixture frame.
    # Base x/yaw and the wall label are deliberately not sensor inputs.
    c, s = math.cos(yaw), math.sin(yaw)
    free_world_x = base_x + c * result.free_points[:, 0] - s * result.free_points[:, 1]
    assert free_world_x.min() >= .06
    endpoints = sensor.origin + np.column_stack((np.cos(result.angles) * result.ranges,
                                                 np.sin(result.angles) * result.ranges))
    endpoint_world_x = base_x + c * endpoints[:, 0] - s * endpoints[:, 1]
    valid_free = np.isfinite(result.ranges) & (result.ranges > result.min_ranges)
    assert endpoint_world_x[valid_free].min() >= .06


def test_close_wall_view_keeps_tracking_and_partial_floor_evidence_valid():
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_bright_start.png")).valid
    result = sensor.process(read("floor_rgb_wall_south_near.png"))
    assert result.valid, result.reason
    assert result.confidence > .9
    assert result.tracking_mask.sum() > result.mask.sum() > 250
    assert np.isfinite(result.ranges).sum() >= 10
    # The brighter uniform wall supplies its own appearance contact. Its
    # location must remain separate from the eroded floor uncertainty endpoint.
    middle = center(result)
    assert result.hits[middle]
    assert result.hit_ranges[middle] > result.ranges[middle] + .1
    assert np.isnan(result.hit_ranges[~result.hits]).all()


def test_partial_mapping_view_retains_actual_rgb_feature_hypotheses_for_odometry():
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_bright_start.png")).valid
    rgb = read("floor_rgb_partial_mapping_failure.png")
    result = sensor.process(rgb)
    assert result.valid
    assert result.tracking_mask.sum() > 1000
    assert result.hits[center(result)]
    assert np.isnan(result.hit_ranges[~result.hits]).all()
    # Mapping uncertainty must not manufacture a texture outage. A separate
    # image-motion estimator still validates the available floor features.
    odometry = VisualOdometry(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert odometry.update(rgb, 0., result.tracking_mask, feature_mask=result.feature_mask).tracking
    tracked = odometry.update(rgb, .1, result.tracking_mask, feature_mask=result.feature_mask)
    assert tracked.tracking
    assert tracked.inliers >= 12


def test_no_conservative_connection_still_preserves_raw_tracking_features():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert sensor.process(floor_image()).valid
    rgb = barrier(floor_image(), .25, 2., .5, (90, 35, 30))
    result = sensor.process(rgb)
    assert not result.valid
    assert result.reason == "no_connected_floor_evidence"
    assert result.tracking_mask.sum() > 1000
    assert not result.mask.any()
    assert np.isnan(result.ranges).all()
    assert result.hits[center(result)]
    assert .225 <= result.hit_ranges[center(result)] <= .275
    assert VisualOdometry(K, T, WIDTH, HEIGHT).update(
        rgb, 0., result.tracking_mask, feature_mask=result.feature_mask).tracking


@pytest.mark.parametrize("exposure", [.35, .45, .7, 1., 1.35])
def test_learned_brightness_envelope_tracks_textured_floor_exposure_without_hits(exposure):
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert sensor.process(floor_image()).valid
    result = sensor.process(floor_image(exposure))
    assert result.valid, result.reason
    assert not result.hits.any()
    assert np.isnan(result.hit_ranges).all()
    assert result.ranges[center(result)] >= 2.45


def test_actual_collision_wall_does_not_relearn_floor_brightness_or_clear_wall():
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_collision_start.png")).valid
    initial_brightness = sensor._floor_brightness
    wall = read("floor_rgb_collision_wall.png")
    for _ in range(3):
        result = sensor.process(wall)
        assert result.valid, result.reason
        assert sensor._floor_brightness < initial_brightness * 1.15
        middle = center(result)
        assert result.hits[middle]
        assert .20 <= result.hit_ranges[middle] <= .35
        # Evaluation labels only: actual last exposure pose and the partition
        # north face. These coordinates never enter FloorVision.
        x, y, yaw = 7.6068073397066005, 4.2795846423132815, -2.1621274302383124
        points = result.free_points
        wx = x + math.cos(yaw) * points[:, 0] - math.sin(yaw) * points[:, 1]
        wy = y + math.sin(yaw) * points[:, 0] + math.cos(yaw) * points[:, 1]
        at_wall_x = (wx >= 4.94) & (wx <= 7.76)
        assert np.all(wy[at_wall_x] >= 4.06)

    # A subsequent view consisting entirely of smooth wall cannot rescale
    # brightness at all. Mapping fails closed, while visible contacts survive.
    before = sensor._floor_brightness
    wall_only = cv2.resize(wall[:120], (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
    unavailable = sensor.process(wall_only)
    assert not unavailable.valid
    assert sensor._floor_brightness == before
    assert not unavailable.mask.any()
    assert np.isnan(unavailable.ranges).all()
    assert unavailable.free_intervals == tuple(() for _ in unavailable.angles)
    assert unavailable.hits[center(unavailable)]
    assert np.isfinite(unavailable.hit_ranges[unavailable.hits]).all()


def test_actual_doorway_preserves_disjoint_visible_floor_segments_without_gap_fill():
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_collision_start.png")).valid
    result = sensor.process(read("floor_rgb_doorway_texture_gap.png"))
    assert result.valid, result.reason
    middle = center(result)
    intervals = result.free_intervals[middle]
    assert len(intervals) == 2
    np.testing.assert_allclose(intervals[0], [.225, .325])
    np.testing.assert_allclose(intervals[1], [.55, 2.5])
    assert (result.min_ranges[middle], result.ranges[middle]) == intervals[0]
    assert not result.hits[middle]
    # Neither representation nor point evidence may replace these two visible
    # regions by their convex hull; the texture-uncertain strip stays unknown.
    assert not any(near <= .45 <= far for near, far in intervals)
    relative = result.free_points - sensor.origin
    central = np.abs(np.arctan2(relative[:, 1], relative[:, 0])) < .001
    distance = np.linalg.norm(relative[central], axis=1)
    assert np.any(distance >= .6)
    assert not np.any((distance > .325 + 1e-9) & (distance < .55 - 1e-9))
    for index, segments in enumerate(result.free_intervals):
        assert isinstance(segments, tuple)
        for offset, (near, far) in enumerate(segments):
            assert np.isfinite([near, far]).all()
            assert 0. < near < far <= sensor.max_range
            if offset:
                assert segments[offset - 1][1] < near
            if result.hits[index]:
                assert far < result.hit_ranges[index]


def test_weak_wall_shading_is_rejected_from_mapping_and_feature_centres():
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_bright_start.png")).valid
    result = sensor.process(read("floor_rgb_wall_weak_shading.png"))
    assert result.valid
    # Independent image labels: this pixel is on the smooth shaded wall;
    # the carpet begins below it. Weak quantized shading once passed as texture.
    assert not result.mask[82, 241]
    assert not result.feature_mask[82, 241]
    assert result.tracking_mask[180, 280]


def test_actual_texture_starvation_frame_has_many_supported_feature_centres():
    fixtures = Path(__file__).parent / "fixtures"
    calibration = json.loads((fixtures / "floor_rgb_calibration.json").read_text())
    read = lambda name: cv2.cvtColor(cv2.imread(str(fixtures / name)), cv2.COLOR_BGR2RGB)
    sensor = FloorVision(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    assert sensor.process(read("floor_rgb_bright_start.png")).valid
    rgb = read("floor_rgb_feature_starvation.png")
    result = sensor.process(rgb)
    assert result.valid
    assert result.feature_mask.shape == result.tracking_mask.shape
    assert result.feature_mask.dtype == bool
    assert np.all(~result.feature_mask | result.tracking_mask)
    odometry = VisualOdometry(calibration["K"], calibration["T_base_camera"], WIDTH, HEIGHT)
    visual = odometry.update(rgb, 0., result.tracking_mask, feature_mask=result.feature_mask)
    assert visual.tracking
    assert visual.features >= 30
    assert visual.inliers >= 12


def test_image_estimated_camera_transform_updates_metric_projection_without_relearning():
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    assert sensor.process(floor_image()).valid
    appearance = tuple(value.copy() if isinstance(value, np.ndarray) else value
                       for value in sensor._appearance)
    old_ground = sensor.ground.copy()
    angle = math.radians(5.)
    delta = np.array([[math.cos(angle), 0., math.sin(angle)], [0., 1., 0.],
                      [-math.sin(angle), 0., math.cos(angle)]])
    estimated = T.copy()
    estimated[:3, :3] = delta @ T[:3, :3]
    estimated[:3, 3] = [.065, .018, .34]
    sensor.set_camera_transform(estimated)
    for before, after in zip(appearance, sensor._appearance):
        np.testing.assert_array_equal(before, after)
    assert not np.allclose(old_ground, sensor.ground, equal_nan=True)
    np.testing.assert_allclose(sensor.origin, estimated[:2, 3])
    ray = estimated[:3, :3] @ np.linalg.solve(K, [160., 200., 1.])
    expected = estimated[:2, 3] - estimated[2, 3] / ray[2] * ray[:2]
    np.testing.assert_allclose(sensor.ground[200, 160], expected)
    rgb = barrier(floor_image(), 1., .6, .5, transform=estimated)
    result = sensor.process(rgb)
    assert result.valid
    assert result.hits[center(result)]
    assert .95 <= result.hit_ranges[center(result)] <= 1.05
    assert result.ranges[center(result)] <= 1.


@pytest.mark.parametrize("bad", [np.eye(3), np.full((4, 4), np.nan), np.zeros((4, 4))])
def test_invalid_dynamic_transform_cannot_replace_valid_geometry(bad):
    sensor = FloorVision(K, T, WIDTH, HEIGHT)
    before = sensor.T.copy(), sensor.ground.copy()
    with pytest.raises(ValueError):
        sensor.set_camera_transform(bad)
    np.testing.assert_array_equal(sensor.T, before[0])
    np.testing.assert_array_equal(sensor.ground, before[1])
