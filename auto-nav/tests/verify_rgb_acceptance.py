"""Independent verification of saved RGB mission and capture evidence.

Run from tests/ against the frozen validation-v6 manifest, or copy this script
alongside source/ and source-sha256.json for a portable evidence bundle. No
simulator, renderer, network, or controller execution is needed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from PIL import Image


SOURCES = ('navigation.py', 'mission.py', 'isaac.py', 'world.py', 'vision.py',
           'vision_isaac.py', 'vision_scene.py', 'floor_vision.py', 'visual_odometry.py')


def source_bundle():
    here = Path(__file__).resolve().parent
    if (here / 'source').is_dir() and (here / 'source-sha256.json').is_file():
        return here / 'source', here / 'source-sha256.json'
    for parent in Path(__file__).resolve().parents:
        manifest = parent / 'artifacts' / 'validation-v6' / 'source-sha256.json'
        if (parent / 'rover_nav').is_dir() and manifest.is_file():
            return parent / 'rover_nav', manifest
    raise ValueError('Missing frozen source bundle: source/ and source-sha256.json')


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def integer(value):
    return type(value) is int


def matrix(value, rows, columns):
    return (isinstance(value, list) and len(value) == rows and
        all(isinstance(row, list) and len(row) == columns and all(map(number, row))
            for row in value))


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def validate(directory):
    directory = Path(directory).resolve()
    try:
        return _validate(directory)
    except (OSError, ValueError, TypeError, KeyError, IndexError, ZeroDivisionError) as error:
        return {'passed': False, 'checks': {'saved_evidence_readable_and_well_formed': False},
                'directory': str(directory), 'verification_error': f'{type(error).__name__}: {error}'}


def _validate(directory):
    report = json.loads((directory / 'report.json').read_text())
    trajectory = json.loads((directory / 'trajectory.json').read_text())
    world = json.loads((directory / 'world.json').read_text())
    calibration = json.loads((directory / 'camera_calibration.json').read_text())
    sources, manifest = source_bundle()
    expected_hashes = json.loads(manifest.read_text())
    actual_hashes = {name: hashlib.sha256((sources / name).read_bytes()).hexdigest()
                     for name in SOURCES}
    hashes_valid = (set(expected_hashes) == set(SOURCES) and
        all(isinstance(value, str) and len(value) == 64 and
            all(char in '0123456789abcdef' for char in value)
            for value in expected_hashes.values()))
    rooms = report['rooms']
    physical_rooms = world['source']['rooms']
    room_names = [room['name'] for room in rooms]
    definitions_match = (len(rooms) == len(physical_rooms) == 3 and len(set(room_names)) == 3 and
        {room['name']: room['bounds'] for room in rooms} ==
        {room['name']: room['bounds'] for room in physical_rooms})
    bounds_valid = all(isinstance(room['bounds'], list) and len(room['bounds']) == 4 and
        all(map(number, room['bounds'])) and room['bounds'][2] - room['bounds'][0] > .9 and
        room['bounds'][3] - room['bounds'][1] > .9 for room in rooms)
    trajectory_valid = (isinstance(trajectory, list) and len(trajectory) > 100 and
        all(isinstance(row, list) and len(row) == 4 and all(map(number, row))
            for row in trajectory))
    if not trajectory_valid or not bounds_valid or not definitions_match:
        raise ValueError('Malformed trajectory or inconsistent three-room scene definitions')
    times = [row[3] for row in trajectory]
    duration, distance = report['simulation_seconds'], report['distance_m']
    chord_distance = sum(math.hypot(second[0] - first[0], second[1] - first[1])
                         for first, second in zip(trajectory, trajectory[1:]))
    coverage = report['coverage']
    reachable, observed = coverage['reachable_cells'], coverage['observed_reachable_cells']
    fraction = coverage['observed_reachable_fraction']
    coverage_valid = (integer(reachable) and integer(observed) and reachable > 0 and
        0 <= observed <= reachable and number(fraction) and 0 <= fraction <= 1 and
        math.isclose(fraction, observed / reachable, rel_tol=1e-12, abs_tol=1e-12))
    calibration_valid = (integer(calibration['width']) and integer(calibration['height']) and
        calibration['width'] > 0 and calibration['height'] > 0 and
        matrix(calibration['K'], 3, 3) and matrix(calibration['T_base_camera'], 4, 4) and
        calibration['K'][0][0] > 0 and calibration['K'][1][1] > 0 and
        calibration['T_base_camera'][2][3] > 0 and
        calibration['T_base_camera'][3] == [0., 0., 0., 1.])
    checks = {
        'reported_success': report['success'] is True,
        'rgb_only_mode': report['sensing'] == 'rgb',
        'finished_without_failure': report['finished'] is True and report['failure'] is None,
        'terminal_exploration_completion': report['navigation_state'] ==
            report['navigator']['status'] == 'complete',
        'all_three_physical_rooms_entered': report['all_rooms_visited'] is True and all(
            room['visited'] is True and number(room['first_visit_time']) for room in rooms),
        'saved_world_matches_room_definitions_and_seed': definitions_match and
            report['seed'] == world['source']['seed'],
        'zero_obstacle_contacts': integer(report['collision_count']) and
            report['collision_count'] == 0 and number(report['max_collision_force_n']) and
            report['max_collision_force_n'] == 0.,
        'coverage_counts_and_fraction_consistent': coverage_valid,
        'at_least_95_percent_observed_reachable_floor': coverage_valid and fraction >= .95,
        'exact_nine_file_frozen_manifest': hashes_valid and actual_hashes == expected_hashes,
        'frozen_current_production_source': report['source_sha256_at_start'] == expected_hashes,
        'physical_trajectory_recorded': trajectory_valid and integer(report['steps']) and
            report['steps'] == len(trajectory) and number(distance) and distance > .1 and
            distance + 1e-6 >= chord_distance,
        'trajectory_and_mission_times_consistent': times[0] >= 0 and
            all(first < second for first, second in zip(times, times[1:])) and
            number(duration) and times[-1] <= duration,
        'fixed_camera_calibration_recorded': calibration_valid,
    }
    # Exactly the strict physical-entry rule used by Mission.tick, including
    # first-entry time; the report's visited booleans alone cannot establish it.
    for room in rooms:
        x0, y0, x1, y1 = room['bounds']
        entries = [row[3] for row in trajectory if
            x0 + .45 < row[0] < x1 - .45 and y0 + .45 < row[1] < y1 - .45]
        checks[f"trajectory_enters_{room['name']}"] = bool(entries)
        checks[f"first_entry_time_matches_{room['name']}"] = bool(entries) and \
            number(room['first_visit_time']) and math.isclose(
                entries[0], room['first_visit_time'], rel_tol=0., abs_tol=1e-7)
    centres = [((r['bounds'][0]+r['bounds'][2])/2,
                (r['bounds'][1]+r['bounds'][3])/2) for r in rooms]
    a, b, c = centres
    checks['room_centres_are_not_linear'] = abs(
        (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])) > 1.
    captures = directory / 'captures'
    records = sorted(captures.glob('capture_*.json'))
    images = sorted(captures.glob('capture_*.png'))
    saved, requested = report['captures_saved'], report['capture_events']
    checks['all_requested_captures_saved'] = (report['capture_complete'] is True and
        integer(saved) and integer(requested) and
        len(records) == len(images) == saved == requested == report['navigator']['capture_count'] > 0)
    events = [event for event in report['events'] if event['type'] == 'rgb_saved']
    checks['capture_events_match_saved_frames'] = len(events) == len(records) and all(
        event['frame_id'] == index and number(event['time'])
        for index, event in enumerate(events))
    event_times = {event['frame_id']: event['time'] for event in events}
    capture_checks = []
    capture_times = []
    allowed_keys = {'frame_id', 'simulation_time', 'image', 'image_sha256', 'width',
        'height', 'K', 'T_base_camera', 'estimated_base_pose', 'tracking',
        'pose_source', 'depth_source'}
    for index, path in enumerate(records):
        record = json.loads(path.read_text())
        image_path = path.with_suffix('.png')
        with Image.open(image_path) as image:
            dimensions_match = image.size == (record['width'], record['height'])
            rgb_png = image.mode == 'RGB' and image.format == 'PNG'
            image.verify()
        timestamp = record['simulation_time']
        capture_times.append(timestamp)
        capture_checks.append(
            set(record) == allowed_keys and integer(record['frame_id']) and record['frame_id'] == index and
            path.stem == f'capture_{index:04d}' and record['image'] == image_path.name and
            dimensions_match and rgb_png and calibration_valid and
            record['width'] == calibration['width'] and record['height'] == calibration['height'] and
            record['K'] == calibration['K'] and record['T_base_camera'] == calibration['T_base_camera'] and
            record['image_sha256'] == hashlib.sha256(image_path.read_bytes()).hexdigest() and
            record['depth_source'] is None and
            record['pose_source'] == 'monocular_floor_visual_odometry' and record['tracking'] is True and
            set(record['estimated_base_pose']) == {'x', 'y', 'yaw'} and
            all(map(number, record['estimated_base_pose'].values())) and
            number(timestamp) and 0 <= timestamp <= duration and
            index in event_times and math.isclose(timestamp, event_times[index], rel_tol=0., abs_tol=1e-7) and
            any(math.isclose(timestamp, time, rel_tol=0., abs_tol=1e-7) for time in times))
    checks['rgb_capture_integrity_and_sensor_contract'] = bool(capture_checks) and all(capture_checks)
    checks['capture_exposure_times_strictly_increase'] = bool(capture_times) and all(
        number(first) and number(second) and first < second
        for first, second in zip(capture_times, capture_times[1:]))
    return clean({'passed': all(checks.values()), 'checks': checks, 'directory': str(directory),
        'seed': report['seed'], 'distance_m': distance, 'simulation_seconds': duration,
        'coverage': fraction, 'collision_count': report['collision_count'],
        'captures_verified': len(records), 'localization_evaluation': report['localization_evaluation'],
        'source_sha256': expected_hashes, 'verified_source_directory': str(sources),
        'frozen_manifest': str(manifest)})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    results = [validate(path) for path in args.directories]
    summary = {'passed': all(result['passed'] for result in results),
        'method': 'Strict saved-trajectory room entries, frozen nine-source manifest and RGB capture verification',
        'trials': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps(summary, indent=2, allow_nan=False))
    raise SystemExit(0 if summary['passed'] else 1)
