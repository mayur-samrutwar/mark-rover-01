"""Conservative surface-completion fixtures; no engine, GPU, or private images."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import completion


def glb_chunks(payload):
    magic, version, size = struct.unpack_from('<4sII', payload)
    if magic != b'glTF' or version != 2 or size != len(payload):
        raise AssertionError('Invalid GLB')
    chunks = []
    offset = 12
    while offset < len(payload):
        length, kind = struct.unpack_from('<II', payload, offset)
        chunks.append((kind, payload[offset + 8:offset + 8 + length]))
        offset += 8 + length
    return chunks


def open_prism(polygon, heights=None, bottom=0.0, top=2.0):
    """Only side walls: two independent horizontal perimeter holes."""
    import trimesh
    count = len(polygon)
    if heights is None:
        heights = [top] * count
    vertices = [[x, bottom, z] for x, z in polygon]
    vertices += [[x, heights[index], z] for index, (x, z) in enumerate(polygon)]
    faces = []
    for a in range(count):
        b = (a + 1) % count
        faces += [[a, b, b + count], [a, b + count, a + count]]
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


def vertical_opening():
    """Planar wall ring: both boundaries are vertical and must remain open."""
    import trimesh
    outer = [[6, 0, 0], [10, 0, 0], [10, 3, 0], [6, 3, 0]]
    inner = [[7, .5, 0], [9, .5, 0], [9, 2.5, 0], [7, 2.5, 0]]
    faces = []
    for a in range(4):
        b = (a + 1) % 4
        faces += [[a, b, b + 4], [a, b + 4, a + 4]]
    return trimesh.Trimesh(vertices=outer + inner, faces=faces, process=False)


def area_xz(polygon):
    return abs(sum(a[0] * b[1] - b[0] * a[1]
                   for a, b in zip(polygon, polygon[1:] + polygon[:1]))) / 2


def inside_xz(point, polygon):
    """Even-odd polygon membership for strictly interior triangle centroids."""
    x, z = point
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > z) != (b[1] > z):
            crossing = a[0] + (z - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if x < crossing:
                inside = not inside
    return inside


@unittest.skipUnless(importlib.util.find_spec('trimesh'), 'reconstruction dependencies unavailable')
class CompletionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        self.square = [[-2, -2], [2, -2], [2, 2], [-2, 2]]

    def tearDown(self):
        self.temp.cleanup()

    def fixture(self, mesh, source_space='raw', scale=1.0, up_axis='Y'):
        import numpy as np
        import trimesh
        from PIL import Image
        exports = self.root / 'exports'
        if source_space == 'scaled':
            exports /= 'scaled'
        exports.mkdir(parents=True, exist_ok=True)
        mesh = mesh.copy()
        mesh.apply_scale(scale)
        mesh.visual = trimesh.visual.TextureVisuals(
            uv=np.zeros((len(mesh.vertices), 2)),
            material=trimesh.visual.material.SimpleMaterial(
                image=Image.new('RGB', (3, 2), (155, 115, 75))))
        (exports / 'scene.glb').write_bytes(trimesh.Scene(mesh).export(file_type='glb'))
        (exports / 'surface.ply').write_bytes(mesh.export(file_type='ply'))
        cameras = {'units': 'metres' if source_space == 'scaled' else 'unknown',
                   'coordinate_frame': 'fixture unchanged Y-up frame', 'up_axis': up_axis,
                   'cameras': [{'center': [scale, scale, scale],
                                'rotation_world_to_camera': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                'translation': [-scale, -scale, -scale]}]}
        (exports / 'cameras.json').write_text(json.dumps(cameras))
        if source_space == 'scaled':
            (exports / 'calibration.json').write_text(json.dumps({'meters_per_unit': scale, 'units': 'metres'}))
        images = self.root / 'images'
        images.mkdir(exist_ok=True)
        Image.new('RGB', (4, 3), (75, 120, 90)).save(images / 'fixture.jpg')
        return exports

    def execute(self, source_space='raw'):
        result = completion.complete_scene(self.root, source_space=source_space)
        self.assertIsInstance(result, dict)
        self.assertIsInstance(result['completion'], dict)
        self.assertIsInstance(result['artifacts'], list)
        self.assertIsInstance(result['warnings'], list)
        self.assertFalse(result['completion']['collision_validated'])
        self.assertEqual(result['completion']['source_space'], source_space)
        for artifact in result['artifacts']:
            path = (self.root / artifact['path']).resolve()
            self.assertTrue(path.is_relative_to(self.root))
            self.assertEqual(artifact['bytes'], path.stat().st_size, artifact['path'])
        return result

    def artifact(self, result, name):
        matches = [self.root / item['path'] for item in result['artifacts'] if item['name'] == name]
        self.assertEqual(len(matches), 1, name)
        return matches[0]

    def mesh(self, result, name):
        import trimesh
        scene = trimesh.load(str(self.artifact(result, name)), force='scene', process=False)
        return trimesh.util.concatenate([item for item in scene.dump() if isinstance(item, trimesh.Trimesh)])

    def test_horizontal_room_gaps_close_but_vertical_openings_remain(self):
        import geometry
        import trimesh
        self.fixture(trimesh.util.concatenate([open_prism(self.square), vertical_opening()]))
        result = self.execute()
        report = result['completion']
        self.assertEqual(report['filled_loops'], 2)
        self.assertEqual(report['skipped_loops'], 2)
        self.assertEqual(report['added_triangles'], 4)
        self.assertEqual(len(report['loops']), 4)
        final = self.mesh(result, 'surface-completed.ply')
        topology = geometry.surface_topology(final)
        self.assertEqual(topology['boundary_edges'], 8)
        self.assertEqual(topology['nonmanifold_edges'], 0)
        self.assertEqual(report['topology_after']['boundary_edges'], 8)
        inferred = self.mesh(result, 'inferred-surfaces.ply')
        self.assertEqual(len(inferred.faces), 4)
        # The wall opening is away from the room; no inferred triangle may touch it.
        self.assertLess(float(inferred.vertices[:, 0].max()), 3)

    def test_concave_horizontal_gaps_use_polygon_interior_without_bridging_the_notch(self):
        import numpy as np
        polygon = [[0, 0], [3, 0], [3, 1], [1, 1], [1, 3], [0, 3]]
        self.fixture(open_prism(polygon))
        result = self.execute()
        inferred = self.mesh(result, 'inferred-surfaces.ply')
        self.assertEqual(result['completion']['filled_loops'], 2)
        self.assertAlmostEqual(float(inferred.area), 2 * area_xz(polygon), places=6)
        for center in np.asarray(inferred.triangles_center):
            self.assertTrue(inside_xz(center[[0, 2]], polygon), center.tolist())
        self.assertEqual(result['completion']['topology_after']['boundary_edges'], 0)
        self.assertEqual(result['completion']['topology_after']['nonmanifold_edges'], 0)

    def test_nonplanar_loop_is_rejected_while_the_flat_loop_is_filled(self):
        self.fixture(open_prism(self.square, heights=[2, 2, 3, 2]))
        result = self.execute()
        self.assertEqual(result['completion']['filled_loops'], 1)
        self.assertEqual(result['completion']['skipped_loops'], 1)
        inferred = self.mesh(result, 'inferred-surfaces.ply')
        self.assertTrue(all(abs(float(y)) < 1e-6 for y in inferred.vertices[:, 1]))
        self.assertEqual(result['completion']['topology_after']['boundary_edges'], 4)

    def test_original_geometry_texture_and_images_are_unchanged_and_inferred_nodes_are_explicit(self):
        exports = self.fixture(open_prism(self.square))
        sources = [exports / name for name in ('scene.glb', 'surface.ply', 'cameras.json')]
        sources += list((self.root / 'images').iterdir())
        original = {path: path.read_bytes() for path in sources}
        source_chunks = glb_chunks(original[exports / 'scene.glb'])
        source_document = json.loads(source_chunks[0][1])
        source_bin = next(payload for kind, payload in source_chunks if kind == 0x004E4942)
        result = self.execute()
        for path, payload in original.items():
            self.assertEqual(path.read_bytes(), payload, str(path))
        completed_chunks = glb_chunks(self.artifact(result, 'scene-completed.glb').read_bytes())
        document = json.loads(completed_chunks[0][1])
        completed_bin = next(payload for kind, payload in completed_chunks if kind == 0x004E4942)
        self.assertEqual(completed_bin[:len(source_bin)], source_bin)
        # Existing mesh/image buffer references and photographic material survive intact.
        self.assertEqual(document['meshes'][:len(source_document['meshes'])], source_document['meshes'])
        self.assertEqual(document['nodes'][:len(source_document['nodes'])], source_document['nodes'])
        for before_scene, after_scene in zip(source_document['scenes'], document['scenes']):
            old_roots = before_scene.get('nodes', [])
            self.assertEqual(after_scene.get('nodes', [])[:len(old_roots)], old_roots)
        self.assertEqual(document.get('images'), source_document.get('images'))
        self.assertEqual(document.get('textures'), source_document.get('textures'))
        self.assertEqual(document['materials'][:len(source_document.get('materials', []))], source_document.get('materials', []))
        explicit_labels = json.dumps({'nodes': document.get('nodes'), 'meshes': document.get('meshes')}).lower()
        self.assertIn('infer', explicit_labels)
        self.assertGreater(len(document['meshes']), len(source_document['meshes']))
        self.assertTrue(self.artifact(result, 'inferred.glb').is_file())
        persisted = json.loads(self.artifact(result, 'completion.json').read_text())
        self.assertEqual(persisted['added_triangles'], result['completion']['added_triangles'])

    def test_raw_and_scaled_completion_share_a_frame_and_preserve_calibration(self):
        import numpy as np
        self.fixture(open_prism(self.square))
        self.fixture(open_prism(self.square), source_space='scaled', scale=2.5)
        calibration = (self.root / 'exports/scaled/calibration.json').read_bytes()
        cameras = (self.root / 'exports/scaled/cameras.json').read_bytes()
        raw = self.execute()
        scaled = self.execute('scaled')
        self.assertEqual(raw['completion']['added_triangles'], scaled['completion']['added_triangles'])
        raw_mesh = self.mesh(raw, 'inferred-surfaces.ply')
        scaled_mesh = self.mesh(scaled, 'inferred-surfaces.ply')
        np.testing.assert_allclose(scaled_mesh.bounds, raw_mesh.bounds * 2.5)
        self.assertEqual((self.root / 'exports/scaled/calibration.json').read_bytes(), calibration)
        self.assertEqual((self.root / 'exports/scaled/cameras.json').read_bytes(), cameras)
        self.assertNotEqual(self.artifact(raw, 'scene-completed.glb'), self.artifact(scaled, 'scene-completed.glb'))
        self.assertTrue(self.artifact(scaled, 'scene-completed.glb').is_relative_to(self.root / 'exports/scaled/completed'))

    def test_repeat_completion_is_stable_and_output_is_versioned(self):
        exports = self.fixture(open_prism(self.square))
        original_hash = hashlib.sha256((exports / 'scene.glb').read_bytes()).hexdigest()
        first = self.execute()
        first_path = self.artifact(first, 'scene-completed.glb')
        first_bytes = first_path.read_bytes()
        second = self.execute()
        self.assertEqual(self.artifact(second, 'scene-completed.glb'), first_path)
        self.assertEqual(first_path.read_bytes(), first_bytes)
        self.assertTrue(first_path.is_relative_to(exports / 'completed'))
        self.assertEqual(hashlib.sha256((exports / 'scene.glb').read_bytes()).hexdigest(), original_hash)

    def test_annulus_outer_boundary_is_not_covered_by_duplicate_geometry(self):
        import trimesh
        ring = vertical_opening()
        # Rotate the vertical ring onto a horizontal floor plane.
        ring.vertices = ring.vertices[:, [0, 2, 1]]
        self.fixture(ring)
        result = self.execute()
        self.assertEqual(result['completion']['filled_loops'], 1)
        self.assertEqual(result['completion']['skipped_loops'], 1)
        inferred = self.mesh(result, 'inferred-surfaces.ply')
        self.assertAlmostEqual(float(inferred.area), 4.0, places=6)
        self.assertEqual(result['completion']['topology_after']['boundary_edges'], 4)

    def test_vertical_only_scene_returns_explicit_noop_without_empty_inferred_files(self):
        exports = self.fixture(vertical_opening())
        original = (exports / 'scene.glb').read_bytes()
        result = self.execute()
        self.assertEqual(result['completion']['filled_loops'], 0)
        self.assertEqual(result['completion']['added_triangles'], 0)
        self.assertFalse(result['completion']['contains_inferred_surfaces'])
        self.assertEqual(self.artifact(result, 'scene-completed.glb').read_bytes(), original)
        names = [item['name'] for item in result['artifacts']]
        self.assertNotIn('inferred.glb', names)
        self.assertNotIn('inferred-surfaces.ply', names)
        self.assertTrue(result['warnings'])

    def test_self_intersecting_horizontal_boundaries_are_preserved(self):
        polygon = [[0, 0], [2, 2], [0, 2], [2, 0]]
        self.fixture(open_prism(polygon))
        result = self.execute()
        self.assertEqual(result['completion']['filled_loops'], 0)
        self.assertEqual(result['completion']['added_triangles'], 0)
        self.assertEqual(result['completion']['skipped_loops'], 2)
        self.assertEqual(result['completion']['topology_after']['boundary_edges'], 8)

    def test_output_directory_symlink_is_rejected_without_writing_outside_job(self):
        exports = self.fixture(open_prism(self.square))
        with tempfile.TemporaryDirectory() as external:
            outside = Path(external).resolve()
            (exports / 'completed').symlink_to(outside, target_is_directory=True)
            with self.assertRaises(completion.CompletionError):
                completion.complete_scene(self.root)
            self.assertEqual(list(outside.iterdir()), [])

    def test_distant_closed_geometry_does_not_relax_local_planarity(self):
        import trimesh
        small = [[-.1, -.1], [.1, -.1], [.1, .1], [-.1, .1]]
        saddle = open_prism(small, heights=[2, 2.04, 2, 2.04])
        distant = trimesh.creation.box(extents=[1, 1, 1])
        distant.apply_translation([100, 0, 0])
        self.fixture(trimesh.util.concatenate([saddle, distant]))
        result = self.execute()
        self.assertEqual(result['completion']['filled_loops'], 1)
        self.assertEqual(result['completion']['skipped_loops'], 1)
        inferred = self.mesh(result, 'inferred-surfaces.ply')
        self.assertTrue(all(abs(float(y)) < 1e-6 for y in inferred.vertices[:, 1]))

    def test_scaled_completion_requires_valid_calibration_and_metric_camera_units(self):
        exports = self.fixture(open_prism(self.square), source_space='scaled', scale=2.5)
        calibration = exports / 'calibration.json'
        calibration.unlink()
        with self.assertRaises(completion.CompletionError):
            completion.complete_scene(self.root, source_space='scaled')
        for invalid in (0, -1, True, float('nan'), float('inf')):
            calibration.write_text(json.dumps({'meters_per_unit': invalid, 'units': 'metres'}))
            with self.subTest(factor=invalid), self.assertRaises(completion.CompletionError):
                completion.complete_scene(self.root, source_space='scaled')
        calibration.write_text(json.dumps({'meters_per_unit': 2.5, 'units': 'metres'}))
        cameras = json.loads((exports / 'cameras.json').read_text())
        cameras['units'] = 'unknown'
        (exports / 'cameras.json').write_text(json.dumps(cameras))
        with self.assertRaises(completion.CompletionError):
            completion.complete_scene(self.root, source_space='scaled')
        self.assertFalse((exports / 'completed').exists())

    def test_camera_estimated_up_is_explicit_and_preserves_source_poses(self):
        exports = self.fixture(open_prism(self.square), up_axis=None)
        path = exports / 'cameras.json'
        camera_data = json.loads(path.read_text())
        camera_data['cameras'] = [
            {'center': [index, 1, 1],
             'rotation_world_to_camera': [[1, 0, 0], [0, -1, 0], [0, 0, -1]]}
            for index in range(3)]
        path.write_text(json.dumps(camera_data))
        original = path.read_bytes()
        result = self.execute()
        self.assertEqual(result['completion']['filled_loops'], 2)
        self.assertEqual(result['completion']['up_source'], 'camera-image-up-estimate')
        self.assertEqual(result['completion']['up'], [0, 1, 0])
        self.assertEqual(path.read_bytes(), original)

    def test_unknown_up_and_unsafe_source_space_do_not_produce_arbitrary_closures(self):
        self.fixture(open_prism(self.square), up_axis=None)
        with self.assertRaises(completion.CompletionError):
            completion.complete_scene(self.root)
        self.assertFalse(any((self.root / 'exports').glob('completed/*/inferred-surfaces.ply')))
        for unsafe in ('../', '../scaled', '/tmp', 'completed', None):
            with self.subTest(source_space=unsafe), self.assertRaises(completion.CompletionError):
                completion.complete_scene(self.root, source_space=unsafe)


if __name__ == '__main__':
    unittest.main()
