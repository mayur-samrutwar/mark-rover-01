import importlib.util
import json
import struct
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry


@unittest.skipUnless(importlib.util.find_spec("trimesh"), "reconstruction dependencies not installed")
class GeometryTests(unittest.TestCase):
    def setUp(self):
        import trimesh
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.source = self.root / "source.obj"
        trimesh.creation.box(extents=(2, 3, 4)).export(self.source)
        self.result = geometry.export_geometry(self.root, self.source)
        geometry._json(self.root / "exports" / "cameras.json", {"units": "unknown", "cameras": [
            {"center": [1, 2, 3], "translation": [-1, -2, -3], "rotation_world_to_camera": [[1,0,0],[0,1,0],[0,0,1]],
             "native_transform_rows": [[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]]}]})

    def tearDown(self):
        self.temp.cleanup()

    def test_baked_photo_materials_are_diffuse_and_keep_embedded_geometry_and_images(self):
        import numpy as np
        import trimesh
        from PIL import Image
        mesh = trimesh.creation.box()
        mesh.visual = trimesh.visual.TextureVisuals(uv=np.zeros((len(mesh.vertices), 2)),
            material=trimesh.visual.material.SimpleMaterial(image=Image.new("RGB", (2, 2), (180, 120, 60))))
        original = trimesh.Scene(mesh).export(file_type="glb")
        corrected = geometry.baked_colour_glb(original)
        old_size = struct.unpack_from("<I", original, 12)[0]
        new_size = struct.unpack_from("<I", corrected, 12)[0]
        self.assertEqual(original[20 + old_size:], corrected[20 + new_size:])
        document = json.loads(corrected[20:20 + new_size])
        for material in document["materials"]:
            pbr = material["pbrMetallicRoughness"]
            self.assertEqual(pbr["metallicFactor"], 0)
            self.assertEqual(pbr["roughnessFactor"], 1)
            self.assertEqual(pbr["baseColorFactor"], [1, 1, 1, 1])
        path = self.root / "exports/scene.glb"
        size = struct.unpack_from("<I", path.read_bytes(), 12)[0]
        plain = json.loads(path.read_bytes()[20:20 + size])
        for primitive in plain["meshes"][0]["primitives"]:
            self.assertEqual(plain["materials"][primitive["material"]]["pbrMetallicRoughness"]["metallicFactor"], 0)

    def test_topology_ignores_texture_seams_without_modifying_mesh(self):
        import numpy as np
        import trimesh
        box = trimesh.creation.box()
        # Every triangle has its own vertices, as in a UV-split export.
        seam_mesh = trimesh.Trimesh(vertices=box.vertices[box.faces].reshape(-1, 3),
                                   faces=np.arange(36).reshape(-1, 3), process=False)
        before = seam_mesh.vertices.copy()
        topology = geometry.surface_topology(seam_mesh)
        self.assertEqual(topology["components"], 1)
        self.assertEqual(topology["boundary_edges"], 0)
        self.assertEqual(topology["assessment"], "unverified")
        self.assertEqual(topology["room_coverage"], "not measured")
        np.testing.assert_array_equal(seam_mesh.vertices, before)

    def test_fragmented_surface_is_reported_but_not_filled(self):
        import trimesh
        pieces = []
        for index in range(25):
            piece = trimesh.creation.box()
            piece.apply_translation([index * 3, 0, 0])
            pieces.append(piece)
        mesh = trimesh.util.concatenate(pieces)
        mesh.export(self.source)
        result = geometry.export_geometry(self.root, self.source)
        topology = result["metrics"]["surface_topology"]
        self.assertEqual(topology["assessment"], "fragmented")
        self.assertEqual(topology["components"], 25)
        self.assertAlmostEqual(topology["largest_component_area_fraction"], 1/25)
        self.assertTrue(any("Fragmented surface" in warning for warning in result["warnings"]))
        _, exported = geometry._load(self.root / "exports" / "surface.ply")
        self.assertEqual(len(exported.faces), len(mesh.faces))

    def test_open_room_surface_is_not_flagged_for_open_boundary_alone(self):
        import trimesh
        mesh = trimesh.creation.box()
        mesh.update_faces([True] * 10 + [False] * 2)
        topology = geometry.surface_topology(mesh)
        self.assertEqual(topology["components"], 1)
        self.assertGreater(topology["boundary_edges"], 0)
        self.assertEqual(topology["assessment"], "unverified")

    def test_scale_consistent_across_visual_collision_and_cameras(self):
        import trimesh
        raw = (self.root / "exports" / "scene.glb").read_bytes()
        result = geometry.apply_scale(self.root, [0,0,0], [1,0,0], 2)
        self.assertEqual(result["calibration"]["meters_per_unit"], 2)
        for name in ("scene.glb", "surface.ply", "collision.obj"):
            _, mesh = geometry._load(self.root / "exports" / "scaled" / name)
            self.assertEqual(list(mesh.extents), [4,6,8])
        cameras = json.loads((self.root / "exports" / "scaled" / "cameras.json").read_text())
        self.assertEqual(cameras["cameras"][0]["center"], [2,4,6])
        self.assertEqual(cameras["cameras"][0]["translation"], [-2,-4,-6])
        self.assertEqual(cameras["cameras"][0]["native_transform_rows"], [[1,0,0,2],[0,1,0,4],[0,0,1,6],[0,0,0,1]])
        original_cameras = json.loads((self.root / "exports" / "cameras.json").read_text())
        self.assertEqual(original_cameras["cameras"][0]["native_transform_rows"][0][3], 1)
        self.assertEqual((self.root / "exports" / "scene.glb").read_bytes(), raw)
        self.assertFalse(result["metrics"]["collision_validated"])
        self.assertFalse(result["metrics"]["floor_aligned"])

    def test_scaled_viewer_recalibration_does_not_compound_scale(self):
        geometry.apply_scale(self.root, [0,0,0], [1,0,0], 2)
        result = geometry.apply_scale(self.root, [0,0,0], [2,0,0], 3, source_space="scaled")
        self.assertEqual(result["calibration"]["meters_per_unit"], 3)
        _, mesh = geometry._load(self.root / "exports" / "scaled" / "collision.obj")
        self.assertEqual(list(mesh.extents), [6,9,12])

    def test_invalid_measurements_rejected(self):
        cases = [([0,0,0], [0,0,0], 1), ([0,0,0], [float("nan"),0,0], 1),
                 ([0,0,0], [1,0,0], float("inf")), ([0,0,0], [1,0,0], -1),
                 ([0,0], [1,0,0], 1), ([True,0,0], [1,0,0], 1)]
        for a, b, measured in cases:
            with self.subTest(a=a,b=b,measured=measured), self.assertRaises(geometry.GeometryError):
                geometry.apply_scale(self.root, a,b,measured)
        self.assertFalse((self.root / "exports" / "scaled").exists())

    def test_missing_camera_geometry_does_not_partially_export(self):
        (self.root / "exports" / "cameras.json").unlink()
        with self.assertRaisesRegex(geometry.GeometryError, "Camera geometry"):
            geometry.apply_scale(self.root, [0,0,0], [1,0,0], 2)
        self.assertFalse((self.root / "exports" / "scaled").exists())

    def test_collision_candidate_preserves_open_boundaries(self):
        import trimesh
        mesh = trimesh.creation.box()
        mesh.update_faces([True] * 10 + [False] * 2)
        mesh.export(self.source)
        geometry.export_geometry(self.root, self.source)
        _, candidate = geometry._load(self.root / "exports" / "collision.obj")
        self.assertFalse(candidate.is_watertight)
        self.assertEqual(len(candidate.faces), 10)

    def test_collision_candidate_is_decimated_without_replacing_visual_mesh(self):
        import trimesh
        mesh = trimesh.creation.icosphere(subdivisions=3)
        mesh.export(self.source)
        result = geometry.export_geometry(self.root, self.source, collision_faces=200)
        self.assertEqual(result["metrics"]["visual_mesh"]["triangles"], len(mesh.faces))
        self.assertLessEqual(result["metrics"]["collision_mesh"]["triangles"], 200)
        self.assertFalse(result["metrics"]["collision_validated"])


if __name__ == "__main__":
    unittest.main()
