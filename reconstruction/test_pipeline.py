"""Integration tests use executable fake engines; no camera or rover commands."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import zipfile
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pipeline


FAKE_ENGINE = r'''#!/usr/bin/env python3
import json, os, pathlib, shutil, sqlite3, subprocess, sys, time
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
command = args.pop(0) if name == 'colmap' else name
options = '--database_path --image_path --ImageReader.single_camera --ImageReader.camera_model --FeatureExtraction.use_gpu --FeatureExtraction.num_threads --FeatureExtraction.max_image_size --SiftExtraction.max_num_features --FeatureMatching.use_gpu --FeatureMatching.num_threads --output_path --Mapper.num_threads --input_path --output_type --max_image_size --num_threads --input-file --output-file --working-folder --image-folder --max-threads --gpu-device --resolution-level --max-resolution --min-resolution --number-views --number-views-fuse --scales --reduce-memory --export-type --archive-type --close-holes --max-texture-size --patch-match-cuda-instances'
if not os.environ.get('FAKE_LEGACY_MVS'):
    options += ' --postprocess-dmaps --constant-weight'
if '--help' in args:
    semantics = '' if os.environ.get('FAKE_LEGACY_MVS') or os.environ.get('FAKE_UNDOCUMENTED_MVS') else '\n--postprocess-dmaps: 8 - adjust-confidence; --constant-weight: all view weights 1 instead of the per-view point confidence; disable it only for confidence recalibrated by the densifier'
    print(f'[{time.time()}] COLMAP 4.2.1 / OpenMVS 2.4.0 fake test engine\n' + options + semantics)
    sys.exit(0)
cwd = pathlib.Path.cwd()
with (cwd / 'calls.jsonl').open('a') as log:
    log.write(json.dumps({'command': command, 'args': args}) + '\n')
def get(key): return pathlib.Path(args[args.index(key)+1])
if os.environ.get('FAKE_SLEEP') == command:
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)'])
    (cwd / 'child.pid').write_text(str(child.pid))
    time.sleep(120)
if os.environ.get('FAKE_MISSING') == command:
    sys.exit(0)
if command == 'feature_extractor':
    db=sqlite3.connect(get('--database_path'))
    db.executescript('CREATE TABLE images(image_id INTEGER); CREATE TABLE keypoints(rows INTEGER); CREATE TABLE two_view_geometries(rows INTEGER);')
    db.executemany('INSERT INTO images VALUES (?)', [(1,), (2,), (3,)])
    db.executemany('INSERT INTO keypoints VALUES (?)', [(500,), (500,), (500,)])
    db.commit()
elif command == 'exhaustive_matcher':
    db=sqlite3.connect(get('--database_path')); db.execute('INSERT INTO two_view_geometries VALUES (50)'); db.commit()
elif command == 'mapper':
    model=get('--output_path')/'0'; model.mkdir(parents=True, exist_ok=True); (model/'images.bin').write_bytes(b'model')
elif command == 'model_converter':
    if args[args.index('--output_type')+1] == 'PLY':
        get('--output_path').write_text('ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 0 0\n0 1 0\n')
        sys.exit(0)
    out=get('--output_path'); out.mkdir(parents=True,exist_ok=True)
    (out/'cameras.txt').write_text('1 SIMPLE_RADIAL 100 80 90 50 40 0\n')
    count=int(os.environ.get('FAKE_REGISTERED', '3'))
    (out/'images.txt').write_text(''.join(f'{i} 1 0 0 0 {-i} 0 0 1 photo {i}.png\n0 0 1\n' for i in range(1,count+1)))
    (out/'points3D.txt').write_text(''.join(f'{i} {i} 0 1 255 0 0 0.2 1 0 2 0\n' for i in range(1,6)))
elif command == 'image_undistorter':
    out=get('--output_path'); shutil.copytree(get('--image_path'),out/'images',dirs_exist_ok=True); (out/'sparse').mkdir(exist_ok=True)
else:
    out=get('--output-file'); out.parent.mkdir(parents=True,exist_ok=True); out.write_bytes(b'fake mvs scene')
    ply='ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nelement face 4\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n3 0 2 1\n3 0 1 3\n3 1 2 3\n3 2 0 3\n'
    out.with_suffix('.ply').write_text(ply)
    if command == 'DensifyPointCloud':
        print('Using Metal backend on Apple Test GPU' if args[args.index('--gpu-device')+1] != '-2' else 'Using CPU backend')
        if '--postprocess-dmaps' in args and not os.environ.get('FAKE_NO_CALIBRATION'):
            count = int(os.environ.get('FAKE_CALIBRATION_COUNT', os.environ.get('FAKE_REGISTERED', '3')))
            print(f'Confidence-maps adjusted: {count} depth-maps (test)')
            if os.environ.get('FAKE_CALIBRATION_FAILURE'):
                print('warning: confidence adjustment failed for 1 depth-maps')
    if command == 'TextureMesh':
        stem=out.stem
        out.with_suffix('.obj').write_text('mtllib '+stem+'.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nvt 0 0\nvt 1 0\nvt 0 1\nusemtl wall\nf 1/1 3/3 2/2\nf 1/1 2/2 4/3\nf 2/1 3/2 4/3\nf 3/1 1/2 4/3\n')
        out.with_suffix('.mtl').write_text('newmtl wall\nKd 1 1 1\nmap_Kd '+stem+'0.png\n')
        shutil.copyfile(next((cwd/'images').glob('*.png')), out.parent/(stem+'0.png'))
'''


@unittest.skipUnless(importlib.util.find_spec("trimesh"), "reconstruction dependencies not installed")
class PipelineTests(unittest.TestCase):
    def setUp(self):
        from PIL import Image
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / "images").mkdir()
        for index in range(3):
            Image.new("RGB", (100, 80), (20 + index, 40, 60)).save(self.root / "images" / f"photo {index+1}.png")
        engines = self.root / "engines"
        engines.mkdir()
        self.paths = {}
        for name in pipeline._ENGINES:
            path = engines / name
            path.write_text(FAKE_ENGINE)
            path.chmod(0o755)
            self.paths[name] = str(path)
        self.config = {"engine_paths": self.paths, "minimum_points": 3, "minimum_registered_fraction": 0.6, "backend": "metal", "stage_timeout_seconds": 30}
        self.updates = []

    def tearDown(self):
        self.temp.cleanup()

    def run_job(self, cancel=None):
        return pipeline.run_pipeline(self.root, self.config, self.updates.append, cancel or threading.Event())

    def calls(self):
        path = self.root / "calls.jsonl"
        return [json.loads(line)["command"] for line in path.read_text().splitlines()] if path.exists() else []

    def test_pipeline_order_real_exports_and_validated_resume(self):
        result = self.run_job()
        self.assertEqual(self.calls(), ["feature_extractor", "exhaustive_matcher", "mapper", "model_converter", "model_converter", "image_undistorter",
                                       "InterfaceCOLMAP", "DensifyPointCloud", "ReconstructMesh", "RefineMesh", "TextureMesh"])
        self.assertEqual(result["backend"], {"requested": "metal", "active": "metal"})
        self.assertEqual(result["metrics"]["registered_images"], 3)
        self.assertEqual(result["metrics"]["visual_mesh"]["triangles"], 4)
        commands = {entry["command"]: entry["args"] for entry in map(json.loads, (self.root / "calls.jsonl").read_text().splitlines())}
        self.assertEqual(commands["DensifyPointCloud"][commands["DensifyPointCloud"].index("--patch-match-cuda-instances") + 1], "1")
        self.assertEqual(commands["DensifyPointCloud"][commands["DensifyPointCloud"].index("--archive-type") + 1], "2")
        self.assertEqual(commands["TextureMesh"][commands["TextureMesh"].index("--max-texture-size") + 1], "4096")
        self.assertEqual(commands["TextureMesh"][commands["TextureMesh"].index("--close-holes") + 1], "0")
        self.assertFalse(result["metrics"]["collision_validated"])
        self.assertTrue(all((self.root / item["path"]).stat().st_size for item in result["artifacts"]))
        self.assertTrue(any(item["kind"] == "sparse_points" for item in result["artifacts"]))
        self.assertTrue(any(item["kind"] == "dense_points" for item in result["artifacts"]))
        with zipfile.ZipFile(self.root / "exports" / "scene.zip") as bundle:
            names = bundle.namelist()
            self.assertIn("cameras.json", names)
            self.assertIn("collision.obj", names)
            self.assertTrue(any(name.startswith("visual/") and name.endswith(".obj") for name in names))
            self.assertTrue(any(name.endswith(".mtl") for name in names))
            self.assertTrue(any(name.endswith(".png") for name in names))
            self.assertFalse(any(name.endswith(".zip") or name.startswith("/") or ".." in Path(name).parts for name in names))
        before = self.calls()
        self.run_job()
        self.assertEqual(self.calls(), before)
        self.assertTrue(any(item.get("resumed") for item in self.updates))
        # An export disappearing invalidates only the export stage.
        (self.root / "exports" / "scene.glb").unlink()
        self.run_job()
        self.assertEqual(self.calls(), before)
        self.assertTrue((self.root / "exports" / "scene.glb").exists())
        (self.root / "exports" / "scene.glb").write_bytes(b"corrupted mesh")
        self.run_job()
        self.assertEqual(self.calls(), before)
        self.assertEqual((self.root / "exports" / "scene.glb").read_bytes()[:4], b"glTF")

    def test_calibrated_profile_argv_metrics_and_resume(self):
        self.config["mvs_profile"] = "calibrated"
        result = self.run_job()
        commands = {entry["command"]: entry["args"] for entry in map(json.loads, (self.root / "calls.jsonl").read_text().splitlines())}
        dense, mesh = commands["DensifyPointCloud"], commands["ReconstructMesh"]
        self.assertEqual(dense[dense.index("--postprocess-dmaps") + 1], "8")
        self.assertEqual(mesh[mesh.index("--constant-weight") + 1], "0")
        # Profile selection does not alter depth estimation or camera settings.
        self.assertEqual(dense[dense.index("--resolution-level") + 1], "1")
        self.assertEqual(dense[dense.index("--number-views-fuse") + 1], "3")
        self.assertNotIn("--geometric-iters", dense)
        self.assertEqual(result["metrics"]["mvs_profile"], "calibrated")
        self.assertEqual(result["metrics"]["confidence_adjusted_images"], 3)
        self.assertEqual(result["metrics"]["mvs_profile_settings"]["constant_weight"], 0)
        before = self.calls()
        self.run_job()
        self.assertEqual(self.calls(), before)

    def test_calibrated_uniform_resolves_distinct_settings(self):
        helps = {"DensifyPointCloud": "--postprocess-dmaps: 8 - adjust-confidence",
                 "ReconstructMesh": "--constant-weight: per-view point confidence; disable it only when recalibrated"}
        uniform = pipeline._resolve_mvs_profile("calibrated-uniform", helps)
        weighted = pipeline._resolve_mvs_profile("calibrated", helps)
        self.assertEqual(uniform, {"name": "calibrated-uniform", "postprocess_dmaps": 8,
                                   "constant_weight": 1, "confidence_confirmation_required": True})
        self.assertEqual(weighted["constant_weight"], 0)
        self.assertNotEqual(uniform, weighted)

    def test_calibrated_uniform_actual_argv_preserves_uniform_votes(self):
        self.config["mvs_profile"] = "calibrated-uniform"
        result = self.run_job()
        commands = {entry["command"]: entry["args"] for entry in map(json.loads, (self.root / "calls.jsonl").read_text().splitlines())}
        dense, mesh = commands["DensifyPointCloud"], commands["ReconstructMesh"]
        self.assertEqual(dense[dense.index("--postprocess-dmaps") + 1], "8")
        self.assertEqual(mesh[mesh.index("--constant-weight") + 1], "1")
        self.assertEqual(result["metrics"]["mvs_profile"], "calibrated-uniform")
        self.assertEqual(result["metrics"]["confidence_adjusted_images"], 3)
        self.assertEqual(result["metrics"]["mvs_profile_settings"]["constant_weight"], 1)

    def test_unknown_profile_fails_before_engine_execution(self):
        self.config["mvs_profile"] = "typo"
        with self.assertRaisesRegex(pipeline.PipelineError, "mvs_profile must be"):
            self.run_job()
        self.assertEqual(self.calls(), [])
        self.assertFalse((self.root / "logs").exists())

    def test_calibrated_profile_rejects_unsupported_or_undocumented_helpers(self):
        self.config["mvs_profile"] = "calibrated"
        for env in ({"FAKE_LEGACY_MVS": "1"}, {"FAKE_UNDOCUMENTED_MVS": "1"}):
            with self.subTest(env=env), patch.dict(os.environ, env):
                with self.assertRaisesRegex(pipeline.PipelineError, "requires documented"):
                    self.run_job()
                self.assertEqual(self.calls(), [])

    def test_standard_profile_keeps_legacy_engine_arguments(self):
        with patch.dict(os.environ, {"FAKE_LEGACY_MVS": "1"}):
            result = self.run_job()
        commands = {entry["command"]: entry["args"] for entry in map(json.loads, (self.root / "calls.jsonl").read_text().splitlines())}
        self.assertNotIn("--postprocess-dmaps", commands["DensifyPointCloud"])
        self.assertNotIn("--constant-weight", commands["ReconstructMesh"])
        self.assertEqual(result["metrics"]["mvs_profile"], "standard")

    def test_profile_change_invalidates_cached_outputs(self):
        self.run_job()
        before = json.loads((self.root / "pipeline-state.json").read_text())["fingerprint"]
        self.config["mvs_profile"] = "calibrated"
        self.run_job()
        state = json.loads((self.root / "pipeline-state.json").read_text())
        self.assertNotEqual(before, state["fingerprint"])
        self.assertEqual(state["mvs_profile"]["name"], "calibrated")
        self.assertEqual(self.calls().count("feature_extractor"), 2)
        self.assertEqual(self.calls().count("ReconstructMesh"), 2)

    def test_calibrated_requires_all_images_confirmed_without_failures(self):
        self.config["mvs_profile"] = "calibrated"
        for env in ({"FAKE_NO_CALIBRATION": "1"}, {"FAKE_CALIBRATION_COUNT": "2"}, {"FAKE_CALIBRATION_FAILURE": "1"}):
            with self.subTest(env=env), patch.dict(os.environ, env):
                with self.assertRaisesRegex(pipeline.PipelineError, "successful latest-run confirmation"):
                    self.run_job()
                self.assertNotIn("ReconstructMesh", self.calls())
                self.assertFalse((self.root / ".stage-markers" / "dense.json").exists())

    def test_calibrated_resume_revalidates_latest_dense_confirmation(self):
        self.config["mvs_profile"] = "calibrated"
        self.run_job()
        before_mesh_calls = self.calls().count("ReconstructMesh")
        # A prior successful run must not validate the newest, unconfirmed run.
        log = self.root / "logs" / "dense.log"
        with log.open("a") as stream:
            stream.write('\n$ ["DensifyPointCloud"]\nUsing Metal backend\n')
        with patch.dict(os.environ, {"FAKE_NO_CALIBRATION": "1"}):
            with self.assertRaisesRegex(pipeline.PipelineError, "successful latest-run confirmation"):
                self.run_job()
        self.assertEqual(self.calls().count("DensifyPointCloud"), 2)
        self.assertEqual(self.calls().count("ReconstructMesh"), before_mesh_calls)
        self.assertFalse((self.root / ".stage-markers" / "dense.json").exists())

    def test_missing_output_fails_and_retry_recovers(self):
        with patch.dict(os.environ, {"FAKE_MISSING": "TextureMesh"}):
            with self.assertRaisesRegex(pipeline.PipelineError, "texture did not produce"):
                self.run_job()
        self.assertFalse((self.root / ".stage-markers" / "texture.json").exists())
        self.assertNotIn("export", [event["stage"] for event in self.updates])
        self.run_job()
        self.assertEqual(self.calls().count("feature_extractor"), 1)
        self.assertEqual(self.calls().count("TextureMesh"), 2)

    def test_input_change_invalidates_all_downstream_outputs(self):
        self.run_job()
        from PIL import Image
        Image.new("RGB", (100, 80), (230, 30, 20)).save(self.root / "images" / "photo 1.png")
        self.run_job()
        self.assertEqual(self.calls().count("feature_extractor"), 2)
        self.assertEqual(self.calls().count("TextureMesh"), 2)

    def test_alignment_rejects_too_few_connected_cameras(self):
        with patch.dict(os.environ, {"FAKE_REGISTERED": "2"}):
            with self.assertRaisesRegex(pipeline.PipelineError, "only 2/3"):
                self.run_job()
        self.assertNotIn("DensifyPointCloud", self.calls())

    def test_exif_orientation_normalized_without_modifying_input(self):
        from PIL import Image
        source = self.root / "images" / "rotated.jpg"
        image = Image.new("RGB", (80, 100))
        exif = image.getexif(); exif[274] = 6
        image.save(source, exif=exif)
        original = source.read_bytes()
        self.run_job()
        with Image.open(self.root / "prepared" / "images" / "rotated.jpg") as prepared:
            self.assertEqual(prepared.size, (100, 80))
            self.assertEqual(prepared.getexif().get(274, 1), 1)
        self.assertEqual(source.read_bytes(), original)

    def test_cancel_terminates_process_group_and_leaves_no_marker(self):
        cancel = threading.Event()
        failures = []
        def worker():
            try:
                self.run_job(cancel)
            except Exception as exc:
                failures.append(exc)
        with patch.dict(os.environ, {"FAKE_SLEEP": "DensifyPointCloud"}):
            thread = threading.Thread(target=worker)
            thread.start()
            deadline = time.monotonic() + 10
            while not (self.root / "child.pid").exists() and time.monotonic() < deadline:
                time.sleep(0.03)
            self.assertTrue((self.root / "child.pid").exists())
            child = int((self.root / "child.pid").read_text())
            cancel.set()
            thread.join(5)
            self.assertFalse(thread.is_alive())
            self.assertIsInstance(failures[0], pipeline.PipelineCancelled)
            result = subprocess.run(["ps", "-p", str(child), "-o", "stat="], capture_output=True, text=True)
            self.assertTrue(result.returncode != 0 or "Z" in result.stdout, "child process must be terminated")
        self.assertFalse((self.root / ".stage-markers" / "dense.json").exists())

    def test_stage_timeout_stops_engine(self):
        self.config["stage_timeout_seconds"] = 1
        with patch.dict(os.environ, {"FAKE_SLEEP": "DensifyPointCloud"}):
            with self.assertRaisesRegex(pipeline.PipelineError, "exceeded its 1-second"):
                self.run_job()

    def test_backend_uses_latest_invocation_and_reports_cpu_fallback(self):
        log = self.root / "backend.log"
        log.write_text('\n$ ["DensifyPointCloud"]\nUsing Metal compute backend for depth-map estimation (4 workers)\n'
                       '\n$ ["DensifyPointCloud"]\nWARNING: Metal runtime health check failed; using CPU depth-map estimation\n')
        self.assertEqual(pipeline._backend(log, "metal"), "cpu")


if __name__ == "__main__":
    unittest.main()
