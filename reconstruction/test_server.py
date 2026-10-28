"""Service integration tests use temporary scans and never contact the rover."""
from __future__ import annotations

import hashlib
import http.client
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import sys

from PIL import Image
try:
    from .jobs import JobError, JobStore, atomic_json
    from .server import ReconstructionHTTPServer, make_handler
except ImportError:
    from jobs import JobError, JobStore, atomic_json
    from server import ReconstructionHTTPServer, make_handler


class FakeEngines:
    def inspect(self):
        return {'status': 'ready', 'paths': {}, 'missing': [],
                'engines': {'colmap': {'available': True, 'version': 'test'},
                            'openmvs': {'available': True, 'version': 'test', 'metal_compiled': True}}}


def create_scan(root, count=3):
    scan = root / 'scan-001'
    station = scan / 'position-001'
    station.mkdir(parents=True)
    photos = []
    for index in range(count):
        buffer = io.BytesIO()
        Image.new('RGB', (64, 48), (30 + index * 60, 90, 120)).save(buffer, 'JPEG')
        data = buffer.getvalue()
        name = f'{index:04d}_photo.jpg'
        (station / name).write_bytes(data)
        photos.append({'id': f'photo-{index}', 'filename': name, 'sha256': hashlib.sha256(data).hexdigest(),
                       'bytes': len(data), 'width': 64, 'height': 48})
    atomic_json(scan / 'session.json', {'version': 1, 'id': scan.name, 'created_at': '2026-10-27T00:00:00Z',
                                      'stations': [{'id': station.name, 'status': 'ready', 'photos': photos}]})
    return scan


def fake_runner(workdir, config, update, cancelled):
    update({'stage': 'alignment', 'status': 'running', 'progress': .2})
    if cancelled.wait(.01):
        raise RuntimeError('Cancelled')
    exports = workdir / 'exports'
    exports.mkdir(exist_ok=True)
    (exports / 'model.ply').write_text('ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 0 0\n0 1 0\n')
    update({'stage': 'alignment', 'status': 'completed', 'progress': .8})
    return {'metrics': {'registered_images': 3}, 'backend': {'active': 'metal'},
            'warnings': ['Test geometry'], 'artifacts': [{'name': 'Points', 'path': 'exports/model.ply', 'kind': 'sparse'}]}


class ServiceFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='rover-reconstruction-service-')
        self.root = Path(self.temporary.name)
        self.captures = self.root / 'captures'
        self.scan = create_scan(self.captures)
        self.stores = []

    def tearDown(self):
        for store in self.stores:
            store.close()
        self.temporary.cleanup()

    def store(self, **kwargs):
        store = JobStore(self.root, captures=self.captures, runs=self.root / 'runs',
                         engines=FakeEngines(), runner=fake_runner, **kwargs)
        self.stores.append(store)
        return store

    def wait_terminal(self, store, identifier):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            value = store.get(identifier)
            if value['status'] in {'completed', 'failed', 'cancelled'}:
                return value
            time.sleep(.01)
        self.fail('Job did not finish')


class MVSProfileTests(ServiceFixture):
    def test_processing_profiles_are_recorded_for_worker(self):
        store = self.store(start_worker=False)
        for profile in ('standard', 'calibrated-uniform', 'calibrated'):
            with self.subTest(profile=profile):
                job = store.create({'scan_id': self.scan.name, 'method': 'mvs',
                                    'mvs_profile': profile})
                self.assertEqual(store.jobs[job['id']]['config']['mvs_profile'], profile)

    def test_invalid_profiles_are_rejected_before_queueing(self):
        store = self.store(start_worker=False)
        for profile in ('unknown', ['standard'], None):
            with self.subTest(profile=profile), self.assertRaisesRegex(JobError, 'Unknown MVS'):
                store.create({'scan_id': self.scan.name, 'mvs_profile': profile})
        with self.assertRaisesRegex(JobError, 'require the COLMAP / OpenMVS'):
            store.create({'scan_id': self.scan.name, 'method': 'photogrammetry',
                          'mvs_profile': 'calibrated-uniform'})
        self.assertEqual(store.list(), [])
        self.assertEqual(list(store.runs.iterdir()), [])


class PublicFitMetadataTests(ServiceFixture):
    def test_fit_flag_is_public_without_exposing_private_configuration(self):
        store = self.store(start_worker=False)
        value = store.create({'scan_id': self.scan.name, 'quality': 'full', 'method': 'mvs'})
        identifier = value['id']
        self.assertIs(value['scene_specific_fit'], False)
        for flag, expected in ((True, True), (False, False), ('true', False)):
            store.jobs[identifier]['config']['scene_specific_fit'] = flag
            for public in (store.get(identifier), store.list()[0]):
                self.assertIs(public['scene_specific_fit'], expected)
                self.assertNotIn('config', public)


class NativeJobTests(ServiceFixture):
    class NativeOnlyEngines:
        def inspect(self):
            return {'status': 'ready', 'paths': {'PhotogrammetryReconstruct': '/fixture/native'},
                    'missing': ['colmap'], 'engines': {'photogrammetry': {'available': True},
                    'colmap': {'available': False}, 'openmvs': {'available': False}}}

    def native_store(self, **kwargs):
        store = JobStore(self.root, captures=self.captures, runs=self.root / 'runs',
                         engines=self.NativeOnlyEngines(), **kwargs)
        self.stores.append(store)
        return store

    def test_native_method_does_not_require_classic_engines(self):
        store = self.native_store(start_worker=False)
        job = store.create({'scan_id': self.scan.name, 'quality': 'full', 'method': 'photogrammetry'})
        self.assertEqual(job['method'], 'photogrammetry')
        self.assertEqual(job['backend']['requested'], 'apple')
        self.assertEqual(store.jobs[job['id']]['config']['native_engine_path'], '/fixture/native')
        self.assertFalse(any('CPU' in value for value in job['warnings']))
        with self.assertRaisesRegex(JobError, 'COLMAP and OpenMVS'):
            store.create({'scan_id': self.scan.name, 'method': 'mvs'})

    def test_unavailable_or_unknown_method_is_rejected_before_queueing(self):
        store = self.store(start_worker=False)
        for method, message in [('photogrammetry', 'unavailable'), ('unknown', 'Unknown')]:
            with self.subTest(method=method), self.assertRaisesRegex(JobError, message):
                store.create({'scan_id': self.scan.name, 'method': method})
        self.assertEqual(store.list(), [])

    def test_worker_dispatches_native_after_verified_snapshot(self):
        calls = []
        def native_runner(workdir, config, update, cancelled):
            calls.append(config['method'])
            self.assertEqual(len(list((workdir / 'images').glob('*.jpg'))), 3)
            self.assertTrue((workdir / 'input-manifest.json').is_file())
            exports = workdir / 'exports'
            exports.mkdir()
            (exports / 'model.ply').write_text('ply\nformat ascii 1.0\nend_header\n')
            update({'stage': 'native', 'status': 'completed'})
            return {'backend': {'active': 'apple'}, 'artifacts': [{'path': 'exports/model.ply', 'kind': 'surface_mesh'}]}
        module = sys.modules[JobStore.__module__]
        with patch.object(module, 'native_module', return_value=SimpleNamespace(
                NATIVE_STAGES=[{'id': 'native', 'label': 'Photogrammetry'}], run_native=native_runner)):
            store = self.native_store()
            job = store.create({'scan_id': self.scan.name, 'quality': 'full', 'method': 'photogrammetry'})
            result = self.wait_terminal(store, job['id'])
        self.assertEqual(result['status'], 'completed', result.get('error'))
        self.assertEqual(calls, ['photogrammetry'])
        self.assertEqual(result['backend']['active'], 'apple')
        self.assertEqual([stage['id'] for stage in result['stages']], ['snapshot', 'native'])


class JobTests(ServiceFixture):
    def test_scan_and_full_snapshot_job(self):
        store = self.store()
        self.assertEqual(store.scans()[0]['photo_count'], 3)
        self.assertTrue(store.scans()[0]['usable'])
        before = {p.name: p.read_bytes() for p in self.scan.rglob('*.jpg')}
        job = store.create({'scan_id': self.scan.name, 'quality': 'draft'})
        result = self.wait_terminal(store, job['id'])
        self.assertEqual(result['status'], 'completed', result.get('error'))
        self.assertEqual(result['backend']['active'], 'metal')
        self.assertEqual(result['calibration']['status'], 'unscaled')
        self.assertFalse(result['calibration']['collision_validated'])
        self.assertEqual(result['metrics']['registered_images'], 3)
        self.assertEqual({p.name: p.read_bytes() for p in self.scan.rglob('*.jpg')}, before)
        snapshot = store.runs / job['id']
        self.assertEqual(len(list((snapshot / 'images').glob('*.jpg'))), 3)
        self.assertTrue((snapshot / 'input-manifest.json').is_file())
        self.assertTrue(result['artifacts'][0]['url'].startswith('/artifacts/'))
        self.assertNotIn('config', result)

    def test_full_run_uses_cpu_and_draft_retains_metal(self):
        store = self.store(start_worker=False)
        full = store.create({'scan_id': self.scan.name, 'quality': 'full'})
        draft = store.create({'scan_id': self.scan.name, 'quality': 'draft'})
        self.assertEqual(full['backend']['requested'], 'cpu')
        self.assertEqual(draft['backend']['requested'], 'metal')
        self.assertFalse(any('not been verified' in warning for warning in full['warnings']))

    def test_insufficient_or_duplicate_views_rejected(self):
        store = self.store(start_worker=False)
        manifest = self.scan / 'session.json'
        value = json.loads(manifest.read_text())
        value['stations'][0]['photos'] = value['stations'][0]['photos'][:2]
        atomic_json(manifest, value)
        self.assertFalse(store.scans()[0]['usable'])
        with self.assertRaises(JobError):
            store.create({'scan_id': self.scan.name})
        value['stations'][0]['photos'].append(dict(value['stations'][0]['photos'][0]))
        atomic_json(manifest, value)
        with self.assertRaises(JobError):
            store.create({'scan_id': self.scan.name})

    def test_source_checksum_change_does_not_modify_original(self):
        store = self.store(start_worker=False)
        job = store.create({'scan_id': self.scan.name})
        image = next(self.scan.rglob('*.jpg'))
        data = image.read_bytes()
        image.write_bytes(b'x' * len(data))
        with self.assertRaisesRegex(JobError, 'checksum'):
            store._snapshot(job['id'], threading.Event())
        self.assertEqual(image.read_bytes(), b'x' * len(data))

    def test_symlink_and_traversal_rejected(self):
        store = self.store(start_worker=False)
        image = next(self.scan.rglob('*.jpg'))
        outside = self.root / 'outside.jpg'
        outside.write_bytes(image.read_bytes())
        image.unlink()
        image.symlink_to(outside)
        self.assertFalse(store.scans()[0]['usable'])
        with self.assertRaises(JobError):
            store.create({'scan_id': '../captures'})

    def test_queued_cancel_and_retry(self):
        store = self.store(start_worker=False)
        job = store.create({'scan_id': self.scan.name})
        self.assertEqual(store.cancel(job['id'])['status'], 'cancelled')
        self.assertEqual(store.retry(job['id'])['status'], 'queued')
        with self.assertRaises(JobError):
            store.retry(job['id'])

    def test_running_cancel_is_observed(self):
        started = threading.Event()
        def waiting(workdir, config, update, cancelled):
            started.set()
            cancelled.wait(3)
            raise RuntimeError('Cancelled')
        store = self.store()
        store.runner = waiting
        job = store.create({'scan_id': self.scan.name})
        self.assertTrue(started.wait(3))
        store.cancel(job['id'])
        self.assertEqual(self.wait_terminal(store, job['id'])['status'], 'cancelled')

    def test_restore_marks_incomplete_job_interrupted(self):
        store = self.store(start_worker=False)
        job = store.create({'scan_id': self.scan.name})
        restored = self.store(start_worker=False)
        value = restored.get(job['id'])
        self.assertEqual(value['status'], 'interrupted')
        self.assertIn('Retry', value['error'])

    def test_queue_limit_and_quality_validation(self):
        store = self.store(start_worker=False)
        with self.assertRaises(JobError):
            store.create({'scan_id': self.scan.name, 'quality': 'arbitrary-flag'})
        for _ in range(3):
            store.create({'scan_id': self.scan.name})
        with self.assertRaisesRegex(JobError, 'Three'):
            store.create({'scan_id': self.scan.name})

    def test_snapshot_reused_after_source_removed(self):
        store = self.store(start_worker=False)
        job = store.create({'scan_id': self.scan.name})
        store._snapshot(job['id'], threading.Event())
        image = next(self.scan.rglob('*.jpg'))
        image.unlink()
        store._snapshot(job['id'], threading.Event())
        self.assertEqual(len(list((store.runs / job['id'] / 'images').glob('*.jpg'))), 3)

    def test_scale_input_rejected_and_private_images_not_served(self):
        store = self.store(start_worker=False)
        job = store.create({'scan_id': self.scan.name})
        for values in [dict(point_a=[0,0,0],point_b=[1,0,0],distance_m=float('nan')),
                       dict(point_a=[0,0,0],point_b=[0,0,0],distance_m=1),
                       dict(point_a=[0,0,0],point_b=[1,0,0],distance_m=True)]:
            with self.assertRaises(JobError):
                store.scale(job['id'], values)
        with self.assertRaises(JobError):
            store.artifact(job['id'], 'source-plan.json')
        with self.assertRaises(JobError):
            store.artifact(job['id'], '../job.json')


class CompletionTests(ServiceFixture):
    class CompletionError(Exception):
        pass

    def completed_source(self):
        store = self.store()
        job = self.wait_terminal(store, store.create({'scan_id': self.scan.name})['id'])
        exports = store.runs / job['id'] / 'exports'
        for name in ('scene.glb', 'surface.ply', 'cameras.json'):
            (exports / name).write_bytes(b'fixture original ' + name.encode())
            store.jobs[job['id']]['artifacts'].append({'path': 'exports/' + name})
        store._save(store.jobs[job['id']])
        return store, job['id'], exports

    def completion_module(self, callback=None):
        def complete_scene(workdir, source_space='raw'):
            if callback:
                callback(workdir, source_space)
            relative = 'exports/' + ('scaled/' if source_space == 'scaled' else '') + 'completed/fixture-v1/'
            output = workdir / relative
            output.mkdir(parents=True, exist_ok=True)
            report = {'source_space': source_space, 'filled_loops': 2, 'skipped_loops': 3,
                      'collision_validated': False, 'source_sha256': 'fixture'}
            artifacts = []
            for name, kind in [('scene-completed.glb', 'completed_visual_mesh'),
                               ('surface-completed.ply', 'completed_surface_mesh'),
                               ('inferred-surfaces.ply', 'inferred_surface'),
                               ('completion.json', 'completion_report')]:
                (output / name).write_text(json.dumps(report) if name.endswith('.json') else 'fixture inferred mesh')
                artifacts.append({'path': relative + name, 'kind': kind})
            return {'completion': report, 'artifacts': artifacts, 'warnings': ['Inferred surfaces are unvalidated.']}
        return SimpleNamespace(CompletionError=self.CompletionError, complete_scene=complete_scene)

    def test_completion_is_separate_persisted_and_preserves_source(self):
        store, identifier, exports = self.completed_source()
        before = {p.name: p.read_bytes() for p in exports.iterdir() if p.is_file()}
        module = sys.modules[JobStore.__module__]
        with patch.object(module, 'completion_module', return_value=self.completion_module()):
            result = store.complete(identifier, {'source_space': 'raw'})
        self.assertEqual({name: (exports / name).read_bytes() for name in before}, before)
        self.assertEqual(result['completion']['raw']['filled_loops'], 2)
        self.assertFalse(result['completion']['raw']['collision_validated'])
        self.assertFalse(result['completing'])
        completed = [a for a in result['artifacts'] if '/completed/' in a['path']]
        self.assertEqual(len(completed), 4)
        self.assertTrue(all(a['inferred'] and not a['collision_validated'] for a in completed))
        self.assertTrue(any(a['path'] == 'exports/scene.glb' for a in result['artifacts']))
        # Repeating completion replaces only its own registered variant.
        with patch.object(module, 'completion_module', return_value=self.completion_module()):
            result = store.complete(identifier, {'source_space': 'raw'})
        self.assertEqual(len([a for a in result['artifacts'] if '/completed/' in a['path']]), 4)
        restored = self.store(start_worker=False)
        self.assertEqual(restored.get(identifier)['completion']['raw'], result['completion']['raw'])
        self.assertFalse(restored.get(identifier)['completing'])

    def test_zero_fill_registers_no_inferred_surface_and_reports_no_invention(self):
        store, identifier, _ = self.completed_source()
        helper = self.completion_module()
        def zero(workdir, source_space='raw'):
            result = helper.complete_scene(workdir, source_space)
            result['completion'].update(filled_loops=0, contains_inferred_surfaces=False)
            result['artifacts'] = [{**entry, 'contains_inferred': False} for entry in result['artifacts']
                                   if entry['kind'] != 'inferred_surface']
            return result
        with patch.object(sys.modules[JobStore.__module__], 'completion_module', return_value=SimpleNamespace(CompletionError=self.CompletionError, complete_scene=zero)):
            result = store.complete(identifier, {})
        self.assertEqual(result['completion']['raw']['filled_loops'], 0)
        self.assertFalse(result['completion']['raw']['contains_inferred_surfaces'])
        self.assertFalse(any(entry.get('kind') == 'inferred_surface' for entry in result['artifacts']))
        completed = next(entry for entry in result['artifacts'] if entry.get('kind') == 'completed_visual_mesh')
        self.assertFalse(completed['contains_inferred'])
        self.assertFalse(completed['inferred'])

    def test_status_space_missing_inputs_and_symlinks_are_rejected(self):
        store, identifier, exports = self.completed_source()
        with self.assertRaises(JobError) as error:
            store.complete('missing', {})
        self.assertEqual(error.exception.status, 404)
        for space in ('../raw', 'exports/scaled', None):
            with self.assertRaises(JobError):
                store.complete(identifier, {'source_space': space})
        with self.assertRaisesRegex(JobError, 'no scaled'):
            store.complete(identifier, {'source_space': 'scaled'})
        store.jobs[identifier]['status'] = 'failed'
        with self.assertRaises(JobError) as error:
            store.complete(identifier, {})
        self.assertEqual(error.exception.status, 409)
        store.jobs[identifier]['status'] = 'completed'
        (exports / 'scene.glb').unlink()
        with self.assertRaises(JobError) as error:
            store.complete(identifier, {})
        self.assertEqual(error.exception.status, 404)
        outside = self.root / 'outside.glb'
        outside.write_bytes(b'outside')
        (exports / 'scene.glb').symlink_to(outside)
        with self.assertRaisesRegex(JobError, 'path|links'):
            store.complete(identifier, {})
        self.assertFalse(store.get(identifier)['completing'])

    def test_scale_and_completion_are_mutually_exclusive(self):
        store, identifier, _ = self.completed_source()
        module = sys.modules[JobStore.__module__]
        started = threading.Event()
        finish = threading.Event()
        errors = []
        def blocking(workdir, space):
            started.set()
            if not finish.wait(5):
                raise RuntimeError('Fixture timed out')
        def execute():
            try:
                store.complete(identifier, {})
            except Exception as error:
                errors.append(error)
        with patch.object(module, 'completion_module', return_value=self.completion_module(blocking)):
            thread = threading.Thread(target=execute)
            thread.start()
            try:
                self.assertTrue(started.wait(2))
                self.assertTrue(store.get(identifier)['completing'])
                self.assertFalse(store.get(identifier)['scaling'])
                for action, values in [(store.complete, {}), (store.scale, {'point_a': [0,0,0], 'point_b': [1,0,0], 'distance_m': 1})]:
                    with self.assertRaises(JobError) as error:
                        action(identifier, values)
                    self.assertEqual(error.exception.status, 409)
            finally:
                finish.set()
                thread.join(5)
        self.assertFalse(errors, errors)
        store.scaling.add(identifier)
        with self.assertRaises(JobError) as error:
            store.complete(identifier, {})
        self.assertEqual(error.exception.status, 409)
        store.scaling.clear()
        self.assertFalse(store.get(identifier)['completing'])

    def test_algorithm_error_and_unsafe_outputs_do_not_register_artifacts(self):
        store, identifier, _ = self.completed_source()
        module = sys.modules[JobStore.__module__]
        def fail(*args, **kwargs):
            raise self.CompletionError('No supported boundary loops.')
        with patch.object(module, 'completion_module', return_value=SimpleNamespace(CompletionError=self.CompletionError, complete_scene=fail)):
            with self.assertRaisesRegex(JobError, 'supported boundary'):
                store.complete(identifier, {})
        def invalid(*args, **kwargs):
            return {'completion': {'source_space': 'raw'}, 'artifacts': [{'path': 'exports/../job.json'}]}
        with patch.object(module, 'completion_module', return_value=SimpleNamespace(CompletionError=self.CompletionError, complete_scene=invalid)):
            with self.assertRaisesRegex(JobError, 'invalid export path'):
                store.complete(identifier, {})
        self.assertFalse(store.get(identifier)['completing'])
        self.assertNotIn('completion', store.get(identifier))


class HTTPTests(ServiceFixture):
    def setUp(self):
        super().setUp()
        self.backend = self.store()
        self.server = ReconstructionHTTPServer(('127.0.0.1', 0), make_handler(self.backend, 0))
        self.port = self.server.server_port
        self.server.RequestHandlerClass = make_handler(self.backend, self.port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        super().tearDown()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        data = json.dumps(body) if body is not None else None
        request_headers = {'Content-Type': 'application/json'} if body is not None else {}
        request_headers.update(headers or {})
        connection.request(method, path, data, request_headers)
        response = connection.getresponse()
        result = response.status, dict(response.getheaders()), response.read()
        connection.close()
        return result

    def test_reconstruction_port_cannot_be_shared_by_another_listener(self):
        with self.assertRaises(OSError):
            ReconstructionHTTPServer(('127.0.0.1', self.port), make_handler(self.backend, self.port))

    def test_http_scan_job_export_and_headers(self):
        status, headers, data = self.request('GET', '/api/scans')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(data)['scans'][0]['photo_count'], 3)
        self.assertEqual(headers['X-Content-Type-Options'], 'nosniff')
        status, _, data = self.request('POST', '/api/jobs', {'scan_id': self.scan.name, 'quality': 'draft'})
        self.assertEqual(status, 202)
        job = self.wait_terminal(self.backend, json.loads(data)['job']['id'])
        status, _, data = self.request('GET', job['artifacts'][0]['url'])
        self.assertEqual(status, 200)
        self.assertTrue(data.startswith(b'ply\n'))
        status, _, _ = self.request('GET', '/artifacts/' + job['id'] + '/images/test.jpg')
        self.assertEqual(status, 404)

    def test_http_scale_exports_and_recalibration_use_correct_coordinates(self):
        import trimesh
        try:
            from . import geometry
        except ImportError:
            import geometry
        def mesh_runner(workdir, config, update, cancelled):
            source = workdir / 'source.obj'
            source.write_text(trimesh.creation.box().export(file_type='obj'))
            result = geometry.export_geometry(workdir, source)
            atomic_json(workdir / 'exports/cameras.json', {'cameras': [{'center': [1, 0, 0], 'translation': [-1, 0, 0]}]})
            result['backend'] = {'active': 'cpu'}
            return result
        self.backend.runner = mesh_runner
        job = self.backend.create({'scan_id': self.scan.name})
        completed = self.wait_terminal(self.backend, job['id'])
        self.assertEqual(completed['status'], 'completed', completed.get('error'))
        path = '/api/jobs/' + job['id'] + '/scale'
        status, _, body = self.request('POST', path, {'point_a': [0,0,0], 'point_b': [1,0,0], 'distance_m': 2})
        self.assertEqual(status, 200, body)
        scaled = json.loads(body)['job']
        self.assertEqual(scaled['calibration']['status'], 'scaled')
        self.assertEqual(scaled['calibration']['meters_per_unit'], 2)
        self.assertTrue(scaled['metrics']['metric_scale_known'])
        self.assertFalse(scaled['calibration']['collision_validated'])
        self.assertTrue(any('one measurement' in warning for warning in scaled['warnings']))
        saved = self.backend.jobs[job['id']]
        saved['completion'] = {'raw': {'source_space': 'raw'}, 'scaled': {'source_space': 'scaled'}}
        for space in ('raw', 'scaled'):
            relative = 'exports/' + ('scaled/' if space == 'scaled' else '') + 'completed/scene-completed.glb'
            file = self.backend.runs / job['id'] / relative
            file.parent.mkdir(parents=True)
            file.write_bytes(b'old completion')
            saved['artifacts'].append({'path': relative, 'kind': 'completed_visual_mesh'})
        status, _, body = self.request('POST', path, {'point_a': [0,0,0], 'point_b': [2,0,0], 'distance_m': 3, 'source_space': 'scaled'})
        self.assertEqual(status, 200, body)
        self.assertEqual(json.loads(body)['job']['calibration']['meters_per_unit'], 3)
        recalibrated = json.loads(body)['job']
        self.assertIn('raw', recalibrated['completion'])
        self.assertNotIn('scaled', recalibrated['completion'])
        self.assertTrue(any(a['path'] == 'exports/completed/scene-completed.glb' for a in recalibrated['artifacts']))
        self.assertFalse(any(a['path'].startswith('exports/scaled/completed/') for a in recalibrated['artifacts']))
        self.assertFalse((self.backend.runs / job['id'] / 'exports/scaled/completed').exists())

    def test_http_completion_dispatches_scaled_space_and_exposes_outputs(self):
        job = self.wait_terminal(self.backend, self.backend.create({'scan_id': self.scan.name})['id'])
        identifier = job['id']
        original = self.backend.jobs[identifier]
        original['calibration'] = {'status': 'scaled', 'meters_per_unit': 2}
        exports = self.backend.runs / identifier / 'exports/scaled'
        exports.mkdir(parents=True)
        for name in ('scene.glb', 'surface.ply', 'cameras.json'):
            (exports / name).write_text('scaled fixture source')
        calls = []
        fixture = CompletionTests()
        helper = fixture.completion_module(lambda workdir, source_space: calls.append((workdir, source_space)))
        with patch.object(sys.modules[JobStore.__module__], 'completion_module', return_value=helper):
            status, _, body = self.request('POST', '/api/jobs/' + identifier + '/complete', {'source_space': 'scaled'})
        self.assertEqual(status, 200, body)
        result = json.loads(body)['job']
        self.assertEqual(calls, [(self.backend.runs / identifier, 'scaled')])
        self.assertFalse(result['completing'])
        self.assertEqual(result['completion']['scaled']['source_space'], 'scaled')
        self.assertNotIn('raw', result['completion'])
        artifact = next(a for a in result['artifacts'] if a.get('kind') == 'completed_visual_mesh')
        self.assertEqual(artifact['units'], 'metres')
        self.assertIn('/exports/scaled/completed/', artifact['url'])
        self.assertEqual(self.request('GET', artifact['url'])[0], 200)
        self.assertEqual(self.request('POST', '/api/jobs/missing/complete', {})[0], 404)
        self.assertEqual(self.request('POST', '/api/jobs/' + identifier + '/complete', {'source_space': '../raw'})[0], 400)

    def test_host_origin_body_and_url_boundaries(self):
        self.assertEqual(self.request('GET', '/api/scans', headers={'Host': 'attacker.example'})[0], 403)
        self.assertEqual(self.request('POST', '/api/jobs', {}, {'Origin': 'https://attacker.example'})[0], 403)
        self.assertEqual(self.request('GET', '/%2e%2e/jobs.py')[0], 400)
        self.assertEqual(self.request('GET', '/server.py')[0], 404)
        self.assertEqual(self.request('GET', '/api/unknown')[0], 404)
        self.assertEqual(self.request('POST', '/api/jobs', {}, {'Content-Type': 'text/plain'})[0], 415)
        self.assertEqual(self.request('OPTIONS', '/api/jobs')[0], 405)


if __name__ == '__main__':
    unittest.main()
