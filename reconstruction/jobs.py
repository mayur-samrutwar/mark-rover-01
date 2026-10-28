"""Local reconstruction jobs and immutable scan snapshots."""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import queue
import re
import secrets
import shutil
import subprocess
import threading
import time

IDENTIFIER = re.compile(r'^[a-zA-Z0-9_-]{1,80}$')
TERMINAL = {'completed', 'failed', 'cancelled', 'interrupted'}
MAX_MANIFEST = 32 * 1024 * 1024
ENGINE_NAMES = ('colmap', 'InterfaceCOLMAP', 'DensifyPointCloud', 'ReconstructMesh', 'RefineMesh', 'TextureMesh')
DEFAULT_STAGES = [
    {'id': 'snapshot', 'label': 'Snapshot'},
    {'id': 'alignment', 'label': 'Camera alignment'},
    {'id': 'dense', 'label': 'Dense geometry'},
    {'id': 'mesh', 'label': 'Surface reconstruction'},
    {'id': 'refine', 'label': 'Mesh refinement'},
    {'id': 'texture', 'label': 'Texturing'},
    {'id': 'export', 'label': 'Exports'},
]


class JobError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def read_json(path):
    if path.stat().st_size > MAX_MANIFEST:
        raise ValueError('Manifest is too large.')
    def invalid(value):
        raise ValueError('Non-finite JSON number.')
    return json.loads(path.read_text(encoding='utf-8'), parse_constant=invalid)


def safe_child(root, *parts):
    root = Path(root).resolve()
    candidate = root.joinpath(*parts)
    if candidate.is_symlink() or not candidate.resolve().is_relative_to(root):
        raise JobError('Invalid storage path.', 400)
    # Reject parent symlinks even when they lead back inside the directory.
    current = candidate
    while current != root:
        if current.is_symlink():
            raise JobError('Symbolic links are not accepted.', 400)
        current = current.parent
        if not current.is_relative_to(root):
            raise JobError('Invalid storage path.', 400)
    return candidate


def file_sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def pipeline_module():
    return importlib.import_module((__package__ + '.pipeline') if __package__ else 'pipeline')


def native_module():
    return importlib.import_module((__package__ + '.native') if __package__ else 'native')


def completion_module():
    return importlib.import_module((__package__ + '.completion') if __package__ else 'completion')


class EngineRegistry:
    """Cached, read-only capability discovery; never installs tools on a request."""
    def __init__(self, project, overrides=None):
        self.project = Path(project)
        self.overrides = overrides or {}
        self._cached = None
        self._at = 0
        self._lock = threading.Lock()

    def inspect(self, refresh=False):
        with self._lock:
            if self._cached is not None and not refresh and time.monotonic() - self._at < 15:
                return copy.deepcopy(self._cached)
            paths = {}
            versions = {}
            runtime = self.project / '.reconstruction-tools'
            probe_directory = runtime / 'logs' / 'health'
            probe_directory.mkdir(parents=True, exist_ok=True)
            try:
                manifest = read_json(runtime / 'toolchain.json')
            except (OSError, ValueError, TypeError):
                manifest = {}
            for name in ENGINE_NAMES:
                candidate = self.overrides.get(name) or os.environ.get('ROVER_ENGINE_' + name.upper())
                candidate = candidate or str(runtime / 'bin' / name)
                if not Path(candidate).is_file() or not os.access(candidate, os.X_OK):
                    candidate = shutil.which(name)
                if candidate:
                    paths[name] = str(Path(candidate).absolute())
            # Version probing only when tools are present. -h is a bounded no-op.
            for name in ('colmap', 'DensifyPointCloud'):
                if name in paths:
                    try:
                        result = subprocess.run([paths[name], '-h'], stdout=subprocess.PIPE,
                                                stderr=subprocess.STDOUT, text=True, timeout=8,
                                                env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen'}, cwd=probe_directory)
                        output = result.stdout[:20000]
                        versions[name] = next((line.strip()[:180] for line in output.splitlines()
                                              if ('COLMAP' if name == 'colmap' else 'OpenMVS') in line), '')
                        if result.returncode not in (0, 1):
                            paths.pop(name, None)
                    except (OSError, subprocess.TimeoutExpired):
                        paths.pop(name, None)
            mvs = manifest.get('openmvs', {})
            validation = manifest.get('validation', {})
            metal_compiled = bool(mvs.get('metal_compiled', False))
            native_path = self.overrides.get('PhotogrammetryReconstruct') or os.environ.get('ROVER_ENGINE_PHOTOGRAMMETRYRECONSTRUCT') or str(runtime / 'bin/PhotogrammetryReconstruct')
            native = {'available': False, 'supported': False}
            if Path(native_path).is_file() and os.access(native_path, os.X_OK):
                try:
                    probe = subprocess.run([native_path, '--probe'], stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE, text=True, timeout=8)
                    info = json.loads(probe.stdout)
                    native = {**info, 'available': probe.returncode == 0 and info.get('supported') is True}
                    if native['available']:
                        paths['PhotogrammetryReconstruct'] = str(Path(native_path).absolute())
                except (OSError, ValueError, subprocess.TimeoutExpired):
                    pass
            missing = [name for name in ENGINE_NAMES if name not in paths]
            self._cached = {
                'status': 'ready' if not missing or native['available'] else 'missing-tools',
                'engines': {
                    'photogrammetry': native,
                    'colmap': {'available': 'colmap' in paths,
                               'version': versions.get('colmap') or manifest.get('colmap', {}).get('version')},
                    'openmvs': {'available': not any(name != 'colmap' for name in missing),
                                'version': versions.get('DensifyPointCloud') or mvs.get('version'),
                                'revision': mvs.get('revision'), 'metal_compiled': metal_compiled,
                                'metal_smoke_tested': bool(validation.get('metal_active', False))},
                },
                'missing': missing, 'paths': paths,
            }
            self._at = time.monotonic()
            return copy.deepcopy(self._cached)


class JobStore:
    def __init__(self, project, *, captures=None, runs=None, engines=None, runner=None, start_worker=True):
        self.project = Path(project).resolve()
        self.captures = Path(captures or self.project / 'reconstruction/captures').resolve()
        self.runs = Path(runs or self.project / 'reconstruction/runs').resolve()
        self.runs.mkdir(parents=True, exist_ok=True)
        self.engines = engines or EngineRegistry(self.project)
        self.runner = runner
        self.lock = threading.RLock()
        self.jobs = {}
        self.cancel_events = {}
        self.scaling = set()
        self.completing = set()
        self.queue = queue.Queue()
        self.closing = threading.Event()
        self.worker = None
        self._restore()
        if start_worker:
            self.worker = threading.Thread(target=self._worker, name='reconstruction-worker', daemon=True)
            self.worker.start()

    def _restore(self):
        for folder in sorted(self.runs.iterdir()):
            if not folder.is_dir() or folder.is_symlink() or not IDENTIFIER.fullmatch(folder.name):
                continue
            try:
                value = read_json(folder / 'job.json')
                if value['id'] != folder.name or value.get('version') != 1:
                    continue
                if value['status'] not in TERMINAL:
                    value.update(status='interrupted', error='The server stopped during this run. Retry resumes verified stages.', updated_at=now())
                    for stage in value.get('stages', []):
                        if stage.get('status') == 'running':
                            stage['status'] = 'interrupted'
                    atomic_json(folder / 'job.json', value)
                self.jobs[value['id']] = value
                self.cancel_events[value['id']] = threading.Event()
            except (OSError, ValueError, KeyError, TypeError):
                continue

    def _save(self, job):
        job['updated_at'] = now()
        atomic_json(self.runs / job['id'] / 'job.json', job)

    def _source(self, scan_id):
        if not isinstance(scan_id, str) or not IDENTIFIER.fullmatch(scan_id):
            raise JobError('Choose a saved scan.')
        folder = safe_child(self.captures, scan_id)
        try:
            value = read_json(safe_child(folder, 'session.json'))
        except (OSError, ValueError, TypeError):
            raise JobError('The scan manifest could not be read.', 404)
        if not isinstance(value, dict) or value.get('id') != scan_id or value.get('version') != 1:
            raise JobError('The scan manifest is invalid.')
        photos = []
        known = set()
        stations = value.get('stations', [])
        if not isinstance(stations, list):
            raise JobError('The scan positions are invalid.')
        for station in stations:
            if not isinstance(station, dict) or not isinstance(station.get('photos', []), list):
                raise JobError('The scan positions are invalid.')
            identifier = station.get('id')
            if not isinstance(identifier, str) or not IDENTIFIER.fullmatch(identifier):
                raise JobError('Invalid capture position.')
            for photo in station.get('photos', []):
                if not isinstance(photo, dict):
                    raise JobError('The captured image record is invalid.')
                name = photo.get('filename')
                if not isinstance(name, str) or Path(name).name != name or not name.lower().endswith(('.jpg', '.jpeg')) or '.thumb.' in name:
                    raise JobError('Invalid captured image filename.')
                path = safe_child(folder, identifier, name)
                if not path.is_file():
                    raise JobError('A captured image is missing. The scan has not been modified.')
                key = identifier + '/' + name
                if key in known:
                    raise JobError('A captured image is listed twice.')
                known.add(key)
                digest = photo.get('sha256')
                if not isinstance(digest, str) or not re.fullmatch(r'[a-f0-9]{64}', digest):
                    raise JobError('A captured image has no valid checksum.')
                photos.append({'source': key, 'filename': identifier + '_' + name,
                               'sha256': digest, 'bytes': path.stat().st_size,
                               'width': photo.get('width'), 'height': photo.get('height'),
                               'camera_settings': (photo.get('metadata') or {}).get('camera_settings', {})})
        return folder, value, photos

    def scans(self):
        result = []
        if not self.captures.exists():
            return result
        for folder in sorted(self.captures.iterdir(), reverse=True):
            if not folder.is_dir() or folder.is_symlink() or not IDENTIFIER.fullmatch(folder.name):
                continue
            try:
                _, value, photos = self._source(folder.name)
                stations = value.get('stations', [])
                item = {'id': value['id'], 'created_at': value.get('created_at'),
                        'photo_count': len(photos), 'position_count': len(stations),
                        'finished_positions': sum(s.get('status') == 'ready' for s in stations),
                        'usable': len(photos) >= 3}
                if len(photos) < 3:
                    item['warning'] = 'At least three distinct overlapping views are required to start.'
                result.append(item)
            except JobError as error:
                result.append({'id': folder.name, 'photo_count': 0, 'position_count': 0,
                               'finished_positions': 0, 'usable': False, 'warning': str(error)})
        return result

    def _stages(self, method="mvs"):
        try:
            stages = native_module().NATIVE_STAGES if method == "photogrammetry" else pipeline_module().STAGES
            if stages and isinstance(stages[0], dict):
                return [{'id': 'snapshot', 'label': 'Snapshot', 'status': 'pending'}] + [
                    {'id': s['id'], 'label': s['label'], 'status': 'pending'} for s in stages if s['id'] != 'snapshot']
        except (ImportError, AttributeError):
            pass
        return [{**s, 'status': 'pending'} for s in DEFAULT_STAGES]

    def create(self, values):
        scan_id, quality = values.get('scan_id'), values.get('quality', 'draft')
        method = values.get('method', 'mvs')
        if method not in ('mvs', 'photogrammetry'):
            raise JobError('Unknown reconstruction method.')
        if quality not in ('draft', 'full'):
            raise JobError('Quality must be draft or full.')
        mvs_profile = values.get('mvs_profile', 'standard')
        if mvs_profile not in ('standard', 'calibrated-uniform', 'calibrated'):
            raise JobError('Unknown MVS processing profile.')
        if method != 'mvs' and mvs_profile != 'standard':
            raise JobError('MVS processing profiles require the COLMAP / OpenMVS method.')
        folder, source, photos = self._source(scan_id)
        if len(photos) < 3:
            raise JobError('At least three distinct overlapping views are required to start.')
        if len({p['sha256'] for p in photos}) < 3:
            raise JobError('The scan needs at least three distinct photographs.')
        capabilities = self.engines.inspect()
        if method == 'photogrammetry':
            if not capabilities.get('engines', {}).get('photogrammetry', {}).get('available'):
                raise JobError('Apple photogrammetry is unavailable on this Mac. Run its local toolchain setup.', 503)
        elif capabilities['status'] != 'ready' or not all(capabilities.get('engines', {}).get(name, {}).get('available') for name in ('colmap', 'openmvs')):
            raise JobError('COLMAP and OpenMVS are not ready. Run the toolchain setup first.', 503)
        with self.lock:
            if self.closing.is_set():
                raise JobError('The service is shutting down.', 503)
            if sum(j['status'] in ('queued', 'running') for j in self.jobs.values()) >= 3:
                raise JobError('Three runs are already queued. Wait for one to finish.', 409)
            identifier = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + secrets.token_hex(4)
            target = self.runs / identifier
            target.mkdir()
            planned = {'version': 1, 'scan_id': scan_id, 'scan_created_at': source.get('created_at'),
                       'images': photos, 'source_manifest_sha256': file_sha(folder / 'session.json')}
            atomic_json(target / 'source-plan.json', planned)
            # The apartment scan recovered more coherent surfaces with CPU
            # PatchMatch. Keep Metal for fast drafts; prefer CPU for full runs.
            backend = 'apple' if method == 'photogrammetry' else 'metal' if quality == 'draft' and capabilities['engines']['openmvs'].get('metal_compiled') else 'cpu'
            job = {'version': 1, 'id': identifier, 'scan_id': scan_id, 'status': 'queued',
                   'quality': quality, 'method': method, 'stage': 'snapshot', 'stages': self._stages(method), 'progress': 0,
                   'created_at': now(), 'updated_at': now(), 'error': None, 'warnings': [],
                   'metrics': {'input_images': len(photos)}, 'artifacts': [], 'log_tail': [],
                   'backend': {'requested': backend, 'active': 'unknown'},
                   'calibration': {'status': 'unscaled', 'units': 'arbitrary', 'collision_validated': False},
                   'config': {'quality': quality, 'method': method, 'mvs_profile': mvs_profile, 'engine_paths': capabilities['paths'], 'backend': backend,
                              'native_engine_path': capabilities['paths'].get('PhotogrammetryReconstruct'),
                              'native_ordering': 'unordered', 'native_sensitivity': 'normal',
                              'max_threads': min(4, max(1, (os.cpu_count() or 4) - 2)),
                              'input_sha': hashlib.sha256(json.dumps(planned, sort_keys=True).encode()).hexdigest(),
                              'minimum_registered_fraction': 0.8}}
            if method == 'mvs' and backend != 'metal' and quality == 'draft':
                job['warnings'].append('Metal has not been verified in this toolchain. This run requests CPU processing.')
            self.jobs[identifier] = job
            self.cancel_events[identifier] = threading.Event()
            self._save(job)
            self.queue.put(identifier)
            return self._public(job)

    def _public(self, job):
        result = copy.deepcopy(job)
        result['scene_specific_fit'] = job.get('config', {}).get('scene_specific_fit') is True
        result.pop('config', None)
        result['scaling'] = job['id'] in self.scaling
        result['completing'] = job['id'] in self.completing
        artifacts = []
        for entry in result.get('artifacts', []):
            relative = entry.get('path')
            if not relative:
                continue
            try:
                path = safe_child(self.runs / job['id'], relative)
                if not path.is_file():
                    continue
                item = {**entry, 'name': entry.get('name') or path.name,
                        'url': '/artifacts/' + job['id'] + '/' + Path(relative).as_posix(),
                        'size': path.stat().st_size}
                artifacts.append(item)
            except (JobError, OSError):
                continue
        result['artifacts'] = artifacts
        logs = sorted((self.runs / job['id'] / 'logs').glob('*.log'), key=lambda p: p.stat().st_mtime)
        if logs:
            try:
                with logs[-1].open('rb') as stream:
                    stream.seek(max(0, logs[-1].stat().st_size - 16000))
                    lines = stream.read().decode('utf-8', errors='replace').splitlines()
                result['log_tail'] = lines[-80:]
            except OSError:
                pass
        return result

    def get(self, identifier):
        with self.lock:
            job = self.jobs.get(identifier)
            if not job:
                raise JobError('Reconstruction run was not found.', 404)
            return self._public(job)

    def list(self):
        with self.lock:
            return [self._public(j) for j in sorted(self.jobs.values(), key=lambda j: j['created_at'], reverse=True)]

    def _update(self, identifier, event):
        with self.lock:
            job = self.jobs[identifier]
            stage_id = event.get('stage')
            if stage_id:
                job['stage'] = stage_id
                stage = next((s for s in job['stages'] if s['id'] == stage_id), None)
                if stage is None:
                    stage = {'id': stage_id, 'label': event.get('label', stage_id), 'status': 'pending'}
                    job['stages'].append(stage)
                status = event.get('status')
                if status:
                    stage['status'] = status
                if event.get('message'):
                    stage['message'] = str(event['message'])[:2000]
            if 'progress' in event:
                progress = float(event['progress'])
                if math.isfinite(progress):
                    job['progress'] = min(.99, max(0, progress))
            for key in ('metrics', 'backend'):
                if isinstance(event.get(key), dict):
                    job[key].update(event[key])
            if isinstance(event.get('warnings'), list):
                job['warnings'] = list(dict.fromkeys(job['warnings'] + event['warnings']))
            if isinstance(event.get('artifacts'), list):
                job['artifacts'] = event['artifacts']
            if event.get('message'):
                job['log_tail'] = (job['log_tail'] + [str(event['message'])[:2000]])[-80:]
            self._save(job)

    def _snapshot(self, identifier, cancel):
        from PIL import Image
        folder = self.runs / identifier
        plan = read_json(folder / 'source-plan.json')
        source = safe_child(self.captures, plan['scan_id'])
        self._update(identifier, {'stage': 'snapshot', 'status': 'running', 'message': 'Checking and copying the saved observations.'})
        images = folder / 'images'
        images.mkdir(exist_ok=True)
        required = sum(int(item['bytes']) for item in plan['images'])
        if shutil.disk_usage(folder).free < required * 3 + 256 * 1024 * 1024:
            raise JobError('Not enough free disk space to snapshot and process this scan.', 507)
        for item in plan['images']:
            if cancel.is_set():
                raise JobError('Cancelled.')
            destination = safe_child(images, item['filename'])
            if destination.exists() and file_sha(destination) == item['sha256']:
                continue
            origin = safe_child(source, *Path(item['source']).parts)
            if not origin.is_file() or origin.stat().st_size != item['bytes']:
                raise JobError('A source image changed after this run was queued. Create a new run.')
            temporary = destination.with_suffix('.part')
            digest = hashlib.sha256()
            try:
                with origin.open('rb') as original, temporary.open('wb') as target:
                    for block in iter(lambda: original.read(1024 * 1024), b''):
                        if cancel.is_set():
                            raise JobError('Cancelled.')
                        digest.update(block)
                        target.write(block)
                    target.flush()
                    os.fsync(target.fileno())
                if digest.hexdigest() != item['sha256']:
                    raise JobError('A captured image failed its checksum check. Original files are unchanged.')
                with Image.open(temporary) as image:
                    if image.format != 'JPEG' or image.width * image.height > 50_000_000:
                        raise JobError('A captured image is invalid or too large.')
                    image.verify()
                os.replace(temporary, destination)
            finally:
                temporary.unlink(missing_ok=True)
        atomic_json(folder / 'input-manifest.json', plan)
        self._update(identifier, {'stage': 'snapshot', 'status': 'completed', 'message': f"Saved {len(plan['images'])} verified photographs for this run.", 'progress': .03})

    def _worker(self):
        while not self.closing.is_set():
            try:
                identifier = self.queue.get(timeout=.25)
            except queue.Empty:
                continue
            try:
                with self.lock:
                    job = self.jobs[identifier]
                    cancel = self.cancel_events[identifier]
                    if job['status'] != 'queued' or cancel.is_set():
                        continue
                    job['status'] = 'running'
                    self._save(job)
                self._snapshot(identifier, cancel)
                runner = self.runner or (native_module().run_native if job['config'].get('method') == 'photogrammetry' else pipeline_module().run_pipeline)
                result = runner(self.runs / identifier, copy.deepcopy(job['config']),
                                lambda event: self._update(identifier, event), cancel)
                with self.lock:
                    if cancel.is_set():
                        raise JobError('Cancelled.')
                    for key in ('metrics', 'backend'):
                        if isinstance(result.get(key), dict):
                            job[key].update(result[key])
                    job['warnings'] = list(dict.fromkeys(job['warnings'] + result.get('warnings', [])))
                    job['artifacts'] = result.get('artifacts', [])
                    if not job['artifacts']:
                        raise JobError('The pipeline produced no verified exports.')
                    for artifact in job['artifacts']:
                        relative = artifact.get('path')
                        if not isinstance(relative, str):
                            raise JobError('The pipeline returned an invalid export path.')
                        exported = safe_child(self.runs / identifier, relative)
                        if not exported.is_file() or exported.stat().st_size == 0:
                            raise JobError('A reconstruction export is missing or empty.')
                    job.update(status='completed', progress=1, error=None)
                    self._save(job)
            except Exception as error:
                with self.lock:
                    job = self.jobs[identifier]
                    cancelled = self.cancel_events[identifier].is_set()
                    job.update(status='cancelled' if cancelled else 'failed', error=None if cancelled else str(error)[:4000])
                    for stage in job['stages']:
                        if stage.get('status') == 'running':
                            stage['status'] = 'cancelled' if cancelled else 'failed'
                    self._save(job)
            finally:
                self.queue.task_done()

    def cancel(self, identifier):
        with self.lock:
            job = self.jobs.get(identifier)
            if not job:
                raise JobError('Reconstruction run was not found.', 404)
            if identifier in self.scaling or identifier in self.completing:
                raise JobError('Wait for the current export to finish.', 409)
            if job['status'] in TERMINAL:
                return self._public(job)
            self.cancel_events[identifier].set()
            if job['status'] == 'queued':
                job['status'] = 'cancelled'
                self._save(job)
            return self._public(job)

    def retry(self, identifier):
        with self.lock:
            job = self.jobs.get(identifier)
            if not job:
                raise JobError('Reconstruction run was not found.', 404)
            if job['status'] not in ('failed', 'cancelled', 'interrupted'):
                raise JobError('Only a failed, cancelled or interrupted run can be retried.', 409)
            capabilities = self.engines.inspect()
            method = job.get('config', {}).get('method', 'mvs')
            ready = capabilities.get('engines', {}).get('photogrammetry', {}).get('available') if method == 'photogrammetry' else all(capabilities.get('engines', {}).get(name, {}).get('available') for name in ('colmap', 'openmvs'))
            if not ready:
                raise JobError('The selected reconstruction method is not available.', 503)
            if sum(j['status'] in ('queued', 'running') for j in self.jobs.values()) >= 3:
                raise JobError('The reconstruction queue is full.', 409)
            job['config']['engine_paths'] = capabilities['paths']
            job['config']['native_engine_path'] = capabilities['paths'].get('PhotogrammetryReconstruct')
            job.update(status='queued', error=None, progress=0)
            self.cancel_events[identifier] = threading.Event()
            self._save(job)
            self.queue.put(identifier)
            return self._public(job)

    def scale(self, identifier, values):
        # This is an export transform, not a declaration that collision geometry was validated.
        a, b, distance = values.get('point_a'), values.get('point_b'), values.get('distance_m')
        def vector(value):
            return isinstance(value, list) and len(value) == 3 and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value)
        if not vector(a) or not vector(b) or not isinstance(distance, (int, float)) or isinstance(distance, bool) or not math.isfinite(distance) or not 0 < distance <= 10000:
            raise JobError('Choose two finite 3D points and enter a positive distance in metres.')
        if math.dist(a, b) < 1e-10:
            raise JobError('Choose two different 3D points.')
        with self.lock:
            job = self.jobs.get(identifier)
            if not job:
                raise JobError('Reconstruction run was not found.', 404)
            if job['status'] != 'completed' or identifier in self.scaling or identifier in self.completing:
                raise JobError('Wait for the completed run before changing its scale.', 409)
            source_space = values.get('source_space', 'raw')
            if source_space not in ('raw', 'scaled'):
                raise JobError('Invalid measurement coordinate space.')
            if source_space == 'scaled':
                previous = job.get('calibration', {}).get('meters_per_unit')
                if not previous:
                    raise JobError('This run has no scaled coordinate system.')
                a = [v / previous for v in a]
                b = [v / previous for v in b]
            # Validate the cleanup subtree before changing any metric exports.
            completed = safe_child(self.runs / identifier, 'exports', 'scaled', 'completed')
            if completed.exists() and not completed.is_dir():
                raise JobError('Invalid completion storage directory.')
            self.scaling.add(identifier)
        try:
            module = importlib.import_module((__package__ + '.geometry') if __package__ else 'geometry')
            try:
                result = module.apply_scale(self.runs / identifier, a, b, distance)
            except module.GeometryError as error:
                raise JobError(str(error)) from error
            with self.lock:
                job['calibration'] = {**result.get('calibration', result), 'status': 'scaled', 'collision_validated': False}
                if isinstance(result.get('metrics'), dict):
                    job['metrics']['calibrated_mesh'] = result['metrics']
                    job['metrics']['metric_scale_known'] = True
                job['warnings'] = list(dict.fromkeys(job['warnings'] + result.get('warnings', [])))
                if result.get('artifacts'):
                    raw = [entry for entry in job['artifacts'] if not entry.get('path', '').startswith('exports/scaled/')]
                    job['artifacts'] = raw + result['artifacts']
                # Metric completion belongs to the previous scale and must not survive recalibration.
                job.setdefault('completion', {}).pop('scaled', None)
                job['artifacts'] = [entry for entry in job['artifacts'] if not entry.get('path', '').startswith('exports/scaled/completed/')]
                completed = safe_child(self.runs / identifier, 'exports', 'scaled', 'completed')
                if completed.exists():
                    shutil.rmtree(completed)
                self._save(job)
        finally:
            with self.lock:
                self.scaling.discard(identifier)
        return self.get(identifier)

    def complete(self, identifier, values):
        """Create a separate inferred surface export without changing the reconstruction."""
        source_space = values.get('source_space', 'raw')
        if source_space not in ('raw', 'scaled'):
            raise JobError('Invalid completion coordinate space.')
        with self.lock:
            job = self.jobs.get(identifier)
            if not job:
                raise JobError('Reconstruction run was not found.', 404)
            if job['status'] != 'completed' or identifier in self.scaling or identifier in self.completing:
                raise JobError('Wait for the completed run and current export before filling gaps.', 409)
            if source_space == 'scaled' and job.get('calibration', {}).get('status') != 'scaled':
                raise JobError('This run has no scaled coordinate system.')
            workdir = safe_child(self.runs, identifier)
            parts = ('exports', 'scaled') if source_space == 'scaled' else ('exports',)
            # Resolve every input before calling the algorithm; it also validates its own inputs.
            for name in ('scene.glb', 'surface.ply', 'cameras.json'):
                if not safe_child(workdir, *parts, name).is_file():
                    raise JobError('The source model, surface and camera exports are required.', 404)
            safe_child(workdir, *parts, 'completed')
            self.completing.add(identifier)
        try:
            module = completion_module()
            try:
                result = module.complete_scene(workdir, source_space=source_space)
            except module.CompletionError as error:
                raise JobError(str(error)) from error
            report = result.get('completion')
            if not isinstance(report, dict) or report.get('source_space') != source_space:
                raise JobError('Completion returned an invalid coordinate-space report.', 500)
            report = {**report, 'collision_validated': False}
            prefix = '/'.join((*parts, 'completed')) + '/'
            artifacts = []
            allowed = {'scene-completed.glb', 'surface-completed.ply', 'inferred-surfaces.ply', 'completion.json', 'inferred.glb'}
            for entry in result.get('artifacts', []):
                relative = entry.get('path')
                remainder = Path(relative[len(prefix):]).parts if isinstance(relative, str) and relative.startswith(prefix) else ()
                if len(remainder) != 2 or not IDENTIFIER.fullmatch(remainder[0]) or remainder[1] not in allowed:
                    raise JobError('Completion returned an invalid export path.', 500)
                path = safe_child(workdir, relative)
                if not path.is_file():
                    raise JobError('Completion output was not found.', 500)
                artifacts.append({**entry, 'bytes': path.stat().st_size,
                                  'units': 'metres' if source_space == 'scaled' else 'arbitrary',
                                  'inferred': bool(report.get('contains_inferred_surfaces', report.get('filled_loops', 0))),
                                  'collision_validated': False})
            names = {Path(entry['path']).name for entry in artifacts}
            fingerprints = {Path(entry['path']).parent.name for entry in artifacts}
            if len(fingerprints) != 1 or not {'scene-completed.glb', 'surface-completed.ply', 'completion.json'}.issubset(names):
                raise JobError('Completion produced an incomplete export set.', 500)
            with self.lock:
                job.setdefault('completion', {})[source_space] = report
                job['artifacts'] = [entry for entry in job['artifacts'] if not entry.get('path', '').startswith(prefix)] + artifacts
                job['warnings'] = list(dict.fromkeys(job.get('warnings', []) + result.get('warnings', [])))
                self._save(job)
        finally:
            with self.lock:
                self.completing.discard(identifier)
        return self.get(identifier)

    def artifact(self, identifier, relative):
        with self.lock:
            job = self.jobs.get(identifier)
            if not job:
                raise JobError('Reconstruction run was not found.', 404)
            path = safe_child(self.runs / identifier, relative)
            registered = {entry.get('path') for entry in job.get('artifacts', [])}
            # Texture dependencies are restricted to the export subtree, never the captured inputs.
            is_export = Path(relative).parts and Path(relative).parts[0] == 'exports'
            if relative not in registered and not is_export:
                raise JobError('Export was not found.', 404)
            if path.suffix.lower() not in {'.glb', '.gltf', '.obj', '.mtl', '.ply', '.png', '.jpg', '.jpeg', '.json', '.txt', '.zip'} or not path.is_file():
                raise JobError('Export was not found.', 404)
            return path

    def close(self):
        self.closing.set()
        with self.lock:
            for identifier, job in self.jobs.items():
                if job['status'] in ('running', 'queued'):
                    self.cancel_events[identifier].set()
        if self.worker:
            self.worker.join(timeout=8)
