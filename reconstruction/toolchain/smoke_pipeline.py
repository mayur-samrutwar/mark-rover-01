"""Exercise the actual reconstruction worker on the public OpenMVS sample."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

project = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(project))
from PIL import Image
from reconstruction.jobs import EngineRegistry, JobStore, atomic_json

folder = Path(tempfile.mkdtemp(prefix='rover-native-e2e-', dir='/private/tmp'))
captures = folder / 'captures'
station = captures / 'public-openmvs-sample' / 'position-001'
station.mkdir(parents=True)
fixture = project / '.reconstruction-tools/src/openMVS/apps/Tests/data/images'
photos = []
for index, source in enumerate(sorted(fixture.iterdir())):
    if source.suffix.lower() not in ('.jpg', '.jpeg', '.heic'):
        continue
    target = station / (source.stem + '.jpg')
    if source.suffix.lower() == '.heic':
        subprocess.run(['sips', '-s', 'format', 'jpeg', str(source), '--out', str(target)], check=True, stdout=subprocess.DEVNULL)
    else:
        shutil.copy2(source, target)
    with Image.open(target) as image:
        width, height = image.size
    data = target.read_bytes()
    photos.append({'id': str(index), 'filename': target.name, 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data), 'width': width, 'height': height})
atomic_json(station.parent / 'session.json', {'version': 1, 'id': station.parent.name, 'created_at': '2026-10-27T00:00:00Z', 'stations': [{'id': station.name, 'status': 'ready', 'photos': photos}]})
engines = EngineRegistry(project)
health = engines.inspect(refresh=True)
if health['status'] != 'ready':
    raise SystemExit('Toolchain not ready: ' + json.dumps(health['missing']))
store = JobStore(project, captures=captures, runs=folder / 'runs', engines=engines)
job = store.create({'scan_id': station.parent.name, 'quality': 'draft'})
print(json.dumps({'test_root': str(folder), 'job_id': job['id'], 'input_images': len(photos)}), flush=True)
last = None
try:
    while True:
        result = store.get(job['id'])
        marker = (result['status'], result['stage'])
        if marker != last:
            print(json.dumps({'status': result['status'], 'stage': result['stage'], 'backend': result['backend']}), flush=True)
            last = marker
        if result['status'] in {'completed', 'failed', 'cancelled'}:
            print(json.dumps({'status': result['status'], 'registered_images': result['metrics'].get('registered_images'), 'backend': result['backend'], 'error': result.get('error'), 'artifacts': [{'name': a['name'], 'bytes': a.get('size')} for a in result['artifacts']], 'test_root': str(folder)}), flush=True)
            break
        time.sleep(.5)
finally:
    store.close()
if result['status'] != 'completed':
    raise SystemExit(1)
