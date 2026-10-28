"""Isolated actual HTTP/JobStore service with synthetic geometry, never hardware."""
from http.server import BaseHTTPRequestHandler
import json
from pathlib import Path
import signal
import sys
import tempfile
import threading

RECONSTRUCTION = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RECONSTRUCTION))
from jobs import JobStore
from server import ReconstructionHTTPServer, make_handler
from test_server import FakeEngines, create_scan

TEXTURED_CUBE = Path(__file__).with_name('textured-cube.glb').read_bytes()
POINTS = "ply\nformat ascii 1.0\nelement vertex 8\nproperty float x\nproperty float y\nproperty float z\nend_header\n-1 -1 -1\n1 -1 -1\n1 1 -1\n-1 1 -1\n-1 -1 1\n1 -1 1\n1 1 1\n-1 1 1\n"

def fixture_runner(workdir, config, update, cancelled):
    exports = workdir / 'exports'
    exports.mkdir(exist_ok=True)
    (exports / 'sparse.ply').write_text(POINTS)
    update({'stage': 'alignment', 'status': 'completed', 'progress': .4,
            'artifacts': [{'path': 'exports/sparse.ply', 'kind': 'sparse', 'units': 'unknown'}]})
    if cancelled.wait(1.5):
        raise RuntimeError('Cancelled')
    (exports / 'scene.glb').write_bytes(TEXTURED_CUBE)
    return {'metrics': {'registered_images': 3, 'input_images': 3,
                        'visual_mesh': {'vertices': 8, 'triangles': 12}},
            'backend': {'active': 'cpu'},
            'warnings': ['Fixture geometry only; no reconstruction engine was run.'],
            'artifacts': [{'path': 'exports/sparse.ply', 'kind': 'sparse', 'units': 'unknown'},
                          {'path': 'exports/scene.glb', 'kind': 'visual_mesh', 'units': 'unknown'}]}

with tempfile.TemporaryDirectory(prefix='rover-real-viewer-') as directory:
    temporary = Path(directory)
    create_scan(temporary / 'captures')
    store = JobStore(RECONSTRUCTION.parent, captures=temporary / 'captures', runs=temporary / 'runs',
                     engines=FakeEngines(), runner=fixture_runner)
    server = ReconstructionHTTPServer(('127.0.0.1', 0), BaseHTTPRequestHandler)
    server.RequestHandlerClass = make_handler(store, server.server_address[1])
    def shutdown(*args):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    print(json.dumps({'port': server.server_address[1]}), flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    finally:
        server.server_close()
        store.close()
