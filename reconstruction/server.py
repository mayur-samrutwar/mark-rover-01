"""Loopback-only reconstruction dashboard and filesystem job service."""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import re
import shutil
import signal
import threading
from urllib.parse import unquote, urlsplit

try:
    from .jobs import JobError, JobStore, IDENTIFIER, safe_child
except ImportError:
    from jobs import JobError, JobStore, IDENTIFIER, safe_child

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
WEB = ROOT / 'web'
MAX_BODY = 16384


class ReconstructionHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    allow_reuse_port = False


def make_handler(store, port, capture_url='http://127.0.0.1:8765/'):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(30)

        def log_message(self, *args):
            pass

        def end_headers(self):
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Permissions-Policy', 'camera=(), microphone=()')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' blob: data:; connect-src 'self' blob: data:; worker-src 'self' blob:; object-src 'none'; base-uri 'self'; frame-ancestors 'none'")
            super().end_headers()

        def json_response(self, value, status=200):
            data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            if self.command != 'HEAD':
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        def check_origin(self):
            hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}
            if self.headers.get('Host') not in hosts:
                raise JobError('Unexpected host.', 403)
            origin = self.headers.get('Origin')
            if origin and origin not in {'http://' + host for host in hosts}:
                raise JobError('Cross-origin requests are not allowed.', 403)
            if self.headers.get('Sec-Fetch-Site') == 'cross-site':
                raise JobError('Cross-site requests are not allowed.', 403)

        def read_json(self):
            if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
                raise JobError('Expected application/json.', 415)
            if self.headers.get('Transfer-Encoding'):
                raise JobError('Chunked request bodies are not accepted.', 400)
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                raise JobError('Invalid request length.')
            if not 0 <= length <= MAX_BODY:
                raise JobError('Request body is too large.', 413)
            data = self.rfile.read(length)
            if len(data) != length:
                raise JobError('The request body was interrupted.')
            try:
                def invalid(value):
                    raise ValueError()
                value = json.loads(data or b'{}', parse_constant=invalid)
                if not isinstance(value, dict):
                    raise ValueError()
                return value
            except (ValueError, UnicodeError):
                raise JobError('Expected a JSON object with finite numbers.')

        def file_response(self, path):
            try:
                stream = path.open('rb')
            except OSError:
                raise JobError('File was not found.', 404)
            content_type = {'.mjs': 'text/javascript', '.js': 'text/javascript', '.glb': 'model/gltf-binary',
                            '.gltf': 'model/gltf+json', '.obj': 'text/plain', '.mtl': 'text/plain',
                            '.ply': 'application/octet-stream'}.get(path.suffix.lower())
            content_type = content_type or mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
            with stream:
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(path.stat().st_size))
                self.end_headers()
                if self.command != 'HEAD':
                    try:
                        shutil.copyfileobj(stream, self.wfile, 128 * 1024)
                    except (BrokenPipeError, ConnectionResetError):
                        pass

        def dispatch(self):
            try:
                self.check_origin()
                raw = urlsplit(self.path).path
                path = unquote(raw)
                if '\x00' in path or '\\' in path or '..' in Path(path).parts:
                    raise JobError('Invalid URL path.')
                if self.command in ('GET', 'HEAD'):
                    if path == '/api/health':
                        health = store.engines.inspect()
                        health.pop('paths', None)
                        self.json_response({**health, 'capture_url': capture_url, 'version': 1,
                                            'storage': 'filesystem', 'workers': 1})
                    elif path == '/api/scans':
                        self.json_response({'scans': store.scans()})
                    elif path == '/api/jobs':
                        self.json_response({'jobs': store.list()})
                    elif match := re.fullmatch(r'/api/jobs/([a-zA-Z0-9_-]{1,80})', path):
                        self.json_response({'job': store.get(match[1])})
                    elif match := re.fullmatch(r'/artifacts/([a-zA-Z0-9_-]{1,80})/(.+)', path):
                        self.file_response(store.artifact(match[1], match[2]))
                    elif path.startswith('/api/'):
                        raise JobError('Endpoint was not found.', 404)
                    else:
                        relative = 'index.html' if path == '/' else path.lstrip('/')
                        file = safe_child(WEB, *Path(relative).parts)
                        if not file.is_file() or file.suffix not in {'.html', '.css', '.js', '.mjs', '.svg', '.png', '.woff2', '.txt', '.json'}:
                            raise JobError('Page was not found.', 404)
                        self.file_response(file)
                elif self.command == 'POST':
                    values = self.read_json()
                    if path == '/api/jobs':
                        self.json_response({'job': store.create(values)}, 202)
                    elif match := re.fullmatch(r'/api/jobs/([a-zA-Z0-9_-]{1,80})/(cancel|retry|scale|complete)', path):
                        action = match[2]
                        operation = getattr(store, action)
                        job = operation(match[1], values) if action in ('scale', 'complete') else operation(match[1])
                        self.json_response({'job': job}, 202 if action == 'retry' else 200)
                    else:
                        raise JobError('Endpoint was not found.', 404)
                else:
                    raise JobError('Method is not allowed.', 405)
            except JobError as error:
                self.json_response({'error': str(error)}, error.status)
                self.close_connection = True
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                self.close_connection = True
            except Exception as error:
                print(f'Reconstruction request failed: {type(error).__name__}: {error}', flush=True)
                self.json_response({'error': 'Reconstruction storage or data could not be read. Check the service log.'}, 500)
                self.close_connection = True

        do_GET = dispatch
        do_HEAD = dispatch
        do_POST = dispatch
        do_OPTIONS = dispatch

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--captures', type=Path, default=ROOT / 'captures')
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs')
    parser.add_argument('--capture-url', default='http://127.0.0.1:8765/')
    config = parser.parse_args()
    if not 1024 <= config.port <= 65535:
        parser.error('--port must be between 1024 and 65535')
    store = JobStore(PROJECT, captures=config.captures, runs=config.runs)
    try:
        server = ReconstructionHTTPServer(('127.0.0.1', config.port), make_handler(store, config.port, config.capture_url))
    except OSError:
        store.close()
        raise
    def shutdown(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f'Reconstruction: http://127.0.0.1:{config.port}/', flush=True)
    print(f'Scan input: {config.captures.resolve()}', flush=True)
    print(f'Run output: {config.runs.resolve()}', flush=True)
    try:
        server.serve_forever(poll_interval=.25)
    finally:
        server.server_close()
        store.close()


if __name__ == '__main__':
    main()
