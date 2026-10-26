"""Local rover control and paired phone capture, with filesystem-only storage."""
from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import shutil
import ssl
import tempfile
import threading
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, urlopen
import zipfile

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[1]
ROVER_PATHS = {'/api/session', '/api/command', '/api/stop', '/api/info'}
STATIC = {'/index.html', '/style.css', '/app.mjs', '/controls.mjs', '/wifi-link.mjs', '/capture.mjs', '/preview-link.mjs'}
PHONE_STATIC = {'/phone/': 'phone.html', '/phone/index.html': 'phone.html',
                '/phone/phone.css': 'phone.css', '/phone/phone.mjs': 'phone.mjs',
                '/phone.css': 'phone.css', '/phone.mjs': 'phone.mjs',
                '/phone-video.mjs': 'phone-video.mjs', '/phone/phone-video.mjs': 'phone-video.mjs'}


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rover', default='http://192.168.1.6')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--capture-host', help='Laptop LAN IPv4 address used by the phone')
    parser.add_argument('--https-port', type=int, default=8443)
    parser.add_argument('--setup-port', type=int, default=8766)
    parser.add_argument('--capture-data-dir', type=Path, default=PROJECT / 'reconstruction' / 'captures')
    parser.add_argument('--cert-dir', type=Path, default=PROJECT / '.capture-tls')
    parser.add_argument('--pair-code', help='Keep a known six-digit pairing code during a local development restart')
    parser.add_argument('--no-capture', action='store_true', help='Run only the original loopback controller')
    return parser.parse_args()


class CaptureHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_handler(config, store=None, surface='operator', tls=None):
    class Handler(SimpleHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        extensions_map = {**SimpleHTTPRequestHandler.extensions_map, '.mjs': 'text/javascript'}

        def __init__(self, *values, **kwargs):
            super().__init__(*values, directory=str(ROOT), **kwargs)

        def setup(self):
            super().setup()
            self.connection.settimeout(35)

        def log_message(self, *values):
            pass

        def end_headers(self):
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Permissions-Policy', 'camera=(self), microphone=()')
            self.send_header('X-Frame-Options', 'DENY')
            super().end_headers()

        @property
        def parsed(self):
            return urlsplit(self.path)

        def check_origin(self):
            if surface == 'operator':
                allowed_hosts = {f'127.0.0.1:{config.port}', f'localhost:{config.port}'}
                scheme = 'http'
            elif surface == 'phone':
                allowed_hosts = {f'{tls["host"]}:{config.https_port}', f'localhost:{config.https_port}', f'127.0.0.1:{config.https_port}'}
                scheme = 'https'
            else:
                allowed_hosts = {f'{tls["host"]}:{config.setup_port}', f'127.0.0.1:{config.setup_port}', f'localhost:{config.setup_port}'}
                scheme = 'http'
            if self.headers.get('Host') not in allowed_hosts:
                self.send_error(403, 'Unexpected host')
                return False
            origin = self.headers.get('Origin')
            if origin and origin not in {f'{scheme}://{host}' for host in allowed_hosts}:
                self.send_error(403, 'Cross-origin access is not allowed')
                return False
            if self.command == 'POST' and self.headers.get('Sec-Fetch-Site') == 'cross-site':
                self.send_error(403, 'Cross-site writes are not allowed')
                return False
            return True

        def json_response(self, value, status=200):
            data = json.dumps(value, allow_nan=False).encode()
            self.bytes_response(data, 'application/json; charset=utf-8', status)

        def bytes_response(self, data, content_type, status=200, filename=None):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            if filename:
                self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
            self.end_headers()
            if self.command != 'HEAD':
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        def read_body(self, maximum):
            from capture_server import CaptureError
            try:
                if self.headers.get('Transfer-Encoding'):
                    raise ValueError()
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= maximum:
                    raise ValueError()
            except ValueError:
                raise CaptureError('Invalid upload length.', 413)
            data = self.rfile.read(length)
            if len(data) != length:
                raise CaptureError('The upload was interrupted.')
            return data

        def read_json(self, maximum=16384):
            from capture_server import CaptureError
            try:
                value = json.loads(self.read_body(maximum) or b'{}')
                if not isinstance(value, dict):
                    raise ValueError()
                return value
            except (ValueError, UnicodeError):
                raise CaptureError('Expected a JSON object.')

        def file_response(self, path, content_type='image/jpeg'):
            try:
                stream = path.open('rb')
            except OSError:
                self.send_error(404)
                return
            with stream:
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(path.stat().st_size))
                self.end_headers()
                if self.command != 'HEAD':
                    try:
                        shutil.copyfileobj(stream, self.wfile, 64 * 1024)
                    except (BrokenPipeError, ConnectionResetError):
                        pass

        def static(self, filename):
            original = self.path
            self.path = '/' + filename
            try:
                super().do_GET() if self.command != 'HEAD' else super().do_HEAD()
            finally:
                self.path = original

        def relay(self):
            path = self.parsed.path
            if surface != 'operator' or path not in ROVER_PATHS:
                self.send_error(403 if surface != 'operator' else 404)
                return
            if (path == '/api/info') != (self.command in {'GET', 'HEAD'}):
                self.send_error(405)
                return
            # Keep control independent of phone pairing, camera traffic and disk work.
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 1024 or self.headers.get('Transfer-Encoding'):
                    raise ValueError()
            except ValueError:
                self.send_error(400)
                return
            headers = {'Content-Type': 'text/plain', 'Connection': 'close'}
            token = self.headers.get('X-Rover-Token')
            if token:
                headers['X-Rover-Token'] = token
            body = self.rfile.read(length) if self.command == 'POST' else None
            request = Request(config.rover.rstrip('/') + path, data=body, headers=headers,
                              method='GET' if self.command == 'HEAD' else self.command)
            try:
                try:
                    response = urlopen(request, timeout=2)
                except HTTPError as error:
                    response = error
                with response:
                    status, data = response.status, response.read(8192)
            except (OSError, URLError):
                status = 502
                data = json.dumps({'error': 'Rover unavailable over Wi-Fi. Check power and home network.'}).encode()
            self.bytes_response(data, 'application/json', status)

        def bootstrap(self):
            if self.parsed.path == '/capture-ca.crt':
                self.bytes_response(Path(tls['ca_file']).read_bytes(), 'application/x-x509-ca-cert', filename='mark-rover-capture-ca.crt')
                return
            if self.parsed.path != '/':
                self.send_error(404)
                return
            # Pairing codes are visible only on the loopback operator page.
            url = f'https://{tls["host"]}:{config.https_port}/phone/'
            page = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Camera setup</title>
<style>body{{font:15px/1.65 system-ui,sans-serif;color:#34383e;max-width:520px;margin:60px auto;padding:24px}}h1{{font-size:24px;font-weight:550}}a{{color:#34383e}}.button{{display:inline-block;background:#303740;color:white;padding:12px 18px;border-radius:10px;text-decoration:none}}li{{margin:18px 0}}small{{color:#7b838d}}</style>
<h1>Camera setup</h1><p>One-time setup for this laptop’s local HTTPS camera page.</p><ol>
<li><a href="/capture-ca.crt">Download the local camera certificate</a>.</li>
<li>In Android Settings, search for <strong>Install a certificate</strong>. Choose <strong>CA certificate</strong> and select <strong>mark-rover-capture-ca.crt</strong>. Samsung may place this under Security and privacy → More security settings → Install from device storage.</li>
<li>Open the camera page below in Chrome. Enter the six-digit pairing code shown on your laptop, allow camera access and tap Start camera.</li></ol>
<p><a class="button" href="{url}">Open camera</a></p><p><small>This public development CA comes from mkcert on your laptop. Its private key stays on the laptop. Android warns because installing a CA changes device trust. Install only the certificate from your own laptop; you can remove it from user credentials after the experiment. Photos stay on your local network.</small></p></html>'''
            self.bytes_response(page.encode(), 'text/html; charset=utf-8')

        def capture_get(self):
            from capture_server import CaptureError
            path = self.parsed.path
            token = self.headers.get('X-Capture-Token')
            if surface == 'phone':
                if path == '/capture/phone/rtc':
                    self.json_response(store.rtc_get(token, phone=True))
                    return
                if path != '/capture/phone/commands':
                    raise CaptureError('This endpoint is available only on the laptop.', 403)
                self.json_response(store.phone_commands(token))
                return
            if path == '/capture/api/state':
                self.json_response(store.state() if store else {'enabled': False, 'error': 'Capture is disabled.'})
            elif path.startswith('/capture/phone/'):
                raise CaptureError('Pair the phone through its HTTPS address.', 403)
            elif not store:
                raise CaptureError('Capture is disabled.', 503)
            elif path == '/capture/api/rtc':
                self.json_response(store.rtc_get())
            elif path == '/capture/api/preview.jpg':
                self.bytes_response(store.preview_bytes(), 'image/jpeg')
            elif match := re.fullmatch(r'/capture/api/photos/([a-zA-Z0-9_-]+)(\.thumb)?\.jpg', path):
                self.file_response(store.photo_path(match[1], bool(match[2])))
            elif match := re.fullmatch(r'/capture/api/export/([a-zA-Z0-9_-]+)\.zip', path):
                folder = store.export_folder(match[1])
                # A temporary on-disk ZIP keeps large sessions out of RAM.
                with tempfile.TemporaryDirectory(prefix='rover-export-') as temporary:
                    archive = Path(temporary) / (folder.name + '.zip')
                    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_STORED) as zipped:
                        for source in sorted(folder.rglob('*')):
                            if source.is_file() and not source.is_symlink() and source.suffix in {'.jpg', '.json'}:
                                zipped.write(source, Path(folder.name) / source.relative_to(folder))
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/zip')
                    self.send_header('Content-Disposition', f'attachment; filename="{folder.name}.zip"')
                    self.send_header('Content-Length', str(archive.stat().st_size))
                    self.end_headers()
                    if self.command != 'HEAD':
                        with archive.open('rb') as stream:
                            try:
                                shutil.copyfileobj(stream, self.wfile, 64 * 1024)
                            except (BrokenPipeError, ConnectionResetError):
                                pass
            else:
                raise CaptureError('Capture endpoint was not found.', 404)

        def capture_post(self):
            from capture_server import CaptureError, MAX_PHOTO_BYTES, MAX_PREVIEW_BYTES
            if not store:
                raise CaptureError('Capture is disabled.', 503)
            path = self.parsed.path
            token = self.headers.get('X-Capture-Token')
            if surface == 'phone':
                if path == '/capture/phone/pair':
                    values = self.read_json()
                    self.json_response(store.pair(values.get('code'), values.get('device', 'Android camera'), self.client_address[0]))
                    return
                with store.lock:
                    store.authenticate(token)
                if path == '/capture/phone/rtc':
                    self.json_response(store.rtc_phone(token, self.read_json(70000)))
                elif path in {'/capture/phone/preview', '/capture/phone/upload'}:
                    if self.headers.get('Content-Type', '').split(';')[0] != 'image/jpeg':
                        raise CaptureError('Expected image/jpeg.', 415)
                    if path.endswith('/preview'):
                        self.json_response(store.put_preview(token, self.read_body(MAX_PREVIEW_BYTES)))
                    else:
                        identifier = parse_qs(self.parsed.query).get('command_id', [''])[0]
                        raw_metadata = self.headers.get('X-Frame-Metadata', '{}')
                        try:
                            if len(raw_metadata) > 8192:
                                raise ValueError()
                            metadata = json.loads(raw_metadata)
                            if not isinstance(metadata, dict):
                                raise ValueError()
                            json.dumps(metadata, allow_nan=False)
                        except (ValueError, UnicodeError):
                            raise CaptureError('Invalid camera metadata.')
                        self.json_response(store.upload(token, identifier, self.read_body(MAX_PHOTO_BYTES), metadata))
                elif path == '/capture/phone/status':
                    self.json_response(store.phone_status(token, self.read_json()))
                elif path == '/capture/phone/error':
                    values = self.read_json()
                    self.json_response(store.phone_error(token, values.get('command_id'), values.get('error', 'Capture failed.')))
                else:
                    raise CaptureError('This endpoint is available only on the laptop.', 403)
                return
            if path.startswith('/capture/phone/'):
                raise CaptureError('Pair the phone through its HTTPS address.', 403)
            values = self.read_json(70000 if path == '/capture/api/rtc' else 16384)
            if path == '/capture/api/rtc':
                self.json_response(store.rtc_operator(values))
            elif path == '/capture/api/sessions':
                self.json_response(store.new_session(), 201)
            elif path == '/capture/api/stations':
                self.json_response(store.new_station(), 201)
            elif match := re.fullmatch(r'/capture/api/stations/([a-zA-Z0-9_-]+)/finish', path):
                self.json_response(store.finish_station(match[1]))
            elif path == '/capture/api/capture':
                self.json_response(store.request_capture(values.get('station_id'), values.get('request_id')), 202)
            elif path == '/capture/api/capture/cancel':
                self.json_response(store.cancel_capture(values.get('request_id')))
            else:
                raise CaptureError('Capture endpoint was not found.', 404)

        def dispatch(self):
            if not self.check_origin():
                return
            path = self.parsed.path
            try:
                if surface == 'setup':
                    self.bootstrap() if self.command in {'GET', 'HEAD'} else self.send_error(405)
                elif path.startswith('/api/'):
                    self.relay()
                elif self.command == 'POST':
                    self.capture_post()
                elif surface == 'phone' and path in {'/', '/phone'}:
                    self.send_response(302)
                    self.send_header('Location', '/phone/' + ('?' + self.parsed.query if self.parsed.query else ''))
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                elif surface == 'phone' and path in PHONE_STATIC:
                    self.static(PHONE_STATIC[path])
                elif path.startswith('/capture/'):
                    self.capture_get()
                elif surface == 'operator' and (path == '/' or path in STATIC):
                    self.static('index.html' if path == '/' else path.lstrip('/'))
                else:
                    self.send_error(404)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                self.close_connection = True
            except Exception as error:
                from capture_server import CaptureError
                if isinstance(error, CaptureError):
                    self.json_response({'error': str(error)}, error.status)
                elif isinstance(error, OSError):
                    self.json_response({'error': 'Capture storage failed. Check free disk space and retry.'}, 507)
                else:
                    print(f'Capture request failed: {type(error).__name__}: {error}', flush=True)
                    self.json_response({'error': 'Capture request failed. Check the server terminal.'}, 500)
                # Rejected large/unauthorised bodies must not become the next request.
                self.close_connection = True

        do_GET = dispatch
        do_POST = dispatch
        do_HEAD = dispatch

    return Handler


def main():
    config = arguments()
    store, tls = None, None
    camera_servers = []
    if not config.no_capture:
        try:
            from capture_server import CaptureStore
            from capture_tls import prepare_tls
            tls = prepare_tls(config.cert_dir, config.capture_host)
            if tls.get('error'):
                print(f'Phone capture unavailable: {tls["error"]}', flush=True)
            else:
                if config.pair_code and not re.fullmatch(r'\d{6}', config.pair_code):
                    raise ValueError('--pair-code must contain six digits.')
                code = config.pair_code or f'{secrets.randbelow(1_000_000):06d}'
                pairing = {'code': code, 'url': f'https://{tls["host"]}:{config.https_port}/phone/?code={code}',
                           'setup_url': f'http://{tls["host"]}:{config.setup_port}/'}
                store = CaptureStore(config.capture_data_dir, pairing=pairing)
                camera = CaptureHTTPServer(('0.0.0.0', config.https_port), make_handler(config, store, 'phone', tls))
                camera_servers.append(camera)
                context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                context.minimum_version = ssl.TLSVersion.TLSv1_2
                context.load_cert_chain(tls['cert_file'], tls['key_file'])
                camera.socket = context.wrap_socket(camera.socket, server_side=True)
                setup = CaptureHTTPServer(('0.0.0.0', config.setup_port), make_handler(config, store, 'setup', tls))
                camera_servers.append(setup)
        except (OSError, ImportError, ValueError) as error:
            print(f'Phone capture unavailable: {error}', flush=True)
            for server in camera_servers:
                server.server_close()
            camera_servers = []
            store = None
    operator = CaptureHTTPServer(('127.0.0.1', config.port), make_handler(config, store))
    for server in camera_servers:
        threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Rover control: http://127.0.0.1:{config.port}/ → {config.rover}', flush=True)
    if store:
        print(f'Phone setup: {store.pairing["setup_url"]}', flush=True)
        print(f'Phone camera: {store.pairing["url"]}', flush=True)
        print(f'Capture folders: {store.root}', flush=True)
    try:
        operator.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        operator.server_close()
        for server in camera_servers:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    main()
