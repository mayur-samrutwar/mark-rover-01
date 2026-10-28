"""Exercise the named local origin and reconstruction mount without rover I/O."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import http.client
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent / 'wifi-control/rover_control'))
import serve


class HostingTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        calls = self.calls

        class Upstream(BaseHTTPRequestHandler):
            def do_GET(self):
                body = b'model-binary' if self.path.startswith('/artifacts/') else b'{"mounted":true}'
                calls.append((self.command, self.path, dict(self.headers), None))
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                if self.command != 'HEAD':
                    self.wfile.write(body)

            do_HEAD = do_GET

            def do_POST(self):
                body = self.rfile.read(int(self.headers['Content-Length']))
                calls.append((self.command, self.path, dict(self.headers), body))
                self.send_response(202)
                self.send_header('Content-Length', '2')
                self.end_headers()
                self.wfile.write(b'{}')

            def log_message(self, *args):
                pass

        self.upstream = ThreadingHTTPServer(('127.0.0.1', 0), Upstream)
        self.config = SimpleNamespace(port=0, rover='http://must-not-contact.invalid',
            public_url='http://rover.localhost:1355', reconstruction_port=self.upstream.server_port)
        self.hardware = patch.object(serve, 'rover_request', side_effect=AssertionError('No rover traffic allowed'))
        self.hardware.start()
        self.operator = serve.CaptureHTTPServer(('127.0.0.1', 0), serve.make_handler(self.config))
        self.config.port = self.operator.server_port
        self.threads = []
        for server in (self.upstream, self.operator):
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.threads.append(thread)

    def tearDown(self):
        for server in (self.operator, self.upstream):
            server.shutdown()
            server.server_close()
        for thread in self.threads:
            thread.join(2)
        self.hardware.stop()

    def request(self, path, method='GET', body=None, **headers):
        request_headers = {'Host': 'rover.localhost:1355', **headers}
        connection = http.client.HTTPConnection('127.0.0.1', self.operator.server_port, timeout=3)
        try:
            connection.request(method, path, body, request_headers)
            response = connection.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            connection.close()

    def test_named_origin_serves_control_and_navigation_link(self):
        status, _, body = self.request('/')
        self.assertEqual(status, 200)
        self.assertIn(b'href="/reconstruction/"', body)
        self.assertEqual(self.request('/', Host='evil.localhost:1355')[0], 403)
        self.assertEqual(self.request('/', Origin='http://evil.example')[0], 403)

    def test_reconstruction_mount_and_large_resource_routes(self):
        self.assertEqual(self.request('/reconstruction')[0], 308)
        for path, upstream in [('/reconstruction/', '/'),
                ('/reconstruction/api/jobs?limit=2', '/api/jobs?limit=2'),
                ('/vendor/three.module.min.js', '/vendor/three.module.min.js'),
                ('/artifacts/job/scene.glb', '/artifacts/job/scene.glb')]:
            self.assertEqual(self.request(path)[0], 200)
            self.assertEqual(self.calls[-1][1], upstream)
            self.assertEqual(self.calls[-1][2]['Host'], f'127.0.0.1:{self.upstream.server_port}')
        self.assertEqual(self.request('/artifacts/job/scene.glb')[2], b'model-binary')
        self.assertEqual(self.request('/artifacts/job/scene.glb', 'HEAD')[2], b'')

    def test_same_origin_posts_are_forwarded_and_cross_origin_is_blocked(self):
        body = '{"scan_id":"fixture"}'
        status, _, _ = self.request('/reconstruction/api/jobs', 'POST', body,
            Origin='http://rover.localhost:1355', **{'Content-Type': 'application/json'})
        self.assertEqual(status, 202)
        self.assertEqual(self.calls[-1][1], '/api/jobs')
        self.assertEqual(self.calls[-1][3], body.encode())
        self.assertEqual(self.calls[-1][2]['Origin'], f'http://127.0.0.1:{self.upstream.server_port}')
        count = len(self.calls)
        self.assertEqual(self.request('/reconstruction/api/jobs', 'POST', body,
            Origin='http://evil.example')[0], 403)
        self.assertEqual(len(self.calls), count)
        self.assertEqual(self.request('/reconstruction/api/jobs', 'POST', body,
            Origin='http://rover.localhost:1355', **{'Sec-Fetch-Site': 'cross-site'})[0], 403)

    def test_mount_is_not_available_on_phone_surface(self):
        handler = serve.make_handler(self.config, surface='phone', tls={'host': '127.0.0.1'})
        # Phone requests already reject operator routes through the normal dispatcher.
        self.config.https_port = self.config.port
        phone = serve.CaptureHTTPServer(('127.0.0.1', 0), handler)
        self.config.https_port = phone.server_port
        thread = threading.Thread(target=phone.serve_forever, daemon=True)
        thread.start()
        try:
            connection = http.client.HTTPConnection('127.0.0.1', phone.server_port, timeout=3)
            connection.request('GET', '/reconstruction/')
            response = connection.getresponse()
            self.assertEqual(response.status, 404)
            response.read()
            connection.close()
            self.assertEqual(self.calls, [])
        finally:
            phone.shutdown()
            phone.server_close()
            thread.join(2)

    def test_public_origin_is_exact_and_rejects_external_names(self):
        for url in ('http://evil.example:1355', 'http://rover.localhost.evil:1355',
                    'http://user:pass@rover.localhost:1355', 'http://rover.localhost:1355/path'):
            with self.assertRaises(ValueError):
                serve.public_origin(SimpleNamespace(public_url=url))


if __name__ == '__main__':
    unittest.main()
