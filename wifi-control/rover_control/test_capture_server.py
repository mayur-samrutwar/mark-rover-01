"""Capture acquisition contracts, persistence and isolated HTTP permission tests.

All datasets live in temporary directories. HTTP servers bind ephemeral localhost
ports, and outbound rover requests are mocked so this suite cannot move hardware.
"""
from __future__ import annotations

from contextlib import ExitStack
import hashlib
import http.client
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile

from PIL import Image

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import capture_server
from capture_server import CaptureError, CaptureStore
import serve


def image_bytes(colour="steelblue", size=(96, 64), format="JPEG"):
    stream = io.BytesIO()
    Image.new("RGB", size, colour).save(stream, format=format)
    return stream.getvalue()


class StoreFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="rover-capture-test-")
        self.addCleanup(self.temporary.cleanup)
        self.data_dir = Path(self.temporary.name) / "captures"
        self.store = CaptureStore(self.data_dir, pairing={"code": "123456"})
        self.jpeg = image_bytes()

    def assert_capture_error(self, status, operation, *args, **kwargs):
        with self.assertRaises(CaptureError) as raised:
            operation(*args, **kwargs)
        self.assertEqual(raised.exception.status, status)
        return raised.exception

    def ready_phone(self):
        if not self.store.phone_token:
            self.token = self.store.pair("123456", "test camera", "127.0.0.1")["token"]
        else:
            self.token = self.store.phone_token
        self.store.phone_status(self.token, {
            "camera_ready": True, "visibility": "visible", "width": 96, "height": 64,
        })
        return self.token

    def start_capture(self, request_id="request-1"):
        self.ready_phone()
        if not self.store.session:
            self.store.new_session()
        return self.store.request_capture(self.store.active_station_id, request_id)["capture"]

    def save_photo(self):
        command = self.start_capture()
        return self.store.upload(self.token, command["id"], self.jpeg, {
            "captured_at": "2026-10-26T12:00:00.000Z", "browser": "fixture",
        })["photo"]


class CaptureStoreTests(StoreFixture):
    def test_pairing_authorisation_and_rate_limit(self):
        self.assert_capture_error(401, self.store.authenticate, "unknown")
        for unused in range(8):
            self.assert_capture_error(403, self.store.pair, "wrong", "camera", "client")
        self.assert_capture_error(429, self.store.pair, "123456", "camera", "client")
        with patch.object(capture_server.time, "monotonic", return_value=time.monotonic() + 61):
            token = self.store.pair("123456", "camera", "client")["token"]
        self.assert_capture_error(401, self.store.phone_commands, "incorrect-token")
        self.assert_capture_error(409, self.store.pair, "123456", "replacement", "other-client")
        self.assertNotIn(token, json.dumps(self.store.state()))

    def test_camera_must_be_visible_fresh_and_online(self):
        self.store.new_session()
        station = self.store.active_station_id
        self.assert_capture_error(409, self.store.request_capture, station, "unpaired")
        self.ready_phone()
        for condition in ("not-started", "hidden", "stale-status", "offline"):
            with self.subTest(condition=condition):
                self.ready_phone()
                if condition == "not-started":
                    self.store.phone["camera_ready"] = False
                elif condition == "hidden":
                    self.store.phone["visibility"] = "hidden"
                elif condition == "stale-status":
                    self.store.last_camera_status = time.monotonic() - 7
                    # Command polling is not proof that camera frames are fresh.
                    self.store.phone_commands(self.token)
                else:
                    self.store.last_seen = time.monotonic() - 7
                self.assert_capture_error(409, self.store.request_capture, station, condition)
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])

    def test_capture_expiry_does_not_write_a_late_photo(self):
        command = self.start_capture()
        self.store.commands[command["id"]]["expires_at"] = time.time() - 1
        self.assertEqual(self.store.state()["capture"]["status"], "expired")
        self.assertEqual(self.store.phone_commands(self.token)["commands"], [])
        self.assert_capture_error(409, self.store.upload, self.token, command["id"], self.jpeg, {})
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])
        replacement = self.store.request_capture(self.store.active_station_id, "replacement")["capture"]
        self.assertNotEqual(command["id"], replacement["id"])

    def test_capture_request_and_upload_retries_are_idempotent(self):
        command = self.start_capture()
        same = self.store.request_capture(self.store.active_station_id, "request-1")["capture"]
        self.assertEqual(same["id"], command["id"])
        self.assert_capture_error(409, self.store.request_capture, self.store.active_station_id, "request-2")
        first = self.store.upload(self.token, command["id"], self.jpeg, {})["photo"]
        retry = self.store.upload(self.token, command["id"], self.jpeg, {})
        self.assertTrue(retry["duplicate"])
        self.assertEqual(retry["photo"], first)
        self.assert_capture_error(409, self.store.upload, self.token, command["id"], image_bytes("tomato"), {})
        self.assertEqual(len(self.store.state()["stations"][0]["photos"]), 1)
        self.assertEqual(len(list(self.data_dir.rglob("*.jpg"))), 2)  # Original + thumbnail.

    def test_cancel_before_delayed_request_reserves_the_request_id(self):
        self.store.new_session()
        cancelled = self.store.cancel_capture("delayed-request")
        self.assertTrue(cancelled["cancelled"])
        self.ready_phone()
        delayed = self.store.request_capture(self.store.active_station_id, "delayed-request")["capture"]
        self.assertEqual(delayed["status"], "cancelled")
        self.assertEqual(delayed["id"], cancelled["capture"]["id"])
        self.assertEqual(delayed["request_id"], "delayed-request")
        self.assertEqual(self.store.phone_commands(self.token)["commands"], [])
        self.assert_capture_error(409, self.store.upload, self.token, delayed["id"], self.jpeg, {})
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])

    def test_cancel_during_image_decode_prevents_a_late_commit(self):
        command = self.start_capture()
        decoding = threading.Event()
        resume = threading.Event()
        failures = []
        original_image = self.store._image

        def slow_image(data, **kwargs):
            picture = original_image(data, **kwargs)
            decoding.set()
            if not resume.wait(timeout=2):
                raise TimeoutError("Test did not release image processing")
            return picture

        def upload():
            try:
                self.store.upload(self.token, command["id"], self.jpeg, {})
            except Exception as error:
                failures.append(error)

        with patch.object(self.store, "_image", side_effect=slow_image):
            thread = threading.Thread(target=upload, daemon=True)
            thread.start()
            try:
                self.assertTrue(decoding.wait(timeout=2))
                self.assertTrue(self.store.cancel_capture("request-1")["cancelled"])
            finally:
                resume.set()
                thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], CaptureError)
        self.assertEqual(failures[0].status, 409)
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])
        self.assertEqual(self.store.state()["stations"][0]["photos"], [])

    def test_cancel_after_completion_preserves_the_acknowledged_photo(self):
        photo = self.save_photo()
        cancelled = self.store.cancel_capture("request-1")
        self.assertFalse(cancelled["cancelled"])
        self.assertEqual(cancelled["capture"]["status"], "complete")
        self.assertEqual(self.store.photo_path(photo["id"]).read_bytes(), self.jpeg)
        retry = self.store.request_capture(self.store.active_station_id, "request-1")["capture"]
        self.assertEqual(retry["status"], "complete")
        self.assertEqual(retry["request_id"], "request-1")
        self.assertTrue(self.store.upload(self.token, retry["id"], self.jpeg, {})["duplicate"])

    def test_groups_require_photos_and_block_transitions_during_capture(self):
        self.store.new_session()
        first_id = self.store.active_station_id
        self.assert_capture_error(409, self.store.finish_station, first_id)
        self.assert_capture_error(409, self.store.new_station)
        command = self.start_capture()
        self.assert_capture_error(409, self.store.finish_station, first_id)
        self.assert_capture_error(409, self.store.new_session)
        self.assert_capture_error(409, self.store.export_folder, self.store.session["id"])
        self.store.upload(self.token, command["id"], self.jpeg, {})
        finished = self.store.finish_station(first_id)
        self.assertIsNone(finished["active_station_id"])
        self.assertEqual(finished["stations"][0]["status"], "ready")
        self.assertEqual(self.store.finish_station(first_id)["stations"], finished["stations"])
        second = self.store.new_station()
        self.assertEqual(second["stations"][1]["number"], 2)
        self.assertNotEqual(second["active_station_id"], first_id)
        old_session = self.store.session["id"]
        self.store.new_session()
        old = json.loads((self.data_dir / old_session / "session.json").read_text())
        self.assertEqual(old["stations"][1]["status"], "empty")
        self.assertIn("closed_at", old)
        self.assertTrue(self.store.export_folder(old_session).is_dir())

    def test_rejected_images_leave_capture_retryable_and_disk_clean(self):
        command = self.start_capture()
        invalid = (
            b"not an image",
            self.jpeg[:100],
            image_bytes(format="PNG"),
            image_bytes(size=(8, 8)),
        )
        for payload in invalid:
            with self.subTest(payload_size=len(payload)):
                self.assert_capture_error(415, self.store.upload, self.token, command["id"], payload, {})
                self.assertEqual(self.store.commands[command["id"]]["status"], "queued")
        with patch.object(capture_server, "MAX_PHOTO_BYTES", 10):
            self.assert_capture_error(413, self.store.upload, self.token, command["id"], self.jpeg, {})
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])
        self.assertEqual(self.store.upload(self.token, command["id"], self.jpeg, {})["photo"]["width"], 96)

    def test_preview_requires_auth_and_cannot_claim_camera_readiness(self):
        self.store.pair("123456", "camera", "client")
        self.token = self.store.phone_token
        self.assert_capture_error(401, self.store.put_preview, "wrong", self.jpeg)
        self.assert_capture_error(415, self.store.put_preview, self.token, image_bytes(size=(1500, 1500)))
        self.store.put_preview(self.token, self.jpeg)
        self.assertEqual(self.store.preview_bytes(), self.jpeg)
        self.assertFalse(self.store.state()["phone"]["camera_ready"])

    def test_photo_metadata_and_manifests_survive_restart(self):
        photo = self.save_photo()
        path = self.store.photo_path(photo["id"])
        self.assertEqual(path.read_bytes(), self.jpeg)
        self.assertEqual(photo["sha256"], hashlib.sha256(self.jpeg).hexdigest())
        self.assertEqual((photo["width"], photo["height"]), (96, 64))
        self.assertIsNone(photo["camera_pose"])
        self.assertIsNone(photo["intrinsics"])
        self.assertIsNone(photo["depth"])
        sidecar = json.loads(path.with_suffix(".json").read_text())
        manifest = json.loads((path.parent / "manifest.json").read_text())
        self.assertEqual(sidecar, photo)
        self.assertEqual(manifest["photos"], [photo])
        self.assertIsNone(manifest["pose_source"])
        recovered = CaptureStore(self.data_dir)
        self.assertEqual(recovered.photo_path(photo["id"]).read_bytes(), self.jpeg)
        self.assertEqual(recovered.state()["stations"][0]["photos"], [photo])
        self.assertFalse(recovered.state()["phone"]["connected"])

    def test_sidecar_recovers_interrupted_session_manifest_update(self):
        photo = self.save_photo()
        session_path = self.data_dir / self.store.session["id"] / "session.json"
        interrupted = json.loads(session_path.read_text())
        interrupted["stations"][0]["photos"] = []
        session_path.write_text(json.dumps(interrupted))
        recovered = CaptureStore(self.data_dir)
        self.assertEqual(recovered.state()["stations"][0]["photos"], [photo])
        reloaded = json.loads(session_path.read_text())
        self.assertEqual(reloaded["stations"][0]["photos"], [photo])

    def test_identifiers_and_client_metadata_cannot_choose_output_paths(self):
        command = self.start_capture()
        self.assert_capture_error(404, self.store.upload, self.token, "../../outside", self.jpeg, {})
        self.assert_capture_error(404, self.store.photo_path, "../outside")
        self.assert_capture_error(404, self.store.export_folder, "../outside")
        metadata = {"filename": "../../outside.jpg", "station_id": "../../escape", "captured_at": "fixture"}
        photo = self.store.upload(self.token, command["id"], self.jpeg, metadata)["photo"]
        actual = self.store.photo_path(photo["id"]).resolve()
        self.assertTrue(actual.is_relative_to(self.data_dir.resolve()))
        self.assertNotIn("..", photo["filename"])
        self.assertFalse((Path(self.temporary.name) / "outside.jpg").exists())


class CaptureHTTPTests(StoreFixture):
    def setUp(self):
        super().setUp()
        self.resources = ExitStack()
        self.addCleanup(self.resources.close)
        self.network = self.resources.enter_context(patch.object(
            serve, "urlopen", side_effect=AssertionError("Tests must not contact the real rover."),
        ))
        self.config = SimpleNamespace(port=0, https_port=0, setup_port=0,
                                      rover="http://must-not-contact.invalid")
        ca = Path(self.temporary.name) / "public-ca.crt"
        ca.write_bytes(b"PUBLIC TEST CA CERTIFICATE")
        self.tls = {"host": "127.0.0.1", "ca_file": str(ca)}
        self.servers = {}
        for surface, attribute in (("operator", "port"), ("phone", "https_port"), ("setup", "setup_port")):
            handler = serve.make_handler(self.config, self.store, surface, self.tls)
            server = serve.CaptureHTTPServer(("127.0.0.1", 0), handler)
            setattr(self.config, attribute, server.server_port)
            self.servers[surface] = server
            thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
            thread.start()
            self.addCleanup(self.close_server, server, thread)
        self.store.pairing.update({
            "url": f"https://127.0.0.1:{self.config.https_port}/phone/?code=123456",
            "setup_url": f"http://127.0.0.1:{self.config.setup_port}/",
        })

    @staticmethod
    def close_server(server, thread):
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)

    def request(self, surface, method, path, body=None, headers=None):
        headers = dict(headers or {})
        if isinstance(body, dict) or isinstance(body, list):
            body = json.dumps(body).encode()
            headers.setdefault("Content-Type", "application/json")
        connection = http.client.HTTPConnection("127.0.0.1", self.servers[surface].server_port, timeout=3)
        try:
            connection.request(method, path, body=body, headers=headers)
            response = connection.getresponse()
            data = response.read()
            return response.status, dict(response.getheaders()), data
        finally:
            connection.close()

    def json_request(self, surface, method, path, body=None, headers=None, expected=200):
        status, response_headers, data = self.request(surface, method, path, body, headers)
        self.assertEqual(status, expected, data[:500])
        self.assertIn("application/json", response_headers.get("Content-Type", ""))
        return json.loads(data)

    def pair_http(self):
        response = self.json_request("phone", "POST", "/capture/phone/pair", {"code": "123456"})
        self.token = response["token"]
        self.phone_headers = {"X-Capture-Token": self.token}
        self.json_request("phone", "POST", "/capture/phone/status", {
            "camera_ready": True, "visibility": "visible", "width": 96, "height": 64,
        }, self.phone_headers)

    def capture_http(self):
        self.pair_http()
        state = self.json_request("operator", "POST", "/capture/api/sessions", {}, expected=201)
        command = self.json_request("operator", "POST", "/capture/api/capture", {
            "station_id": state["active_station_id"], "request_id": "http-request",
        }, expected=202)["capture"]
        return state, command

    def test_pairing_and_surface_permissions_prevent_motor_access(self):
        self.json_request("phone", "POST", "/capture/phone/pair", {"code": "invalid"}, expected=403)
        self.json_request("phone", "GET", "/capture/phone/commands", expected=401)
        self.pair_http()
        for method, path, body in (
            ("POST", "/api/command", b"M 100 100"),
            ("POST", "/api/session", b""),
            ("GET", "/api/info", None),
            ("POST", "/capture/api/capture", {"request_id": "unauthorised"}),
            ("GET", "/capture/api/state", None),
        ):
            with self.subTest(method=method, path=path):
                status, unused_headers, unused_body = self.request("phone", method, path, body, self.phone_headers)
                self.assertEqual(status, 403)
        status, unused_headers, unused_body = self.request("operator", "POST", "/capture/phone/status", {})
        self.assertEqual(status, 403)
        status, unused_headers, unused_body = self.request("operator", "GET", "/capture/phone/commands")
        self.assertIn(status, {403, 404})
        self.network.assert_not_called()

    def test_origin_host_and_cross_site_write_rejection(self):
        for headers in (
            {"Origin": "https://untrusted.example"},
            {"Host": "untrusted.example"},
            {"Host": "127.0.0.1:1"},
        ):
            with self.subTest(headers=headers):
                status, response_headers, unused_body = self.request("operator", "GET", "/capture/api/state", headers=headers)
                self.assertEqual(status, 403)
                self.assertNotIn("Access-Control-Allow-Origin", response_headers)
        status, unused_headers, unused_body = self.request("operator", "POST", "/capture/api/sessions", {}, {"Sec-Fetch-Site": "cross-site"})
        self.assertEqual(status, 403)
        self.json_request("operator", "GET", "/capture/api/state", headers={
            "Origin": f"http://127.0.0.1:{self.config.port}",
        })
        status, unused_headers, unused_body = self.request("phone", "POST", "/capture/phone/pair", {"code": "123456"}, {
            "Origin": f"http://127.0.0.1:{self.config.https_port}",
        })
        self.assertEqual(status, 403)
        self.json_request("phone", "POST", "/capture/phone/pair", {"code": "123456"}, {
            "Origin": f"https://127.0.0.1:{self.config.https_port}",
        })
        self.network.assert_not_called()

    def test_setup_serves_only_bootstrap_and_public_certificate(self):
        status, headers, certificate = self.request("setup", "GET", "/capture-ca.crt")
        self.assertEqual(status, 200)
        self.assertEqual(certificate, b"PUBLIC TEST CA CERTIFICATE")
        self.assertIn("attachment", headers["Content-Disposition"])
        for path in ("/.capture-tls/key.pem", "/rootCA-key.pem", "/capture/api/state", "/api/info", "/phone/"):
            with self.subTest(path=path):
                self.assertEqual(self.request("setup", "GET", path)[0], 404)
        self.assertEqual(self.request("setup", "POST", "/", {})[0], 405)
        self.assertEqual(self.request("setup", "GET", "/")[0], 200)
        self.network.assert_not_called()

    def test_http_capture_download_thumbnail_and_zip_round_trip(self):
        state, command = self.capture_http()
        session_id = state["session"]["id"]
        pending = self.json_request("phone", "GET", "/capture/phone/commands", headers=self.phone_headers)
        self.assertEqual(pending["commands"][0]["id"], command["id"])
        self.json_request("operator", "GET", f"/capture/api/export/{session_id}.zip", expected=409)
        photo = self.json_request("phone", "POST", f"/capture/phone/upload?command_id={command['id']}", self.jpeg, {
            **self.phone_headers, "Content-Type": "image/jpeg", "X-Frame-Metadata": '{"captured_at":"fixture","zoom":1}',
        })["photo"]
        status, headers, downloaded = self.request("operator", "GET", photo["url"])
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "image/jpeg")
        self.assertEqual(downloaded, self.jpeg)
        status, headers, head_body = self.request("operator", "HEAD", photo["url"])
        self.assertEqual(status, 200)
        self.assertEqual(head_body, b"")
        self.assertEqual(int(headers["Content-Length"]), len(self.jpeg))
        thumbnail = self.request("operator", "GET", photo["thumbnail_url"])[2]
        with Image.open(io.BytesIO(thumbnail)) as image:
            self.assertEqual(image.format, "JPEG")
            self.assertLessEqual(image.width, 320)
            self.assertLessEqual(image.height, 240)
        self.json_request("operator", "POST", f"/capture/api/stations/{state['active_station_id']}/finish", {})
        self.json_request("operator", "POST", "/capture/api/stations", {}, expected=201)
        folder = self.data_dir / session_id
        (folder / "private-key.pem").write_text("not an export asset")
        outside = Path(self.temporary.name) / "outside.jpg"
        outside.write_text("outside content")
        (folder / "outside-link.jpg").symlink_to(outside)
        status, headers, archive = self.request("operator", "GET", f"/capture/api/export/{session_id}.zip")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "application/zip")
        with zipfile.ZipFile(io.BytesIO(archive)) as zipped:
            names = zipped.namelist()
            self.assertIn(f"{session_id}/session.json", names)
            image_path = f"{session_id}/{state['active_station_id']}/{photo['filename']}"
            self.assertEqual(zipped.read(image_path), self.jpeg)
            self.assertNotIn(f"{session_id}/private-key.pem", names)
            self.assertNotIn(f"{session_id}/outside-link.jpg", names)
            self.assertTrue(all(name.startswith(session_id + "/") and ".." not in Path(name).parts for name in names))
            manifest = json.loads(zipped.read(f"{session_id}/{state['active_station_id']}/manifest.json"))
            self.assertEqual(manifest["photos"][0]["sha256"], hashlib.sha256(self.jpeg).hexdigest())
        self.network.assert_not_called()

    def test_operator_cancellation_stops_pending_phone_upload(self):
        unused_state, command = self.capture_http()
        cancelled = self.json_request("operator", "POST", "/capture/api/capture/cancel", {
            "request_id": "http-request",
        })
        self.assertTrue(cancelled["cancelled"])
        self.assertEqual(self.json_request("phone", "GET", "/capture/phone/commands", headers=self.phone_headers)["commands"], [])
        self.json_request("phone", "POST", f"/capture/phone/upload?command_id={command['id']}", self.jpeg, {
            **self.phone_headers, "Content-Type": "image/jpeg",
        }, expected=409)
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])
        self.json_request("phone", "POST", "/capture/api/capture/cancel", {"request_id": "http-request"},
                          self.phone_headers, expected=403)
        tombstone = self.json_request("operator", "POST", "/capture/api/capture/cancel", {"request_id": "delayed-http"})
        self.assertEqual(tombstone["capture"]["status"], "cancelled")
        self.json_request("operator", "POST", "/capture/api/capture/cancel", {"request_id": "../bad"}, expected=400)

    def test_invalid_uploads_and_metadata_are_rejected_before_storage(self):
        unused_state, command = self.capture_http()
        path = f"/capture/phone/upload?command_id={command['id']}"
        self.json_request("phone", "POST", path, self.jpeg, self.phone_headers, expected=415)
        for metadata in ("not-json", "[]", '{"value":NaN}'):
            with self.subTest(metadata=metadata):
                self.json_request("phone", "POST", path, self.jpeg, {
                    **self.phone_headers, "Content-Type": "image/jpeg", "X-Frame-Metadata": metadata,
                }, expected=400)
        self.json_request("phone", "POST", path, b"not a JPEG", {
            **self.phone_headers, "Content-Type": "image/jpeg",
        }, expected=415)
        self.json_request("phone", "POST", path, b"x", {
            **self.phone_headers, "Content-Type": "image/jpeg", "Content-Length": str(capture_server.MAX_PHOTO_BYTES + 1),
        }, expected=413)
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])
        self.assertEqual(self.store.state()["capture"]["status"], "queued")

    def test_traversal_and_malformed_requests_do_not_escape_or_start_scans(self):
        for path in (
            "/../capture_tls.py", "/capture/api/photos/../../outside.jpg",
            "/capture/api/photos/%2e%2e%2foutside.jpg", "/capture/api/export/..%2foutside.zip",
            "/.capture-tls/key.pem", "/capture_server.py",
        ):
            with self.subTest(path=path):
                self.assertEqual(self.request("operator", "GET", path)[0], 404)
        for malformed in (b"not json", b"[]", b"null"):
            self.json_request("operator", "POST", "/capture/api/sessions", malformed, expected=400)
        self.assertIsNone(self.store.session)
        self.json_request("operator", "POST", "/capture/api/sessions", b"x", {
            "Transfer-Encoding": "chunked",
        }, expected=413)
        self.assertIsNone(self.store.session)
        self.network.assert_not_called()

    def test_existing_operator_relay_works_without_camera_and_is_mocked(self):
        class FakeResponse(io.BytesIO):
            status = 200
        self.network.side_effect = None
        self.network.return_value = FakeResponse(b'{"ok":true}')
        self.store.enabled = False
        result = self.json_request("operator", "POST", "/api/stop", b"STOP", {"X-Rover-Token": "fixture-token"})
        self.assertTrue(result["ok"])
        outbound = self.network.call_args.args[0]
        self.assertEqual(outbound.full_url, "http://must-not-contact.invalid/api/stop")
        self.assertEqual(outbound.data, b"STOP")
        self.assertEqual(outbound.get_method(), "POST")
        self.assertEqual(self.network.call_args.kwargs["timeout"], 2)

    def test_rtc_signaling_round_trip_and_surface_permissions(self):
        self.json_request("phone", "GET", "/capture/phone/rtc", expected=401)
        self.pair_http()
        self.assertIsNone(self.json_request("operator", "GET", "/capture/api/rtc")["rtc"])
        offered = self.json_request("operator", "POST", "/capture/api/rtc", {
            "action": "offer", "connection_id": RTC_ID, "sdp": RTC_OFFER,
        })
        self.assertEqual(offered["rtc"]["connection_id"], RTC_ID)
        remote = self.json_request("phone", "GET", "/capture/phone/rtc", headers=self.phone_headers)
        self.assertEqual(remote["rtc"]["offer"]["sdp"], RTC_OFFER)
        self.json_request("phone", "POST", "/capture/phone/rtc", {
            "action": "answer", "connection_id": RTC_ID, "sdp": RTC_ANSWER,
        }, self.phone_headers)
        answer = self.json_request("operator", "GET", "/capture/api/rtc")["rtc"]
        self.assertEqual(answer["answer"]["sdp"], RTC_ANSWER)
        for surface, route, headers in (
            ("phone", "/capture/api/rtc", self.phone_headers),
            ("operator", "/capture/phone/rtc", {}),
        ):
            with self.subTest(surface=surface, route=route):
                self.json_request(surface, "GET", route, headers=headers, expected=403)
                self.json_request(surface, "POST", route, {
                    "action": "status", "connection_id": RTC_ID, "state": "closed",
                }, headers, expected=403)
        self.json_request("operator", "POST", "/capture/api/rtc", {
            "action": "status", "connection_id": RTC_ID, "state": "closed",
        })
        self.assertEqual(self.json_request("phone", "GET", "/capture/phone/rtc", headers=self.phone_headers)["rtc"]["operator_state"], "closed")
        self.network.assert_not_called()

    def test_rtc_rejects_offline_camera_stale_answer_and_bad_input(self):
        self.json_request("operator", "POST", "/capture/api/rtc", {
            "action": "offer", "connection_id": RTC_ID, "sdp": RTC_OFFER,
        }, expected=409)
        self.pair_http()
        for identifier in (RTC_ID, RTC_ID_2):
            if identifier == RTC_ID_2:
                self.json_request("operator", "POST", "/capture/api/rtc", {
                    "action": "status", "connection_id": RTC_ID, "state": "closed",
                })
            self.json_request("operator", "POST", "/capture/api/rtc", {
                "action": "offer", "connection_id": identifier, "sdp": RTC_OFFER,
            })
        self.json_request("phone", "POST", "/capture/phone/rtc", {
            "action": "answer", "connection_id": RTC_ID, "sdp": RTC_ANSWER,
        }, self.phone_headers, expected=409)
        self.json_request("operator", "POST", "/capture/api/rtc", {
            "action": "offer", "connection_id": "../invalid", "sdp": RTC_OFFER,
        }, expected=400)
        self.json_request("phone", "POST", "/capture/phone/status", {
            "camera_ready": False, "visibility": "visible", "width": 0, "height": 0,
        }, self.phone_headers)
        self.assertEqual(self.json_request("operator", "GET", "/capture/api/rtc")["rtc"]["operator_state"], "closed")
        self.network.assert_not_called()


# Signaling stays in memory and cannot grant access to either motor commands or
# capture uploads. These fixtures test lifecycle contracts, not browser codecs.
RTC_OFFER = "v=0\r\no=- 1 1 IN IP4 127.0.0.1\r\ns=fixture\r\nt=0 0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\na=recvonly\r\n"
RTC_ANSWER = RTC_OFFER.replace("recvonly", "sendonly")
RTC_ID = "11223344-5566-4788-99aa-bbccddeeff00"
RTC_ID_2 = "22334455-6677-4899-aabb-ccddeeff0011"


class RTCCaptureStoreTests(StoreFixture):
    def offer(self, identifier=RTC_ID):
        return self.store.rtc_operator({"action": "offer", "connection_id": identifier, "sdp": RTC_OFFER})

    def test_camera_and_phone_authentication_are_required(self):
        self.assert_capture_error(409, self.offer)
        self.ready_phone()
        self.assert_capture_error(401, self.store.rtc_get, "wrong", phone=True)
        self.assert_capture_error(401, self.store.rtc_phone, "wrong", {
            "action": "answer", "connection_id": RTC_ID, "sdp": RTC_ANSWER,
        })
        rtc = self.offer()["rtc"]
        self.assertEqual(rtc["connection_id"], RTC_ID)
        self.assertEqual(self.store.rtc_get(self.token, phone=True)["rtc"]["offer"]["sdp"], RTC_OFFER)
        self.assertNotIn(self.token, json.dumps(self.store.rtc_get()))
        self.assertEqual(list(self.data_dir.rglob("*.jpg")), [])

    def test_answer_stale_generations_and_camera_restart(self):
        self.ready_phone()
        self.offer()
        answer = self.store.rtc_phone(self.token, {
            "action": "answer", "connection_id": RTC_ID, "sdp": RTC_ANSWER,
        })
        self.assertEqual(answer["rtc"]["answer"]["sdp"], RTC_ANSWER)
        self.store.rtc_operator({"action": "status", "connection_id": RTC_ID, "state": "closed"})
        self.offer(RTC_ID_2)
        self.assert_capture_error(409, self.store.rtc_phone, self.token, {
            "action": "answer", "connection_id": RTC_ID, "sdp": RTC_ANSWER,
        })
        self.assert_capture_error(409, self.store.rtc_operator, {
            "action": "status", "connection_id": RTC_ID, "state": "playing",
        })
        self.store.phone_status(self.token, {"camera_ready": False, "visibility": "visible", "width": 0, "height": 0})
        self.assertEqual(self.store.rtc_get()["rtc"]["operator_state"], "closed")
        self.ready_phone()
        self.assertEqual(self.offer("33445566-7788-49aa-bbcc-ddeeff001122")["rtc"]["connection_id"], "33445566-7788-49aa-bbcc-ddeeff001122")

    def test_phone_activity_does_not_extend_operator_preview_lease(self):
        self.ready_phone()
        self.offer()
        now_mono, now_wall = time.monotonic(), time.time()
        with patch.object(capture_server.time, "monotonic", return_value=now_mono + 9), \
             patch.object(capture_server.time, "time", return_value=now_wall + 9):
            # A fresh phone camera heartbeat must not revive an abandoned viewer.
            self.store.phone_status(self.token, {
                "camera_ready": True, "visibility": "visible", "width": 96, "height": 64,
            })
            self.assertEqual(self.store.rtc_get(self.token, phone=True)["rtc"]["operator_state"], "closed")
            self.assert_capture_error(409, self.store.rtc_phone, self.token, {
                "action": "status", "connection_id": RTC_ID, "state": "connected",
            })

    def test_signaling_inputs_are_bounded(self):
        self.ready_phone()
        for bad_id in ("../../escape", "", "x" * 300):
            with self.subTest(identifier=bad_id):
                self.assert_capture_error(400, self.offer, bad_id)
        for bad_sdp in (None, {}, "", "v=0\r\n" + "x" * 65536):
            with self.subTest(sdp_type=type(bad_sdp).__name__):
                with self.assertRaises(CaptureError) as raised:
                    self.store.rtc_operator({"action": "offer", "connection_id": RTC_ID, "sdp": bad_sdp})
                self.assertIn(raised.exception.status, (400, 413))
        self.offer()
        self.assert_capture_error(400, self.store.rtc_operator, {
            "action": "status", "connection_id": RTC_ID, "state": "unknown",
        })
        self.assert_capture_error(400, self.store.rtc_phone, self.token, {
            "action": "status", "connection_id": RTC_ID, "state": "playing",
        })


if __name__ == "__main__":
    unittest.main()
