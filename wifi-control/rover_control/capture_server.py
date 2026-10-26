"""Filesystem-backed manual capture sessions. No motor commands or database."""
from __future__ import annotations

import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import secrets
import threading
import time
from datetime import datetime, timezone

from PIL import Image, ImageOps, UnidentifiedImageError

Image.MAX_IMAGE_PIXELS = 50_000_000
IDENTIFIER = re.compile(r"^[a-zA-Z0-9_-]{1,80}$")
CAPTURE_TTL = 30
MAX_PHOTO_BYTES = 20 * 1024 * 1024
MAX_PREVIEW_BYTES = 512 * 1024
RTC_LEASE_SECONDS = 8


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class CaptureError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


class CaptureStore:
    def __init__(self, data_dir, *, enabled=True, pairing=None, error=None):
        self.root = Path(data_dir).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.enabled = enabled
        self.error = error
        self.pairing = pairing or {}
        self.pair_code = str(self.pairing.get("code", ""))
        self.phone_token = None
        self.phone = {"camera_ready": False, "visibility": "hidden", "width": 0, "height": 0}
        self.last_seen = 0
        self.last_camera_status = 0
        self.preview = None
        self.preview_seq = 0
        self.preview_at = 0
        self.session = None
        self.active_station_id = None
        self.commands = {}
        self.latest_command_id = None
        self.request_ids = {}
        self.pair_attempts = {}
        self.rtc = None
        self.rtc_operator_seen = 0
        self.rtc_closed_ids = set()
        self._restore()

    def _restore(self):
        for folder in sorted(self.root.iterdir(), reverse=True):
            if not folder.is_dir() or not IDENTIFIER.fullmatch(folder.name):
                continue
            try:
                session = json.loads((folder / "session.json").read_text())
                if session.get("id") != folder.name or session.get("version") != 1:
                    continue
                self.session = session
                for station in session.get("stations", []):
                    if station.get("status") == "open":
                        self.active_station_id = station["id"]
                    # Complete image sidecars recover an interrupted manifest update.
                    known = {photo["id"] for photo in station.get("photos", [])}
                    station_dir = folder / station["id"]
                    for sidecar in sorted(station_dir.glob("*.json")):
                        if sidecar.name == "manifest.json":
                            continue
                        try:
                            photo = json.loads(sidecar.read_text())
                            if photo["id"] not in known and (station_dir / photo["filename"]).is_file():
                                station["photos"].append(photo)
                                known.add(photo["id"])
                        except (OSError, ValueError, KeyError, TypeError):
                            continue
                self._save()
                break
            except (OSError, ValueError, KeyError, TypeError):
                continue

    def _save(self):
        if not self.session:
            return
        folder = self.root / self.session["id"]
        # Each station also has a standalone reconstruction input manifest.
        for station in self.session["stations"]:
            atomic_json(folder / station["id"] / "manifest.json", {
                "version": 1, "session_id": self.session["id"],
                "acquisition": "manual-rgb-browser", "pose_source": None,
                "depth_source": None, **station,
            })
        atomic_json(folder / "session.json", self.session)

    def _require_enabled(self):
        if not self.enabled:
            raise CaptureError(self.error or "Phone capture is unavailable.", 503)

    def _expire(self):
        command = self.commands.get(self.latest_command_id)
        if command and command["status"] == "queued" and time.time() > command["expires_at"]:
            command.update(status="expired", error="Phone capture timed out. Check that its camera page is visible.")
        return command

    def _require_no_capture(self):
        command = self._expire()
        if command and command["status"] == "queued":
            raise CaptureError("Wait for the current capture to finish.", 409)

    def _station(self, identifier=None):
        if not self.session:
            raise CaptureError("Start a scan first.", 409)
        identifier = identifier or self.active_station_id
        for station in self.session["stations"]:
            if station["id"] == identifier:
                return station
        raise CaptureError("Capture position was not found.", 404)

    def _phone_state(self):
        age = time.monotonic() - self.last_seen
        camera_age = time.monotonic() - self.last_camera_status
        connected = bool(self.phone_token) and age < 6
        ready = connected and camera_age < 6 and self.phone.get("camera_ready", False) and self.phone.get("visibility") == "visible"
        return {**self.phone, "connected": connected, "camera_ready": ready,
                "last_seen": self.phone.get("last_seen"), "age_ms": round(age * 1000) if self.last_seen else None}

    def state(self):
        with self.lock:
            command = self._expire()
            self._rtc_expire()
            session = self.session
            result = {
                "version": 1, "enabled": self.enabled, "error": self.error,
                "session": {key: value for key, value in session.items() if key != "stations"} if session else None,
                "stations": session["stations"] if session else [],
                "active_station_id": self.active_station_id,
                "phone": self._phone_state(), "capture": command,
                "realtime_preview": {"available": True, "connection_id": self.rtc.get("connection_id") if self.rtc else None,
                                     "operator_state": self.rtc.get("operator_state") if self.rtc else None},
                "preview": {"seq": self.preview_seq, "url": f"/capture/api/preview.jpg?seq={self.preview_seq}",
                            "age_ms": round((time.monotonic() - self.preview_at) * 1000)} if self.preview else None,
                "pairing": self.pairing, "storage_path": str(self.root),
                "export_url": f"/capture/api/export/{session['id']}.zip" if session else None,
            }
            return copy.deepcopy(result)

    def new_session(self):
        with self.lock:
            self._require_enabled()
            self._require_no_capture()
            if self.session:
                for station in self.session["stations"]:
                    if station["status"] == "open":
                        station["status"] = "ready" if station["photos"] else "empty"
                        station["finished_at"] = utc_now()
                self.session["closed_at"] = utc_now()
                self._save()
            identifier = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + secrets.token_hex(3)
            self.session = {"version": 1, "id": identifier, "created_at": utc_now(),
                            "acquisition": "manual-rgb-browser", "pose_source": None,
                            "depth_source": None, "metric_scale": None, "stations": []}
            self.active_station_id = None
            self.latest_command_id = None
            self._new_station()
            return self.state()

    def _new_station(self):
        number = len(self.session["stations"]) + 1
        station = {"id": f"position-{number:03d}", "number": number, "status": "open", "created_at": utc_now(), "photos": []}
        self.session["stations"].append(station)
        self.active_station_id = station["id"]
        self._save()
        return station

    def new_station(self):
        with self.lock:
            self._require_enabled()
            self._require_no_capture()
            if not self.session:
                raise CaptureError("Start a scan first.", 409)
            if self.active_station_id:
                raise CaptureError("Finish the current position before starting the next.", 409)
            self.latest_command_id = None
            self._new_station()
            return self.state()

    def finish_station(self, identifier):
        with self.lock:
            self._require_no_capture()
            station = self._station(identifier)
            if station["status"] == "ready":
                return self.state()
            if station["status"] != "open" or identifier != self.active_station_id:
                raise CaptureError("This position is not open.", 409)
            if not station["photos"]:
                raise CaptureError("Capture at least one photo before finishing this position.", 409)
            station.update(status="ready", finished_at=utc_now())
            self.active_station_id = None
            self._save()
            return self.state()

    def pair(self, code, device, address):
        with self.lock:
            self._require_enabled()
            now = time.monotonic()
            attempts = [stamp for stamp in self.pair_attempts.get(address, []) if now - stamp < 60]
            self.pair_attempts[address] = attempts
            if len(attempts) >= 8:
                raise CaptureError("Too many pairing attempts. Wait one minute.", 429)
            if not isinstance(code, str) or not secrets.compare_digest(code, self.pair_code):
                attempts.append(now)
                raise CaptureError("Pairing code does not match the laptop.", 403)
            if self._phone_state()["connected"]:
                raise CaptureError("A phone is already paired. Stop its camera and close that page before pairing another.", 409)
            self.phone_token = secrets.token_hex(32)
            self._rtc_close("Phone pairing changed.")
            self.phone = {"device": str(device)[:240], "camera_ready": False,
                          "visibility": "visible", "width": 0, "height": 0}
            self.last_camera_status = 0
            self._touch()
            return {"token": self.phone_token}

    def authenticate(self, token):
        if not isinstance(token, str) or not self.phone_token or not secrets.compare_digest(token, self.phone_token):
            raise CaptureError("Phone pairing expired. Pair again using the laptop code.", 401)

    def _touch(self):
        self.last_seen = time.monotonic()
        self.phone["last_seen"] = utc_now()

    def phone_status(self, token, values):
        with self.lock:
            self.authenticate(token)
            self._touch()
            self.last_camera_status = time.monotonic()
            for name in ("device", "error"):
                if name in values:
                    self.phone[name] = str(values[name])[:500] if values[name] else None
            self.phone["camera_ready"] = values.get("camera_ready") is True
            self.phone["visibility"] = "visible" if values.get("visibility") == "visible" else "hidden"
            for name in ("width", "height"):
                value = values.get(name, 0)
                self.phone[name] = value if isinstance(value, int) and 0 <= value <= 20000 else 0
            if self.phone["visibility"] != "visible" or not self.phone["width"] or not self.phone["height"]:
                self._rtc_close("Phone camera is off or hidden.")
            return {"ok": True}

    def _rtc_close(self, reason):
        if self.rtc:
            self.rtc.update(operator_state="closed", phone_state="closed", error=reason, expires_at=time.time())
            self.rtc_closed_ids.add(self.rtc["connection_id"])
            if len(self.rtc_closed_ids) > 128:
                self.rtc_closed_ids.pop()

    def _rtc_expire(self):
        if self.rtc and self.rtc.get("operator_state") != "closed" and time.monotonic() - self.rtc_operator_seen > RTC_LEASE_SECONDS:
            self._rtc_close("Video viewer heartbeat expired.")
        if self.rtc and (not self._phone_state()["connected"] or self.phone.get("visibility") != "visible"):
            self._rtc_close("Phone camera is unavailable.")

    @staticmethod
    def _rtc_sdp(value):
        if not isinstance(value, str) or not value.startswith("v=0") or len(value.encode()) > 65536:
            raise CaptureError("Invalid video session description.")
        if "m=video " not in value or "m=audio " in value:
            raise CaptureError("Only one-way camera video is supported.")
        return value

    def rtc_get(self, token=None, *, phone=False):
        with self.lock:
            if phone:
                self.authenticate(token)
                self._touch()
            self._rtc_expire()
            return {"rtc": copy.deepcopy(self.rtc)}

    def rtc_operator(self, values):
        with self.lock:
            self._require_enabled()
            self._rtc_expire()
            identifier = values.get("connection_id")
            if not isinstance(identifier, str) or not IDENTIFIER.fullmatch(identifier):
                raise CaptureError("A valid video connection ID is required.")
            action = values.get("action")
            if action == "offer":
                if identifier in self.rtc_closed_ids:
                    raise CaptureError("This video connection has already closed.", 409)
                if not self._phone_state()["camera_ready"]:
                    raise CaptureError("Start the phone camera before connecting video.", 409)
                sdp = self._rtc_sdp(values.get("sdp"))
                if self.rtc and self.rtc.get("operator_state") != "closed":
                    if self.rtc["connection_id"] == identifier and self.rtc["offer"]["sdp"] == sdp:
                        return {"rtc": copy.deepcopy(self.rtc)}
                    raise CaptureError("Another video connection is active. Wait for it to close.", 409)
                self.rtc_operator_seen = time.monotonic()
                self.rtc = {"connection_id": identifier, "offer": {"type": "offer", "sdp": sdp}, "answer": None,
                            "operator_state": "connecting", "phone_state": "connecting",
                            "created_at": utc_now(), "expires_at": time.time() + RTC_LEASE_SECONDS}
            elif action == "status":
                if not self.rtc or self.rtc["connection_id"] != identifier:
                    raise CaptureError("Video connection is no longer active.", 409)
                state = values.get("state")
                if state not in {"connecting", "playing", "failed", "closed"}:
                    raise CaptureError("Invalid viewer state.")
                if state in {"closed", "failed"}:
                    self._rtc_close("Video viewer closed." if state == "closed" else "Video viewer reconnecting.")
                elif self.rtc["operator_state"] == "closed":
                    raise CaptureError("Video connection has expired.", 409)
                else:
                    self.rtc_operator_seen = time.monotonic()
                    self.rtc.update(operator_state=state, expires_at=time.time() + RTC_LEASE_SECONDS)
                    for key in ("frames_decoded", "frames_per_second"):
                        value = values.get(key)
                        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and 0 <= value <= 1e12:
                            self.rtc[key] = value
            else:
                raise CaptureError("Unknown video action.")
            return {"rtc": copy.deepcopy(self.rtc)}

    def rtc_phone(self, token, values):
        with self.lock:
            self.authenticate(token)
            self._touch()
            self._rtc_expire()
            identifier = values.get("connection_id")
            if not self.rtc or self.rtc["connection_id"] != identifier or self.rtc["operator_state"] == "closed":
                raise CaptureError("Video connection is no longer active.", 409)
            action = values.get("action")
            if action == "answer":
                sdp = self._rtc_sdp(values.get("sdp"))
                if self.rtc["answer"] and self.rtc["answer"]["sdp"] != sdp:
                    raise CaptureError("A different answer already exists for this video connection.", 409)
                self.rtc["answer"] = {"type": "answer", "sdp": sdp}
            elif action == "status":
                state = values.get("state")
                if state not in {"connecting", "connected", "failed", "closed"}:
                    raise CaptureError("Invalid phone video state.")
                self.rtc["phone_state"] = state
                if values.get("error"):
                    self.rtc["error"] = str(values["error"])[:300]
            else:
                raise CaptureError("Unknown video action.")
            return {"rtc": copy.deepcopy(self.rtc)}

    def request_capture(self, station_id, request_id):
        with self.lock:
            self._require_enabled()
            if not isinstance(request_id, str) or not IDENTIFIER.fullmatch(request_id):
                raise CaptureError("A valid capture request ID is required.")
            if request_id in self.request_ids:
                return {"capture": copy.deepcopy(self.commands[self.request_ids[request_id]])}
            self._require_no_capture()
            station = self._station(station_id)
            if station["status"] != "open" or station_id != self.active_station_id:
                raise CaptureError("Start a new position before capturing.", 409)
            if not self._phone_state()["camera_ready"]:
                raise CaptureError("Open the phone page and start its camera first.", 409)
            identifier = secrets.token_hex(16)
            command = {"id": identifier, "type": "capture", "session_id": self.session["id"],
                       "station_id": station_id, "request_id": request_id, "status": "queued", "created_at": utc_now(),
                       "expires_at": time.time() + CAPTURE_TTL}
            self.commands[identifier] = command
            self.latest_command_id = identifier
            self.request_ids[request_id] = identifier
            return {"capture": copy.deepcopy(command)}

    def cancel_capture(self, request_id):
        with self.lock:
            if not isinstance(request_id, str) or not IDENTIFIER.fullmatch(request_id):
                raise CaptureError("A valid capture request ID is required.")
            identifier = self.request_ids.get(request_id)
            if identifier:
                command = self.commands[identifier]
                if command["status"] == "queued":
                    command.update(status="cancelled", error="Capture cancelled by operator.")
            else:
                # Reserve even unseen request IDs: a delayed POST cannot enqueue
                # a photo after the operator has already cancelled and resumed.
                identifier = secrets.token_hex(16)
                command = {"id": identifier, "type": "capture", "request_id": request_id,
                           "status": "cancelled", "created_at": utc_now(),
                           "error": "Capture cancelled by operator."}
                self.commands[identifier] = command
                self.request_ids[request_id] = identifier
            return {"cancelled": command["status"] == "cancelled", "request_id": request_id,
                    "capture": copy.deepcopy(command)}

    def phone_commands(self, token):
        with self.lock:
            self.authenticate(token)
            self._touch()
            command = self._expire()
            return {"commands": [copy.deepcopy(command)] if command and command["status"] == "queued" else []}

    @staticmethod
    def _image(data, *, preview=False):
        if not data or len(data) > (MAX_PREVIEW_BYTES if preview else MAX_PHOTO_BYTES):
            raise CaptureError("Image is empty or exceeds the upload limit.", 413)
        try:
            picture = Image.open(io.BytesIO(data))
            width, height = picture.size
            if picture.format != "JPEG" or width < 16 or height < 16 or width * height > (2_000_000 if preview else 50_000_000):
                raise CaptureError("Expected a JPEG within the supported dimensions.", 415)
            picture.load()
            return picture
        except (UnidentifiedImageError, OSError, ValueError, Image.DecompressionBombError) as error:
            raise CaptureError("The uploaded JPEG could not be decoded.", 415) from error

    def put_preview(self, token, data):
        with self.lock:
            self.authenticate(token)
        self._image(data, preview=True)
        with self.lock:
            self.authenticate(token)
            self._touch()
            self.preview = data
            self.preview_seq += 1
            self.preview_at = time.monotonic()
            return {"ok": True, "seq": self.preview_seq}

    def preview_bytes(self):
        with self.lock:
            if not self.preview:
                raise CaptureError("No camera preview yet.", 404)
            return self.preview

    def upload(self, token, command_id, data, metadata):
        with self.lock:
            self.authenticate(token)
            command = self.commands.get(command_id)
            if not command:
                raise CaptureError("Capture request was not found.", 404)
            checksum = hashlib.sha256(data).hexdigest()
            if command["status"] == "complete":
                photo = self._find_photo(command["photo_id"])[0]
                if photo["sha256"] != checksum:
                    raise CaptureError("This capture already has a different image.", 409)
                return {"photo": copy.deepcopy(photo), "duplicate": True}
            self._expire()
            if command["status"] != "queued":
                raise CaptureError("This capture has expired or failed. Request another photo.", 409)
        picture = self._image(data)
        thumbnail = ImageOps.exif_transpose(picture).convert("RGB")
        thumbnail.thumbnail((320, 240))
        thumbnail_buffer = io.BytesIO()
        thumbnail.save(thumbnail_buffer, format="JPEG", quality=78)
        with self.lock:
            self.authenticate(token)
            self._expire()
            if command["status"] == "complete":
                photo = self._find_photo(command["photo_id"])[0]
                if photo["sha256"] != checksum:
                    raise CaptureError("This capture already has a different image.", 409)
                return {"photo": copy.deepcopy(photo), "duplicate": True}
            if command["status"] != "queued" or not self.session or command["session_id"] != self.session["id"]:
                raise CaptureError("The capture is no longer active.", 409)
            station = self._station(command["station_id"])
            if station["status"] != "open":
                raise CaptureError("This position is already finished.", 409)
            number = len(station["photos"]) + 1
            filename = f"{number:04d}_{command_id}.jpg"
            folder = self.root / self.session["id"] / station["id"]
            folder.mkdir(parents=True, exist_ok=True)
            target = folder / filename
            temporary = target.with_suffix(".jpg.part")
            with temporary.open("wb") as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
            (folder / (target.stem + ".thumb.jpg")).write_bytes(thumbnail_buffer.getvalue())
            photo = {
                "id": command_id, "number": number, "filename": filename,
                "bytes": len(data), "width": picture.width, "height": picture.height,
                "sha256": checksum, "captured_at": str(metadata.get("captured_at", utc_now()))[:80],
                "received_at": utc_now(), "metadata": metadata,
                "url": f"/capture/api/photos/{command_id}.jpg",
                "thumbnail_url": f"/capture/api/photos/{command_id}.thumb.jpg",
                "camera_pose": None, "intrinsics": None, "depth": None,
            }
            atomic_json(folder / (target.stem + ".json"), photo)
            station["photos"].append(photo)
            self._save()
            command.update(status="complete", photo_id=command_id, completed_at=utc_now())
            self._touch()
            return {"photo": copy.deepcopy(photo)}

    def phone_error(self, token, command_id, error):
        with self.lock:
            self.authenticate(token)
            self._touch()
            command = self.commands.get(command_id)
            if command and command["status"] == "queued":
                command.update(status="error", error=str(error)[:500])
            return {"ok": True}

    def _find_photo(self, identifier):
        # Reads only server-created manifests; request paths are never filesystem paths.
        sessions = [self.session] if self.session else []
        for folder in self.root.iterdir():
            if not folder.is_dir() or (self.session and folder.name == self.session["id"]):
                continue
            try:
                sessions.append(json.loads((folder / "session.json").read_text()))
            except (OSError, ValueError):
                continue
        for session in sessions:
            for station in session.get("stations", []):
                for photo in station.get("photos", []):
                    if photo["id"] == identifier:
                        return photo, self.root / session["id"] / station["id"] / photo["filename"]
        raise CaptureError("Photo was not found.", 404)

    def photo_path(self, identifier, thumbnail=False):
        if not IDENTIFIER.fullmatch(identifier):
            raise CaptureError("Photo was not found.", 404)
        with self.lock:
            _, path = self._find_photo(identifier)
            return path.with_name(path.stem + ".thumb.jpg") if thumbnail else path

    def export_folder(self, identifier):
        if not IDENTIFIER.fullmatch(identifier):
            raise CaptureError("Scan was not found.", 404)
        folder = self.root / identifier
        if not (folder / "session.json").is_file():
            raise CaptureError("Scan was not found.", 404)
        with self.lock:
            self._require_no_capture()
        return folder
