"""Explicit, fail-stop access to the installed ``rover-wifi-v1`` firmware.

This module never creates a session until ``arm`` is called, never retries a
movement request, and has no movement heartbeat or reconnect loop. The caller
must provide fresh commands and stop on stale perception. PWM is expressed in
physical left/right coordinates. The current wiring keeps those channel sides
and reverses both motor polarities: physical PWM (left, right) is encoded as
wire PWM (-left, -right), matching the manual Wi-Fi controller.

The ESP32 remains the independent 350 ms final motor lease. A request deadline
does not prove delivery of STOP: ``stop`` reports its acknowledgement separately.
"""
from __future__ import annotations

import json
import math
from numbers import Integral
import re
import secrets
import threading
import time
from typing import Callable
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


PROTOCOL = "rover-wifi-v1"
MAX_SEQUENCE = 2_147_483_647
_TOKEN = re.compile(r"[0-9a-f]{32}\Z")


class TransportFault(RuntimeError):
    """Movement is disarmed; a fresh, explicit arm is required."""


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class WiFiMotorTransport:
    """One command at a time, with no queued or automatically replayed motion.

    ``request`` is injectable for offline tests. Its signature is
    ``(method, path, body_bytes_or_None, headers, timeout_seconds) -> dict``.
    It must not follow redirects. A bounded daemon wrapper also enforces an
    overall deadline, including slow DNS or trickled response bytes. A timed-out
    operation must finish before another arm can acquire a new motor session.

    A STOP invalidates the local command epoch *before* doing network I/O. A
    late acknowledgement cannot restore authority or update the active state.
    """

    def __init__(self, base_url: str, *, timeout: float = .15,
                 max_command_age_s: float = .15,
                 request: Callable | None = None,
                 clock: Callable[[], float] = time.monotonic):
        parsed = urlsplit(base_url)
        if (parsed.scheme not in {"http", "https"} or not parsed.hostname
                or parsed.username is not None or parsed.password is not None
                or parsed.path not in {"", "/"} or parsed.query or parsed.fragment):
            raise ValueError("Rover URL must be an HTTP(S) origin without credentials or a path")
        try:
            parsed.port
        except ValueError as error:
            raise ValueError("Invalid rover port") from error
        if not math.isfinite(timeout) or not 0. < timeout <= .15:
            raise ValueError("Request deadline must be positive and at most 150 ms")
        if not math.isfinite(max_command_age_s) or not 0. < max_command_age_s <= .15:
            raise ValueError("Command age bound must be positive and at most 150 ms")
        self.base_url = base_url.rstrip("/")
        self.timeout = float(timeout)
        self.max_command_age_s = float(max_command_age_s)
        self._clock = clock
        self._request_impl = request or self._http_request
        self._lock = threading.RLock()
        self._requests: set[threading.Event] = set()
        self._command_busy = False
        self._arming = False
        self._token: str | None = None
        self._epoch = 0
        self._last_started: float | None = None
        self._closed = False
        self.armed = False
        self.fault: str | None = None
        self.sequence = 0

    def _http_request(self, method, path, body, headers, timeout):
        request = Request(self.base_url + path, data=body, headers=headers, method=method)
        # A redirect must never replay a command or forward its session token.
        opener = build_opener(ProxyHandler({}), _NoRedirect)
        try:
            response = opener.open(request, timeout=timeout)
        except HTTPError as error:
            # Do not expose the response body or private session token in errors.
            raise TransportFault(f"Rover HTTP request rejected ({error.code})") from None
        with response:
            data = response.read(8193)
        if len(data) > 8192:
            raise TransportFault("Rover response exceeds the protocol limit")
        try:
            value = json.loads(data)
        except (ValueError, UnicodeError):
            raise TransportFault("Invalid rover JSON response") from None
        if not isinstance(value, dict):
            raise TransportFault("Rover response must be an object")
        return value

    def _request(self, method, path, body=None, token=None):
        headers = {"Content-Type": "text/plain", "Connection": "close"}
        if token is not None:
            headers["X-Rover-Token"] = token
        completed = threading.Event()
        result = []
        with self._lock:
            self._requests.add(completed)

        def perform():
            try:
                result.append((True, self._request_impl(method, path, body, headers, self.timeout)))
            except Exception as error:
                result.append((False, error))
            finally:
                with self._lock:
                    self._requests.discard(completed)
                completed.set()

        threading.Thread(target=perform, name="rover-http", daemon=True).start()
        if not completed.wait(self.timeout):
            raise TransportFault("Rover request deadline exceeded")
        success, value = result[0]
        if not success:
            if isinstance(value, TransportFault):
                raise value
            raise TransportFault("Rover network request failed") from None
        if not isinstance(value, dict):
            raise TransportFault("Rover response must be an object")
        return value

    @staticmethod
    def _state(message, expected_type, *, armed=None, sequence=None):
        if (message.get("protocol") != PROTOCOL or message.get("type") != expected_type
                or type(message.get("ready")) is not bool
                or type(message.get("armed")) is not bool
                or type(message.get("seq")) is not int
                or not 0 <= message["seq"] <= MAX_SEQUENCE
                or any(type(message.get(side)) is not int or not -255 <= message[side] <= 255
                       for side in ("left", "right"))):
            raise TransportFault("Invalid or incompatible rover state")
        if not message["ready"]:
            raise TransportFault("Rover motor outputs are not ready")
        if armed is not None and message["armed"] is not armed:
            raise TransportFault("Rover motor lease is not in the expected state")
        if sequence is not None and message["seq"] != sequence:
            raise TransportFault("Rover command sequence acknowledgement mismatched")
        # Applied values can lag the accepted command by a motor-task tick or
        # reversal pause; their equality to requested PWM is not an ACK contract.
        result = {key: value for key, value in message.items() if key != "token"}
        result["left"], result["right"] = -message["left"], -message["right"]
        return result

    def info(self):
        """Read firmware status without acquiring or renewing motor authority."""
        return self._state(self._request("GET", "/api/info"), "info")

    def arm(self, cancelled: Callable[[], bool] | None = None):
        """Explicitly acquire a zero-PWM session. Refuses an existing controller.

        An optional metadata-only cancellation predicate closes the gap between
        dispatching an arm worker and that worker entering this method. It is
        checked under the short state lock, never across network I/O.
        """
        with self._lock:
            if cancelled is not None and cancelled():
                raise TransportFault("Arming was cancelled")
            if self._closed:
                raise TransportFault("Transport is closed")
            if self.armed or self._arming or self._requests or self._command_busy:
                raise TransportFault("Stop and finish pending requests before arming")
            self._epoch += 1
            epoch = self._epoch
            self._arming = True
            started = self._clock()
        token = None
        try:
            message = self._request("POST", "/api/session", secrets.token_hex(16).encode())
            candidate = message.get("token")
            if isinstance(candidate, str) and _TOKEN.fullmatch(candidate):
                token = candidate
            else:
                raise TransportFault("Invalid rover session token")
            state = self._state(message, "hello", armed=True, sequence=0)
            if state["left"] != 0 or state["right"] != 0:
                raise TransportFault("New rover session did not start stopped")
            with self._lock:
                if epoch != self._epoch or self._closed or cancelled is not None and cancelled():
                    raise TransportFault("Arming was cancelled")
                self._token = token
                self.sequence = 0
                self._last_started = started
                self.armed = True
                self._arming = False
                self.fault = None
            return state
        except Exception as error:
            with self._lock:
                if epoch == self._epoch:
                    self.armed = False
                    self.fault = str(error)
            # Only the token this acquisition actually returned may be stopped.
            if token is not None:
                self._stop_token(token)
            raise
        finally:
            with self._lock:
                self._arming = False

    def _stop_token(self, token):
        try:
            state = self._state(self._request("POST", "/api/stop", b"", token),
                                "stopped", armed=False)
            return state["left"] == 0 and state["right"] == 0
        except Exception:
            return False

    def stop(self, reason="operator_stop"):
        """Disarm locally first; return whether the owned session acknowledged STOP.

        With no owned token there is no remote command to send. A failed STOP
        does not extend authority: the ESP32 independently expires its lease.
        """
        with self._lock:
            self._epoch += 1
            self.armed = False
            self.fault = str(reason)
            token, self._token = self._token, None
            self._last_started = None
        return self._stop_token(token) if token is not None else True

    def send_pwm(self, physical_left, physical_right, *, issued_at=None):
        """Send one fresh physical-side PWM command; any fault latches a STOP."""
        try:
            with self._lock:
                if not self.armed or self._closed or self._token is None:
                    raise TransportFault("Explicit arming is required")
                if self._command_busy:
                    raise TransportFault("A motor command is already in flight")
                if any(isinstance(value, bool) or not isinstance(value, Integral)
                       or not -255 <= value <= 255 for value in (physical_left, physical_right)):
                    raise TransportFault("Motor PWM must be an integer within [-255, 255]")
                now = self._clock()
                issued = now if issued_at is None else float(issued_at)
                if (not math.isfinite(issued) or issued > now + 1e-6
                        or now - issued > self.max_command_age_s):
                    raise TransportFault("Motor command is stale or has an invalid timestamp")
                if self._last_started is None or now - self._last_started >= .25:
                    raise TransportFault("Local motor lease expired; explicit rearming is required")
                if self.sequence >= MAX_SEQUENCE:
                    raise TransportFault("Motor command sequence limit reached")
                self.sequence += 1
                sequence, token, epoch = self.sequence, self._token, self._epoch
                self._command_busy = True
                self._last_started = now
            body = f"M {sequence} {-int(physical_left)} {-int(physical_right)}".encode()
            message = self._request("POST", "/api/command", body, token)
            state = self._state(message, "state", armed=True, sequence=sequence)
            with self._lock:
                if epoch != self._epoch or not self.armed:
                    raise TransportFault("Motor command was cancelled")
            return state
        except Exception as error:
            # Do not let an old request disarm a *new* session after cancellation.
            with self._lock:
                still_owned = "epoch" not in locals() or epoch == self._epoch
            if still_owned:
                self.stop(str(error))
            if isinstance(error, TransportFault):
                raise
            raise TransportFault("Motor command failed") from None
        finally:
            with self._lock:
                if "epoch" in locals():
                    self._command_busy = False

    def close(self):
        with self._lock:
            self._closed = True
        return self.stop("transport_closed")
