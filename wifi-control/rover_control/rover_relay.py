"""Bounded, single-attempt rover relay and token-free diagnostic history.

Requests use direct sockets, never environment proxies or redirects. A deadline
can report failure after a POST was delivered; it cannot certify motor state.
Only an explicit GET probe reads board state, and diagnostic reads never arm it.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import errno
import http.client
import json
import math
import socket
import ssl
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
import uuid

PROTOCOL = 'rover-wifi-v1'
PATH_DEADLINES = {'/api/command': .20, '/api/stop': .20,
                  '/api/session': 1.2, '/api/info': 1.2}
MAX_RESPONSE_BYTES = 8192


class RelayFault(RuntimeError):
    def __init__(self, code, *, upstream_status=None, errno_code=None):
        # These values, never exception messages, are suitable for diagnostics.
        super().__init__(code)
        self.code, self.upstream_status, self.errno_code = code, upstream_status, errno_code


@dataclass(frozen=True)
class Origin:
    scheme: str
    host: str
    port: int
    url: str

    @classmethod
    def parse(cls, value):
        if not isinstance(value, str) or any(ch.isspace() for ch in value):
            raise ValueError('Rover URL must be an HTTP(S) origin without credentials or a path.')
        try:
            parsed = urlsplit(value)
            port = parsed.port if parsed.port is not None else (443 if parsed.scheme == 'https' else 80)
        except ValueError:
            raise ValueError('Invalid rover URL or port.') from None
        if (parsed.scheme not in {'http', 'https'} or not parsed.hostname
                or parsed.username is not None or parsed.password is not None
                or parsed.path not in {'', '/'} or parsed.query or parsed.fragment
                or not 1 <= port <= 65535 or '\\' in parsed.netloc):
            raise ValueError('Rover URL must be an HTTP(S) origin without credentials or a path.')
        host = parsed.hostname
        authority = f'[{host}]' if ':' in host else host
        default = 443 if parsed.scheme == 'https' else 80
        if port != default:
            authority += f':{port}'
        return cls(parsed.scheme, host, port, f'{parsed.scheme}://{authority}')


def classify(error):
    """Map exceptions to fixed codes, without copying tokens, URLs or messages."""
    if isinstance(error, RelayFault):
        return error
    if isinstance(error, HTTPError):
        return RelayFault('upstream_http_error', upstream_status=error.code)
    if isinstance(error, URLError):
        return classify(error.reason) if isinstance(error.reason, BaseException) else RelayFault('upstream_io_error')
    if isinstance(error, (TimeoutError, socket.timeout)):
        return RelayFault('upstream_timeout')
    if isinstance(error, socket.gaierror):
        return RelayFault('dns_failure')
    code = getattr(error, 'errno', None)
    if isinstance(error, ConnectionRefusedError) or code == errno.ECONNREFUSED:
        return RelayFault('connection_refused', errno_code='ECONNREFUSED')
    if code in {errno.ENETUNREACH, errno.EHOSTUNREACH}:
        return RelayFault('host_or_network_unreachable', errno_code=errno.errorcode.get(code))
    if isinstance(error, ConnectionResetError) or code == errno.ECONNRESET:
        return RelayFault('connection_reset', errno_code='ECONNRESET')
    if isinstance(error, ssl.SSLError):
        return RelayFault('tls_failure')
    return RelayFault('upstream_io_error')


class Deadline:
    """Cancellation closes a registered socket without holding a network lock."""
    def __init__(self, end):
        self.end = end
        self.cancelled = threading.Event()
        self._lock = threading.Lock()
        self._socket = None

    def remaining(self):
        value = self.end - time.monotonic()
        if self.cancelled.is_set() or value <= 0:
            raise RelayFault('upstream_timeout')
        return value

    def register(self, sock):
        with self._lock:
            self._socket = sock
            cancelled = self.cancelled.is_set()
        if cancelled:
            sock.close()
            raise RelayFault('upstream_timeout')
        sock.settimeout(self.remaining())

    def close(self):
        self.cancelled.set()
        with self._lock:
            sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass


def request_rover(origin, method, path, body, headers, timeout, deadline):
    """One direct HTTP exchange. Injectable as a whole for offline tests.

    DNS may finish after the outer deadline, but the check after resolution
    prevents a late connection or movement request from starting afterward.
    There is one chosen address and no fallback, redirect or retry.
    """
    deadline.remaining()
    addresses = socket.getaddrinfo(origin.host, origin.port, type=socket.SOCK_STREAM)
    deadline.remaining()
    if not addresses:
        raise RelayFault('dns_failure')
    family, kind, protocol, _, address = addresses[0]
    sock = socket.socket(family, kind, protocol)
    deadline.register(sock)
    connection = http.client.HTTPConnection(origin.host, origin.port, timeout=timeout)
    try:
        # Assigning our own socket bypasses HTTPConnection.connect(), including
        # its TCP_NODELAY setting. Headers and a small POST body are sent in
        # separate writes; Nagle plus the peer's delayed ACK can otherwise hold
        # the body for about 200 ms and exhaust the control request deadline.
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError as error:
            if error.errno != errno.ENOPROTOOPT:
                raise
        sock.connect(address)
        deadline.remaining()
        if origin.scheme == 'https':
            sock.settimeout(deadline.remaining())
            sock = ssl.create_default_context().wrap_socket(sock, server_hostname=origin.host)
            deadline.register(sock)
        # Supplying the connected socket prevents http.client from doing another
        # DNS lookup or connecting after this deadline's cancellation.
        connection.sock = sock
        sock.settimeout(deadline.remaining())
        connection.request(method, path, body=body, headers=headers)
        sock.settimeout(deadline.remaining())
        response = connection.getresponse()
        data = response.read(MAX_RESPONSE_BYTES + 1)
        deadline.remaining()
        return response.status, data
    finally:
        connection.close()
        sock.close()


def _object(raw):
    try:
        value = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
        json.dumps(value, allow_nan=False)
    except (ValueError, TypeError, UnicodeError, RecursionError):
        raise RelayFault('invalid_json_or_protocol') from None
    if not isinstance(value, dict):
        raise RelayFault('invalid_json_or_protocol')
    return value


def _state(value, expected):
    if (not isinstance(value, dict) or value.get('type') != expected
            or value.get('protocol') != PROTOCOL
            or type(value.get('ready')) is not bool or type(value.get('armed')) is not bool
            or type(value.get('seq')) is not int or not 0 <= value['seq'] <= 2147483647
            or any(type(value.get(side)) is not int or not -255 <= value[side] <= 255
                   for side in ('left', 'right'))):
        raise RelayFault('invalid_json_or_protocol')


def _board(value):
    """GET-only board summary, with a fixed allowlist and no secret fields."""
    _state(value, 'info')
    return {key: value[key] for key in ('protocol', 'ready', 'armed', 'seq', 'left', 'right')}


class RoverRelay:
    """Thread-safe diagnostic metadata; network operations never hold its lock."""
    def __init__(self, target, *, request=None, history_limit=64):
        self.origin = Origin.parse(target)
        if type(history_limit) is not int or not 1 <= history_limit <= 256:
            raise ValueError('Invalid relay history limit.')
        self.request = request or request_rover
        self._lock = threading.Lock()
        self._history = deque(maxlen=history_limit)
        self._inflight = 0
        self._consecutive_failures = 0
        self._last_success = None

    @staticmethod
    def timeout_for(path):
        return PATH_DEADLINES[path]

    def _begin(self, method, path, started_at):
        with self._lock:
            self._inflight += 1
        return {'request_id': uuid.uuid4().hex, 'method': method, 'path': path,
                'target_origin': self.origin.url,
                'started_at_utc': datetime.now(timezone.utc).isoformat(),
                'timeout_ms': round(PATH_DEADLINES[path] * 1000),
                'attempt_count': 0, '_started': started_at}

    def _finish(self, event, code, *, upstream_status=None, errno_code=None):
        finished = time.monotonic()
        event = dict(event)
        event['elapsed_ms'] = round(max(0., finished - event.pop('_started')) * 1000, 2)
        event.update(result_code=code, upstream_http_status=upstream_status, errno_code=errno_code,
                     delivery='read_only' if event['method'] == 'GET' else
                     'acknowledged' if code == 'ok' else 'unknown')
        with self._lock:
            self._inflight -= 1
            self._history.append(event)
            if code == 'ok':
                self._consecutive_failures = 0
                self._last_success = finished
            else:
                self._consecutive_failures += 1
        return event

    @staticmethod
    def _error(code, status=502):
        messages = {'upstream_timeout': 'Rover Wi-Fi request timed out.',
                    'connection_refused': 'Rover refused the Wi-Fi connection.',
                    'dns_failure': 'Rover hostname could not be resolved.',
                    'host_or_network_unreachable': 'Rover network address is unreachable.',
                    'connection_reset': 'Rover reset the Wi-Fi connection.',
                    'request_body_timeout': 'Control request upload timed out.',
                    'incomplete_request_body': 'Control request upload was interrupted.',
                    'redirect_refused': 'Rover returned an unexpected redirect.',
                    'invalid_json_or_protocol': 'Rover returned an incompatible response.',
                    'response_too_large': 'Rover response exceeded the size limit.'}
        return status, json.dumps({'error': messages.get(code, 'Rover unavailable over Wi-Fi.'),
                                   'relay_code': code}).encode()

    def reject(self, method, path, code, *, status=400, started_at=None):
        event = self._begin(method, path, time.monotonic() if started_at is None else started_at)
        safe = code if code in {'request_body_timeout', 'incomplete_request_body'} else 'invalid_request'
        self._finish(event, safe)
        return self._error(safe, status)

    def relay(self, method, path, body=None, token=None, *, started_at=None):
        if path not in PATH_DEADLINES or method != ('GET' if path == '/api/info' else 'POST'):
            raise ValueError('Relay method/path is not allowlisted.')
        if (body is not None and not isinstance(body, bytes) or body is not None and len(body) > 1024
                or method == 'GET' and body is not None):
            raise ValueError('Invalid relay request body.')
        started = time.monotonic() if started_at is None else started_at
        if not isinstance(started, (int, float)) or not math.isfinite(started):
            raise ValueError('Invalid request start time.')
        event = self._begin(method, path, started)
        control = Deadline(started + PATH_DEADLINES[path])
        headers = {'Content-Type': 'text/plain', 'Connection': 'close'}
        # A read-only probe or info request never needs or forwards a lease token.
        if method == 'POST' and token:
            headers['X-Rover-Token'] = token
        completed, result = threading.Event(), []
        def perform():
            try:
                remaining = control.remaining()
                result.append((True, self.request(self.origin, method, path, body, headers, remaining, control)))
            except Exception as error:
                result.append((False, classify(error)))
            finally:
                completed.set()
        try:
            remaining = control.remaining()
            event['attempt_count'] = 1
            threading.Thread(target=perform, daemon=True, name='rover-relay-http').start()
            if not completed.wait(control.remaining()):
                raise RelayFault('upstream_timeout')
            control.remaining()
            if not result[0][0]:
                raise result[0][1]
            status, raw = result[0][1]
            if type(status) is not int or not 100 <= status <= 599 or not isinstance(raw, bytes):
                raise RelayFault('invalid_json_or_protocol')
            if len(raw) > MAX_RESPONSE_BYTES:
                raise RelayFault('response_too_large', upstream_status=status)
            if 300 <= status <= 399:
                raise RelayFault('redirect_refused', upstream_status=status)
            if status >= 400:
                # Firmware 401/409 are meaningful control errors. Preserve both
                # their HTTP status and bounded JSON response without logging it.
                try:
                    _object(raw)
                except RelayFault:
                    raw = json.dumps({'error': f'Rover returned HTTP {status}.',
                                      'relay_code': 'upstream_http_error'}).encode()
                self._finish(event, 'upstream_http_error', upstream_status=status)
                return status, raw
            expected = {'/api/info': 'info', '/api/session': 'hello',
                        '/api/command': 'state', '/api/stop': 'stopped'}[path]
            try:
                _state(_object(raw), expected)
            except RelayFault:
                raise RelayFault('invalid_json_or_protocol', upstream_status=status) from None
            self._finish(event, 'ok', upstream_status=status)
            return status, raw
        except Exception as error:
            fault = classify(error)
            self._finish(event, fault.code, upstream_status=fault.upstream_status, errno_code=fault.errno_code)
            if (fault.code == 'upstream_http_error' and type(fault.upstream_status) is int
                    and 400 <= fault.upstream_status <= 599):
                return fault.upstream_status, json.dumps({
                    'error': f'Rover returned HTTP {fault.upstream_status}.',
                    'relay_code': 'upstream_http_error'}).encode()
            return self._error(fault.code)
        finally:
            control.close()

    def status(self):
        now = time.monotonic()
        with self._lock:
            return {'target_origin': self.origin.url, 'history_limit': self._history.maxlen,
                    'inflight_count': self._inflight, 'consecutive_failures': self._consecutive_failures,
                    'last_success_age_s': None if self._last_success is None else max(0., now - self._last_success),
                    'deadlines_ms': {path: round(value * 1000) for path, value in PATH_DEADLINES.items()},
                    'retries_enabled': False, 'events': [dict(event) for event in self._history]}

    def probe(self):
        status, raw = self.relay('GET', '/api/info')
        board = None
        if status == 200:
            try:
                board = _board(_object(raw))
            except RelayFault:
                pass
        result = self.status()
        result.update(probe_http_status=status, board=board)
        return result
