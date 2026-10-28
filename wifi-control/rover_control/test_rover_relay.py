"""Offline relay deadlines, direct-routing contracts, and sanitized diagnostics.

Outbound exchanges are injected; no tests contact the real rover or DNS.
"""
from __future__ import annotations

import errno
import json
from pathlib import Path
import socket
import sys
import threading
import time
from urllib.error import HTTPError, URLError

import pytest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import rover_relay
from rover_relay import Deadline, Origin, PATH_DEADLINES, RelayFault, RoverRelay, request_rover


def state(kind='info', **changes):
    value = dict(type=kind, protocol='rover-wifi-v1', ready=True, armed=False,
                 seq=0, left=0, right=0)
    value.update(changes)
    return json.dumps(value).encode()


class FakeRequest:
    def __init__(self, response=None, error=None):
        self.calls = []
        self.response = response or (200, state())
        self.error = error
    def __call__(self, *args):
        self.calls.append(args)
        if self.error is not None:
            raise self.error
        return self.response


@pytest.mark.parametrize('url', ['http://user:secret@rover', 'http://rover/path', 'http://rover?token=x',
    'http://rover#secret', 'ftp://rover', 'file:///tmp/rover', 'http://rover:0', 'http://rover:65536',
    'http://rover:bad', 'http://rover/\n', ' http://rover', 'http://rover\\other', '', None])
def test_invalid_origin_is_rejected_without_any_request(url):
    fake = FakeRequest()
    with pytest.raises(ValueError):
        RoverRelay(url, request=fake)
    assert fake.calls == []


def test_configured_target_and_cached_status_are_inert_and_token_free():
    fake = FakeRequest()
    relay = RoverRelay('http://192.168.1.91:8080/', request=fake)
    status = relay.status()
    assert status['target_origin'] == 'http://192.168.1.91:8080'
    assert status['events'] == [] and status['last_success_age_s'] is None
    assert status['retries_enabled'] is False and fake.calls == []


@pytest.mark.parametrize('path,method,kind,budget', [
    ('/api/info', 'GET', 'info', 1.2), ('/api/session', 'POST', 'hello', 1.2),
    ('/api/command', 'POST', 'state', .2), ('/api/stop', 'POST', 'stopped', .2)])
def test_single_exchange_uses_path_deadline_and_configured_target(path, method, kind, budget):
    fake = FakeRequest((200, state(kind)))
    relay = RoverRelay('http://192.168.1.91', request=fake)
    code, raw = relay.relay(method, path, None if method == 'GET' else b'command', token='private-token')
    assert code == 200 and json.loads(raw)['type'] == kind
    assert len(fake.calls) == 1
    origin, actual_method, actual_path, body, headers, timeout, control = fake.calls[0]
    assert origin.host == '192.168.1.91' and actual_method == method and actual_path == path
    assert 0 < timeout <= budget and control.cancelled.is_set()
    assert ('X-Rover-Token' in headers) is (method == 'POST')
    event = relay.status()['events'][0]
    assert event['attempt_count'] == 1 and event['result_code'] == 'ok'
    assert event['timeout_ms'] == round(budget * 1000)
    assert 'private-token' not in json.dumps(relay.status())


@pytest.mark.parametrize('error,code', [
    (TimeoutError('private-token'), 'upstream_timeout'),
    (URLError(TimeoutError('private-token')), 'upstream_timeout'),
    (ConnectionRefusedError(errno.ECONNREFUSED, 'private-token'), 'connection_refused'),
    (URLError(socket.gaierror(-2, 'private-token')), 'dns_failure'),
    (OSError(errno.ENETUNREACH, 'private-token'), 'host_or_network_unreachable'),
    (OSError(errno.EHOSTUNREACH, 'private-token'), 'host_or_network_unreachable'),
    (ConnectionResetError(errno.ECONNRESET, 'private-token'), 'connection_reset'),
    (OSError('private-token'), 'upstream_io_error')])
def test_transport_failures_have_controlled_codes_and_no_token_or_retry(error, code):
    fake = FakeRequest(error=error)
    relay = RoverRelay('http://rover.local', request=fake)
    status, raw = relay.relay('POST', '/api/command', b'M 1 20 20', 'private-token')
    assert status == 502 and json.loads(raw)['relay_code'] == code
    event = relay.status()['events'][0]
    assert event['result_code'] == code and event['delivery'] == 'unknown'
    assert 'private-token' not in json.dumps(relay.status()) and 'private-token' not in raw.decode()
    assert len(fake.calls) == 1


@pytest.mark.parametrize('http_status', [401, 409, 503])
def test_upstream_http_control_errors_preserve_status_and_json(http_status):
    raw = json.dumps({'error': 'Control lease expired.'}).encode()
    fake = FakeRequest((http_status, raw))
    relay = RoverRelay('http://rover.local', request=fake)
    status, response = relay.relay('POST', '/api/command', b'M 1 20 20')
    assert status == http_status and response == raw
    event = relay.status()['events'][0]
    assert event['result_code'] == 'upstream_http_error'
    assert event['upstream_http_status'] == http_status and event['delivery'] == 'unknown'
    assert len(fake.calls) == 1


@pytest.mark.parametrize('http_status', [401, 409])
def test_injected_http_error_preserves_http_status_without_raw_exception(http_status):
    fake = FakeRequest(error=HTTPError('http://rover.local', http_status, 'private-token', {}, None))
    relay = RoverRelay('http://rover.local', request=fake)
    status, raw = relay.relay('POST', '/api/command', b'M 1 20 20')
    assert status == http_status and 'private-token' not in raw.decode()
    assert relay.status()['events'][0]['result_code'] == 'upstream_http_error'


@pytest.mark.parametrize('response,code', [
    ((302, b''), 'redirect_refused'), ((307, b''), 'redirect_refused'),
    ((200, b'x' * 8193), 'response_too_large'),
    ((200, b'<html>Wrong device</html>'), 'invalid_json_or_protocol'),
    ((200, b'{"protocol":"wrong","type":"info"}'), 'invalid_json_or_protocol'),
    ((200, b'{"protocol":"rover-wifi-v1","type":"info","seq":NaN}'), 'invalid_json_or_protocol'),
    ((200, state(ready='yes')), 'invalid_json_or_protocol')])
def test_bad_upstream_response_is_rejected_without_redirect_or_replay(response, code):
    fake = FakeRequest(response)
    relay = RoverRelay('http://rover.local', request=fake)
    status, raw = relay.relay('GET', '/api/info')
    assert status == 502 and json.loads(raw)['relay_code'] == code
    assert len(fake.calls) == 1 and relay.status()['events'][0]['result_code'] == code


def test_explicit_probe_is_exactly_one_info_get_and_returns_no_secret_fields():
    fake = FakeRequest((200, state(token='private-token', session='private-session')))
    relay = RoverRelay('http://rover.local', request=fake)
    result = relay.probe()
    assert len(fake.calls) == 1
    _, method, path, body, headers, _, _ = fake.calls[0]
    assert (method, path, body) == ('GET', '/api/info', None)
    assert 'X-Rover-Token' not in headers
    assert result['board'] == dict(protocol='rover-wifi-v1', ready=True, armed=False, seq=0, left=0, right=0)
    assert 'private-' not in json.dumps(result)


def test_successful_session_token_is_returned_only_to_caller_never_diagnostics():
    raw = state('hello', token='a' * 32)
    relay = RoverRelay('http://rover.local', request=FakeRequest((200, raw)))
    assert relay.relay('POST', '/api/session', b'client-nonce')[1] == raw
    diagnostics = json.dumps(relay.status())
    assert 'a' * 32 not in diagnostics and 'client-nonce' not in diagnostics


def test_history_is_bounded_and_status_never_probes():
    fake = FakeRequest()
    relay = RoverRelay('http://rover.local', request=fake, history_limit=3)
    for _ in range(5):
        relay.relay('GET', '/api/info')
    result = relay.status()
    assert len(result['events']) == 3 and len(fake.calls) == 5
    assert result['last_success_age_s'] >= 0 and result['inflight_count'] == 0


def test_stop_is_parallel_to_blocked_command_and_deadline_cancels_it():
    entered, release = threading.Event(), threading.Event()
    calls = []
    def request(origin, method, path, body, headers, timeout, control):
        calls.append(path)
        if path == '/api/command':
            entered.set(); release.wait(1.)
            return 200, state('state')
        return 200, state('stopped')
    relay = RoverRelay('http://rover.local', request=request)
    result = []
    command = threading.Thread(target=lambda: result.append(relay.relay('POST', '/api/command', b'M 1 20 20')))
    command.start()
    assert entered.wait(.3)
    started = time.monotonic()
    assert relay.relay('POST', '/api/stop', b'')[0] == 200
    assert time.monotonic() - started < .15
    command.join(.5)
    assert not command.is_alive() and result[0][0] == 502
    assert calls.count('/api/command') == calls.count('/api/stop') == 1
    release.set()


def test_expired_upload_deadline_prevents_any_outbound_request():
    fake = FakeRequest((200, state('state')))
    relay = RoverRelay('http://rover.local', request=fake)
    status, raw = relay.relay('POST', '/api/command', b'M 1 20 20', started_at=time.monotonic() - .3)
    assert status == 502 and json.loads(raw)['relay_code'] == 'upstream_timeout'
    assert fake.calls == [] and relay.status()['events'][0]['attempt_count'] == 0


def test_dns_finishing_after_deadline_cannot_start_late_connection(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    created = []
    def resolve(*args, **kwargs):
        entered.set(); release.wait(1.)
        return [(socket.AF_INET, socket.SOCK_STREAM, 6, '', ('192.168.1.5', 80))]
    monkeypatch.setattr(rover_relay.socket, 'getaddrinfo', resolve)
    monkeypatch.setattr(rover_relay.socket, 'socket', lambda *args: created.append(args))
    relay = RoverRelay('http://rover.local')
    status, raw = relay.relay('POST', '/api/command', b'M 1 20 20')
    assert entered.is_set() and status == 502
    release.set(); time.sleep(.03)
    assert created == []


def test_deadline_closes_registered_socket_while_upstream_worker_is_blocked():
    class FakeSocket:
        closed = False
        def settimeout(self, value): pass
        def shutdown(self, how): pass
        def close(self): self.closed = True
    sock, release = FakeSocket(), threading.Event()
    def request(origin, method, path, body, headers, timeout, control):
        control.register(sock)
        release.wait(1.)
        return 200, state('state')
    relay = RoverRelay('http://rover.local', request=request)
    started = time.monotonic()
    assert relay.relay('POST', '/api/command', b'M 1 20 20')[0] == 502
    assert time.monotonic() - started < .4 and sock.closed
    release.set()


def test_direct_request_uses_target_not_environment_proxy_and_does_not_redirect(monkeypatch):
    calls = []
    class FakeSocket:
        def settimeout(self, value): pass
        def setsockopt(self, level, option, value): calls.append(('sockopt', level, option, value))
        def connect(self, address): calls.append(('connect', address))
        def close(self): pass
    class Response:
        status = 302
        def read(self, size):
            calls.append(('read', size)); return b''
    class Connection:
        sock = None
        def __init__(self, host, port, timeout): calls.append(('connection', host, port))
        def request(self, method, path, body, headers): calls.append(('request', method, path))
        def getresponse(self): return Response()
        def close(self): pass
    monkeypatch.setenv('HTTP_PROXY', 'http://must-not-contact.invalid:9999')
    monkeypatch.setattr(rover_relay.socket, 'getaddrinfo', lambda host, port, **kwargs:
        [(socket.AF_INET, socket.SOCK_STREAM, 6, '', ('192.168.1.5', port))])
    monkeypatch.setattr(rover_relay.socket, 'socket', lambda *args: FakeSocket())
    monkeypatch.setattr(rover_relay.http.client, 'HTTPConnection', Connection)
    control = Deadline(time.monotonic() + 1.)
    assert request_rover(Origin.parse('http://rover.local'), 'GET', '/api/info', None, {}, 1., control)[0] == 302
    assert ('connect', ('192.168.1.5', 80)) in calls
    assert sum(item[0] == 'request' for item in calls) == 1
    assert all('must-not-contact' not in repr(item) for item in calls)


def test_small_post_disables_nagle_before_real_http_client_sends_headers_and_body(monkeypatch):
    # Keep the actual http.client serializer so the regression covers its two
    # writes, rather than a fake request implementation that hides the split.
    calls = []
    class FakeSocket:
        nodelay = False
        def settimeout(self, value): pass
        def setsockopt(self, level, option, value):
            assert (level, option, value) == (socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.nodelay = True
            calls.append(('nodelay',))
        def connect(self, address): calls.append(('connect', address))
        def sendall(self, data):
            assert self.nodelay, 'Small POST writes must not wait for a delayed TCP ACK'
            calls.append(('send', bytes(data)))
        def close(self): pass
    class Response:
        status = 200
        def read(self, size): return state('state')
    monkeypatch.setattr(rover_relay.socket, 'getaddrinfo', lambda host, port, **kwargs:
        [(socket.AF_INET, socket.SOCK_STREAM, 6, '', ('192.168.1.5', port))])
    monkeypatch.setattr(rover_relay.socket, 'socket', lambda *args: FakeSocket())
    monkeypatch.setattr(rover_relay.http.client.HTTPConnection, 'getresponse', lambda self: Response())
    body = b'M 1 128 128'
    result = request_rover(Origin.parse('http://rover.local'), 'POST', '/api/command', body,
                          {'Content-Type': 'text/plain', 'Connection': 'close'}, .2,
                          Deadline(time.monotonic() + .2))
    assert result[0] == 200
    sends = [call[1] for call in calls if call[0] == 'send']
    assert len(sends) == 2 and sends[1] == body
    assert sends[0].startswith(b'POST /api/command HTTP/1.1\r\n')
    assert f'Content-Length: {len(body)}\r\n'.encode() in sends[0]
    assert b'Expect:' not in sends[0] and b'Transfer-Encoding:' not in sends[0]
    assert [call[0] for call in calls[:2]] == ['nodelay', 'connect']
    assert PATH_DEADLINES['/api/command'] == .2
