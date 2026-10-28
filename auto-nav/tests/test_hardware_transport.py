"""Offline motor protocol and cancellation tests. No real hardware is contacted."""
from __future__ import annotations

import threading
import time

import pytest

from rover_nav.hardware_transport import MAX_SEQUENCE, TransportFault, WiFiMotorTransport, _NoRedirect


class Clock:
    def __init__(self):
        self.now = 1.

    def __call__(self):
        return self.now


class Firmware:
    def __init__(self):
        self.calls = []
        self.armed = False
        self.sequence = 0
        self.left = self.right = 0
        self.token = "1" * 32
        self.change = None

    def state(self, kind):
        return dict(type=kind, protocol="rover-wifi-v1", ready=True,
                    armed=self.armed, seq=self.sequence, left=self.left, right=self.right)

    def __call__(self, method, path, body, headers, timeout):
        self.calls.append((method, path, body, dict(headers), timeout))
        assert timeout <= .15
        if path == "/api/info":
            return self.state("info")
        if path == "/api/session":
            assert method == "POST" and len(body) == 32
            assert "X-Rover-Token" not in headers
            if self.armed:
                raise RuntimeError("Another controller is connected")
            self.armed = True
            self.sequence = self.left = self.right = 0
            return dict(self.state("hello"), token=self.token)
        assert headers["X-Rover-Token"] == self.token
        if path == "/api/stop":
            self.armed = False
            self.left = self.right = 0
            return self.state("stopped")
        assert path == "/api/command" and method == "POST"
        prefix, sequence, left, right = body.decode().split()
        assert prefix == "M"
        if not self.armed or int(sequence) <= self.sequence:
            self.armed = False
            raise RuntimeError("Motor lease expired")
        self.sequence, self.left, self.right = int(sequence), int(left), int(right)
        result = self.state("state")
        if self.change:
            self.change(result)
        return result


def transport(**kwargs):
    firmware, clock = Firmware(), Clock()
    link = WiFiMotorTransport("http://rover.local", request=firmware, clock=clock, **kwargs)
    return link, firmware, clock


def test_construct_and_info_have_no_motor_authority_or_post():
    link, firmware, _ = transport()
    assert firmware.calls == [] and not link.armed
    assert link.info()["armed"] is False
    assert [call[:2] for call in firmware.calls] == [("GET", "/api/info")]


def test_explicit_arm_sequence_and_installed_physical_side_mapping():
    link, firmware, clock = transport()
    hello = link.arm()
    assert hello["armed"] and "token" not in hello
    assert link.armed and link.sequence == 0
    first = link.send_pwm(-30, 80, issued_at=clock.now)
    assert firmware.calls[-1][2] == b"M 1 30 -80"
    assert (first["left"], first["right"]) == (-30, 80)
    clock.now += .1
    link.send_pwm(0, 0, issued_at=clock.now)
    assert firmware.calls[-1][2] == b"M 2 0 0"
    assert link.stop()
    assert not link.armed and not firmware.armed
    with pytest.raises(TransportFault, match="Explicit arming"):
        link.send_pwm(1, 1)
    assert firmware.calls[-1][1] == "/api/stop"


def test_new_explicit_arm_is_required_after_stop_and_resets_sequence():
    link, firmware, _ = transport()
    link.arm()
    link.send_pwm(10, 10)
    link.stop()
    link.arm()
    assert link.sequence == 0 and link.fault is None
    link.send_pwm(0, 0)
    assert firmware.calls[-1][2] == b"M 1 0 0"


def test_existing_manual_controller_is_not_stopped_or_taken_over():
    link, firmware, _ = transport()
    firmware.armed = True
    with pytest.raises(TransportFault):
        link.arm()
    assert not link.armed and firmware.armed
    assert [call[1] for call in firmware.calls] == ["/api/session"]


@pytest.mark.parametrize("left,right", [(True, 1), (1.2, 1), (256, 0), (0, -256), ("1", 1)])
def test_invalid_pwm_never_reaches_command_and_disarms(left, right):
    link, firmware, _ = transport()
    link.arm()
    with pytest.raises(TransportFault, match="PWM"):
        link.send_pwm(left, right)
    assert not link.armed and not firmware.armed
    assert all(call[1] != "/api/command" for call in firmware.calls)


@pytest.mark.parametrize("timestamp", [.84, 1.01, float("nan"), float("inf")])
def test_old_future_or_nonfinite_command_is_rejected_before_wire(timestamp):
    link, firmware, _ = transport()
    link.arm()
    with pytest.raises(TransportFault, match="timestamp"):
        link.send_pwm(30, 30, issued_at=timestamp)
    assert not firmware.armed
    assert all(call[1] != "/api/command" for call in firmware.calls)


def test_long_gap_does_not_automatically_renew_or_restart_motor_lease():
    link, firmware, clock = transport()
    link.arm()
    clock.now += .25
    with pytest.raises(TransportFault, match="Local motor lease expired"):
        link.send_pwm(30, 30)
    assert not firmware.armed
    assert all(call[1] != "/api/command" for call in firmware.calls)


@pytest.mark.parametrize("replacement", [dict(seq=0), dict(seq=True), dict(armed=False),
                                        dict(ready=False), dict(protocol="other"),
                                        dict(left=256), dict(right=False), dict(type="hello")])
def test_rejected_or_malformed_command_ack_faults_and_stops(replacement):
    link, firmware, _ = transport()
    link.arm()
    firmware.change = lambda result: result.update(replacement)
    with pytest.raises(TransportFault):
        link.send_pwm(20, 20)
    assert not link.armed and not firmware.armed
    assert firmware.calls[-1][1] == "/api/stop"
    assert sum(call[1] == "/api/command" for call in firmware.calls) == 1


def test_applied_pwm_may_lag_accepted_command_during_motor_tick_or_reversal_pause():
    link, firmware, _ = transport()
    link.arm()
    firmware.change = lambda result: result.update(left=0, right=0)
    assert link.send_pwm(40, -40)["seq"] == 1
    assert link.armed


def test_sequence_limit_stops_without_wraparound():
    link, firmware, _ = transport()
    link.arm()
    link.sequence = MAX_SEQUENCE
    with pytest.raises(TransportFault, match="sequence limit"):
        link.send_pwm(10, 10)
    assert not firmware.armed
    assert all(call[1] != "/api/command" for call in firmware.calls)


def test_stop_failure_reports_unacknowledged_and_never_leaves_local_authority():
    link, firmware, _ = transport()
    link.arm()

    def failed(method, path, body, headers, timeout):
        raise OSError("unreachable")

    link._request_impl = failed
    assert link.stop() is False
    assert not link.armed
    with pytest.raises(TransportFault, match="Explicit arming"):
        link.send_pwm(20, 20)


def test_stop_invalidates_inflight_command_before_stop_network_and_late_ack():
    link, firmware, _ = transport()
    link.arm()
    entered, release = threading.Event(), threading.Event()
    errors = []

    def delayed(method, path, body, headers, timeout):
        if path == "/api/command":
            result = firmware(method, path, body, headers, timeout)
            entered.set()
            assert release.wait(1.)
            return result
        if path == "/api/stop":
            assert not link.armed
        return firmware(method, path, body, headers, timeout)

    link._request_impl = delayed

    def send():
        try:
            link.send_pwm(20, 20)
        except TransportFault as error:
            errors.append(str(error))

    worker = threading.Thread(target=send)
    worker.start()
    assert entered.wait(1.)
    assert link.stop("camera_lost")
    with pytest.raises(TransportFault, match="pending requests"):
        link.arm()
    release.set()
    worker.join(1.)
    assert not worker.is_alive()
    assert errors == ["Motor command was cancelled"]
    assert not link.armed and not firmware.armed
    assert link.fault == "camera_lost"


def test_stop_during_session_acquisition_cancels_and_disarms_late_hello():
    link, firmware, _ = transport()
    entered, release = threading.Event(), threading.Event()
    errors = []

    def delayed(method, path, body, headers, timeout):
        result = firmware(method, path, body, headers, timeout)
        if path == "/api/session":
            entered.set()
            assert release.wait(1.)
        return result

    link._request_impl = delayed

    def acquire():
        try:
            link.arm()
        except TransportFault as error:
            errors.append(str(error))

    worker = threading.Thread(target=acquire)
    worker.start()
    assert entered.wait(1.)
    link.stop("cancel_arm")
    with pytest.raises(TransportFault, match="pending requests"):
        link.arm()
    release.set()
    worker.join(1.)
    assert errors == ["Arming was cancelled"]
    assert not link.armed and not firmware.armed
    assert link.fault == "cancel_arm"
    link.arm()
    assert link.armed
    link.stop()


def test_malformed_hello_with_owned_token_gets_stopped():
    link, firmware, _ = transport()

    def invalid(method, path, body, headers, timeout):
        result = firmware(method, path, body, headers, timeout)
        if path == "/api/session":
            result["left"] = 1
        return result

    link._request_impl = invalid
    with pytest.raises(TransportFault, match="did not start stopped"):
        link.arm()
    assert not link.armed and not firmware.armed
    assert firmware.calls[-1][1] == "/api/stop"


def test_request_overall_deadline_latches_and_pending_operation_blocks_rearm():
    link, firmware, _ = transport(timeout=.03)
    link.arm()
    entered, release = threading.Event(), threading.Event()

    def stalled(method, path, body, headers, timeout):
        if path == "/api/command":
            entered.set()
            release.wait(1.)
        return firmware(method, path, body, headers, timeout)

    link._request_impl = stalled
    started = time.monotonic()
    with pytest.raises(TransportFault, match="deadline"):
        link.send_pwm(20, 20)
    assert entered.is_set() and time.monotonic() - started < .3
    assert not link.armed and not firmware.armed
    with pytest.raises(TransportFault, match="pending requests"):
        link.arm()
    release.set()


def test_close_stops_and_cannot_be_rearmed():
    link, firmware, _ = transport()
    link.arm()
    assert link.close()
    assert not link.armed and not firmware.armed
    with pytest.raises(TransportFault, match="closed"):
        link.arm()


@pytest.mark.parametrize("url", ["file:///etc/passwd", "http://u:p@rover.local", "http://rover.local/api",
                                  "http://rover.local/?x=1", "http://rover.local/#x", "http://rover.local:bad"])
def test_url_cannot_embed_credentials_or_change_protocol_destination(url):
    with pytest.raises(ValueError):
        WiFiMotorTransport(url)


@pytest.mark.parametrize("kwargs", [dict(timeout=.351), dict(timeout=0), dict(timeout=float("nan")),
                                     dict(max_command_age_s=.16), dict(max_command_age_s=-1)])
def test_unbounded_request_or_command_age_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        WiFiMotorTransport("http://rover.local", **kwargs)


@pytest.mark.parametrize("code", [301, 302, 303, 307, 308])
def test_redirect_handler_refuses_to_create_replayed_command_or_forward_token(code):
    from urllib.request import Request
    request = Request("http://rover.local/api/command", data=b"M 1 40 40",
                      headers={"X-Rover-Token": "1" * 32}, method="POST")
    assert _NoRedirect().redirect_request(
        request, None, code, "redirect", {}, "http://elsewhere/api/command") is None


def test_http_backend_installs_no_redirect_and_no_proxy_handlers(monkeypatch):
    import rover_nav.hardware_transport as module
    from urllib.error import HTTPError
    from urllib.request import ProxyHandler
    installed = []

    class Opener:
        def open(self, request, timeout):
            assert request.full_url == "http://rover.local/api/session"
            assert request.get_method() == "POST"
            assert timeout <= .15
            raise HTTPError(request.full_url, 307, "redirect", {}, None)

    def build(*handlers):
        installed.extend(handlers)
        return Opener()

    monkeypatch.setattr(module, "build_opener", build)
    link = WiFiMotorTransport("http://rover.local")
    with pytest.raises(TransportFault, match="307"):
        link.arm()
    assert any(handler is _NoRedirect for handler in installed)
    assert any(isinstance(handler, ProxyHandler) and handler.proxies == {} for handler in installed)



def test_arm_cancelled_before_dispatch_has_no_request():
    link, firmware, _ = transport()
    cancelled = threading.Event()
    cancelled.set()
    with pytest.raises(TransportFault, match="cancelled"):
        link.arm(cancelled=cancelled.is_set)
    assert firmware.calls == [] and not link.armed and not firmware.armed


def test_arm_predicate_cancelled_at_zero_ack_releases_own_token():
    link, firmware, _ = transport()
    cancelled = threading.Event()
    request = firmware
    def cancel_after_reply(*args):
        value = request(*args)
        if args[1] == "/api/session":
            cancelled.set()
        return value
    link._request_impl = cancel_after_reply
    with pytest.raises(TransportFault, match="cancelled"):
        link.arm(cancelled=cancelled.is_set)
    assert not link.armed and not firmware.armed
    assert [call[1] for call in firmware.calls] == ["/api/session", "/api/stop"]
    with pytest.raises(TransportFault):
        link.send_pwm(40, 40)


@pytest.mark.parametrize("physical,wire", [
    ((0, 0), b"M 1 0 0"),
    ((128, 128), b"M 1 -128 -128"),  # physical forward
    ((-128, -128), b"M 1 128 128"),  # physical reverse
    ((100, -100), b"M 1 -100 100"),  # clockwise pivot
    ((-100, 100), b"M 1 100 -100"),  # counterclockwise pivot
    ((30, 80), b"M 1 -30 -80"),  # forward arc keeps its wheel sides
    ((-30, -80), b"M 1 30 80"),  # reverse arc
    ((0, 80), b"M 1 0 -80"),  # stationary left wheel
    ((255, -255), b"M 1 -255 255"),  # signed protocol limits
    ((-255, 255), b"M 1 255 -255"),
])
def test_polarity_mapping_preserves_sides_for_straight_pivots_arcs_and_limits(physical, wire):
    link, firmware, clock = transport()
    link.arm()
    result = link.send_pwm(*physical, issued_at=clock.now)
    assert firmware.calls[-1][2] == wire
    assert (result["left"], result["right"]) == physical
    assert link.stop()
    assert firmware.left == firmware.right == 0


def test_read_only_applied_pwm_status_uses_inverse_polarity_without_channel_swap():
    link, firmware, _ = transport()
    firmware.left, firmware.right = -20, 60
    result = link.info()
    assert (result["left"], result["right"]) == (20, -60)
    assert [call[:2] for call in firmware.calls] == [("GET", "/api/info")]
    assert not link.armed
