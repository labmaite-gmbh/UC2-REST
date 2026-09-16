"""Regression tests for Motor.isBusy()'s fail-open bug.

isBusy() used to catch any exception while parsing post_json()'s response
and return False ("not busy") -- meaning a broken/timed-out serial exchange
(post_json() returns the string "communication interrupted" on that path,
see mserial.py) was silently indistinguishable from "the move genuinely
finished." ESP32Axis.wait_for_move_complete() (locai-impl) does
`while not self.is_move_complete(): ...`, so this made a dead connection
look like a completed move instead of raising, letting the caller act on an
unconfirmed hardware state. isBusy() must now raise CommunicationError
instead, so callers can tell "don't know" from "confirmed not busy" and
retry accordingly (see lm_hardware's esp32_conn.py call_with_retry).
"""
import pytest

from uc2rest.mserial import CommunicationError
from uc2rest.motor import Motor


class _StubParent:
    def __init__(self, response):
        self._response = response

    def post_json(self, path, payload, timeout=1):
        return self._response


def test_is_busy_raises_communication_error_on_the_interrupted_sentinel():
    motor = Motor(_StubParent("communication interrupted"))
    with pytest.raises(CommunicationError):
        motor.isBusy(0)


def test_is_busy_raises_communication_error_on_a_malformed_response():
    motor = Motor(_StubParent({"unexpected": "shape"}))
    with pytest.raises(CommunicationError):
        motor.isBusy(0)


def test_is_busy_still_returns_true_when_any_stepper_reports_busy():
    response = {"motor": {"steppers": [{"isbusy": 1}, {"isbusy": 0}, {"isbusy": 0}, {"isbusy": 0}]}}
    motor = Motor(_StubParent(response))
    assert motor.isBusy(0) is True


def test_is_busy_still_returns_false_on_a_real_not_busy_response():
    response = {"motor": {"steppers": [{"isbusy": 0}, {"isbusy": 0}, {"isbusy": 0}, {"isbusy": 0}]}}
    motor = Motor(_StubParent(response))
    assert motor.isBusy(0) is False


def test_is_busy_reads_the_real_api_v2_firmwares_isRunning_field():
    """2026-09-16 live-hardware regression: real API v2 firmware
    (UC2Client's own "Using API version 2" debug line) returns each
    stepper's busy flag as "isRunning", not "isbusy" -- confirmed directly
    from a live /motor_get response captured off real hardware. The old
    hardcoded "isbusy" lookup raised a KeyError on every single call
    against this firmware, which the fail-open bug this file's other tests
    cover used to silently turn into "not busy" -- and which, once that bug
    was fixed to raise CommunicationError instead, made isBusy() raise on
    literally every call, permanently blocking every stage move."""
    response = {"motor": {"steppers": [
        {"stepperid": 0, "position": 8202, "isRunning": 1, "isStop": 0},
        {"stepperid": 1, "position": 86466, "isRunning": 0, "isStop": 0},
        {"stepperid": 2, "position": 224160, "isRunning": 0, "isStop": 0},
        {"stepperid": 3, "position": 8204, "isRunning": 0, "isStop": 0},
    ]}, "qid": 28}
    motor = Motor(_StubParent(response))
    assert motor.isBusy(0) is True


def test_is_busy_returns_false_when_isRunning_reports_every_stepper_idle():
    response = {"motor": {"steppers": [
        {"stepperid": i, "position": 0, "isRunning": 0, "isStop": 0} for i in range(4)
    ]}}
    motor = Motor(_StubParent(response))
    assert motor.isBusy(0) is False
