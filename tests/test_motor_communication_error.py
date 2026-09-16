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
