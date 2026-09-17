"""Regression tests for Motor.get_position()'s fail-open bug.

Same class of bug as isBusy() (see test_motor_communication_error.py), never
fixed here: get_position() only ever populated the returned position array
from inside `if "motor" in r:`, with no else branch -- so a broken/timed-out
serial exchange (post_json() returns the string "communication interrupted",
or None if the reader thread has already stopped, see mserial.py) silently
fell through to the array's untouched initial value, (0., 0., 0., 0.), and
was returned as if it were the stage's real, confirmed position. No exception,
no warning.

Traced from a 2026-09-17 live run where the stage crashed into the frame edges
after the serial link stalled: colision_avoidance.py's _move() calls
position() to decide whether the target is in the same slot as "here" (direct
move) or a different one (home Z, move XY, move Z) -- entirely trusting
whatever position() returns. A silent (0,0,0,0) during a comms stall is
exactly the kind of confidently-wrong input that routes a move through the
wrong branch with no error anywhere to catch.

get_position() must now raise CommunicationError instead, matching isBusy(),
so callers can tell "don't know" from "confirmed at (0,0,0,0)" -- and so
ESP32Axis.position() (locai-impl) has something to catch and fall back to a
cached last-known-good position for, rather than trusting a fabricated one.
"""
import numpy as np
import pytest

from uc2rest.mserial import CommunicationError
from uc2rest.motor import Motor


class _StubParent:
    def __init__(self, response):
        self._response = response

    def post_json(self, path, payload, getReturn=True, nResponses=1, timeout=1):
        return self._response


def test_get_position_raises_communication_error_on_the_interrupted_sentinel():
    motor = Motor(_StubParent("communication interrupted"))
    with pytest.raises(CommunicationError):
        motor.get_position()


def test_get_position_raises_communication_error_on_none():
    """post_json()/sendMessage() can return None outright (e.g. the reader
    thread already stopped when the call was made) -- distinct from the
    "communication interrupted" sentinel string, and previously not handled
    either: `"motor" in None` itself raises TypeError, uncaught."""
    motor = Motor(_StubParent(None))
    with pytest.raises(CommunicationError):
        motor.get_position()


def test_get_position_raises_communication_error_on_a_malformed_response():
    motor = Motor(_StubParent({"unexpected": "shape"}))
    with pytest.raises(CommunicationError):
        motor.get_position()


def test_get_position_still_returns_the_real_position_on_a_good_response():
    response = {"motor": {"steppers": [
        {"stepperid": 0, "position": 1000},
        {"stepperid": 1, "position": 2000},
        {"stepperid": 2, "position": 3000},
        {"stepperid": 3, "position": 4000},
    ]}}
    motor = Motor(_StubParent(response))
    pos = motor.get_position()
    # motorAxisOrder default [0,1,2,3] and default step sizes are all 1, so
    # position[stepperid] should come straight through.
    assert np.array_equal(pos, np.array([1000., 2000., 3000., 4000.]))
