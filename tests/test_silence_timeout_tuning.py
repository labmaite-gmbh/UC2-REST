"""Pins the relationship between mserial.Serial._SILENCE_TIMEOUT_S and
Motor.isBusy()'s default per-call timeout.

_SILENCE_TIMEOUT_S must stay strictly below the tightest per-call timeout any
caller uses in a hot loop -- currently Motor.isBusy()'s default of 1s, polled
continuously by ESP32Axis.wait_for_move_complete(). If it ever crept up to or
above that value again, a caller would give up and post_json()'s single retry
would re-queue a second command before the dispatch gate had been freed for
the first one -- racing a gate that isn't open yet, usually losing, and
falling through to a full reconnect() instead of the plain silence recovery
_SILENCE_TIMEOUT_S exists to provide (see test_serial_silence_timeout_recovery.py).
"""
import inspect

from uc2rest.mserial import Serial
from uc2rest.motor import Motor


def test_silence_timeout_is_below_motor_isbusy_poll_timeout():
    isbusy_default_timeout = inspect.signature(Motor.isBusy).parameters["timeout"].default
    assert Serial._SILENCE_TIMEOUT_S < isbusy_default_timeout, (
        f"_SILENCE_TIMEOUT_S ({Serial._SILENCE_TIMEOUT_S}s) must stay below "
        f"Motor.isBusy()'s default timeout ({isbusy_default_timeout}s), or a "
        f"caller's own give-up races the background dispatch gate instead of "
        f"waiting for it to clear cleanly."
    )
