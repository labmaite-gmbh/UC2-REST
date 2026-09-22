"""Regression tests for a 2026-09-16 live-hardware incident: reconnect()
degraded a working connection to a silent dummy MockSerial backend.

lm_hardware's call_with_retry() (added this session) calls reconnect() to
recover from a CommunicationError. reconnect() closes the current serial
connection and immediately calls openDevice() to reopen the *same* port --
but the OS can take a moment to actually release a just-closed COM port
handle, so the reopen attempt hit a transient PermissionError. openDevice()
had no retry for that specific case: on any failure it falls straight
through to findCorrectSerialDevice() (scanning for a different port, which
found nothing) and then silently constructs a MockSerial "dummy" backend --
with no exception raised anywhere. Every subsequent call then silently
talked to nothing, real hardware access lost for the rest of the process.

Two fixes: openDevice() now retries the reopen a few times with a short
backoff before giving up (fixes the actual race), and reconnect() now
raises if it ends up on a disconnected/dummy backend anyway (so a caller
like call_with_retry finds out the reconnect failed instead of silently
retrying against a fake connection that can never respond).
"""
import queue
import threading

import pytest

from uc2rest.mserial import CommunicationError, Serial


class _RecordingLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)

    def debug(self, message):
        pass


class _FakeParent:
    def __init__(self):
        self.logger = _RecordingLogger()


class _OpenableSer:
    """A fake serial handle good enough for openDevice()'s post-connect
    bookkeeping (write_timeout assignment, isOpen()/open(), the
    readline()-until-empty buffer flush)."""
    BAUDRATES = (110, 300)

    def __init__(self):
        self.write_timeout = None
        self._open = True

    def isOpen(self):
        return self._open

    def open(self):
        self._open = True

    def close(self):
        self._open = False

    def readline(self):
        return b""


def _bare_serial(parent) -> Serial:
    s = Serial.__new__(Serial)
    s.ser = None
    s.serialport = "COM4"
    s.baudrate = 115200
    s.timeout = 5
    s._parent = parent
    s.identity = "UC2_Feather"
    s.DEBUG = False
    s.is_connected = False
    s.write_timeout = 0.02
    s.read_timeout = 0.02
    s.thread = None
    s.running = False
    s.cmdCallBackFct = None
    s.resetLastCommand = False
    s.command_queue = queue.Queue()
    s.responses = {}
    s.commands = {}
    s.lock = threading.Lock()
    s.callBackList = []
    s.identifier_counter = 0
    return s


def _cleanup(s):
    s.running = False
    if s.thread is not None:
        s.thread.join(timeout=1)


def test_open_device_retries_before_falling_back_to_a_dummy_connection(monkeypatch):
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    parent = _FakeParent()
    s = _bare_serial(parent)
    attempts = []

    def _fake_try_to_connect(port):
        attempts.append(port)
        if len(attempts) < 3:
            return False
        s.serialdevice = _OpenableSer()
        return True

    s.tryToConnect = _fake_try_to_connect
    s.findCorrectSerialDevice = lambda: pytest.fail("must not fall back to port-scanning/dummy")

    try:
        ser = s.openDevice(port="COM4", baud_rate=115200)
        assert len(attempts) == 3
        assert s.is_connected is True
        assert isinstance(ser, _OpenableSer)
    finally:
        _cleanup(s)


def test_open_device_still_falls_back_to_a_dummy_after_genuinely_exhausting_retries(monkeypatch):
    """A port that's actually gone (not just transiently busy) must still
    end up on the documented dummy/no-device fallback, not retry forever."""
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    parent = _FakeParent()
    s = _bare_serial(parent)
    s.tryToConnect = lambda port: False
    s.findCorrectSerialDevice = lambda: None  # no other port found either

    try:
        ser = s.openDevice(port="COM4", baud_rate=115200)
        # is_connected is racy against _process_commands()'s own thread
        # (see reconnect()'s comment) -- the returned object's type is the
        # unambiguous signal that the dummy fallback was actually used.
        assert type(ser).__name__ == "MockSerial"
    finally:
        _cleanup(s)


def test_reconnect_raises_if_it_falls_back_to_a_dummy_connection(monkeypatch):
    """reconnect() silently succeeding while actually holding a disconnected
    dummy serial backend is worse than raising -- a caller like
    esp32_conn.py's call_with_retry must be told the reconnect failed, not
    proceed to retry against a fake connection that will never respond."""
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    parent = _FakeParent()
    s = _bare_serial(parent)
    s.tryToConnect = lambda port: False
    s.findCorrectSerialDevice = lambda: None

    try:
        with pytest.raises(CommunicationError):
            s.reconnect()
    finally:
        _cleanup(s)


def test_open_device_skips_the_retry_loop_for_the_notconnected_sentinel(monkeypatch):
    """findCorrectSerialDevice() sets self.serialport = "NotConnected" when
    no real port was ever found, and reconnect() always passes
    self.serialport straight back into openDevice() as `port`. Retrying
    tryToConnect("NotConnected") -- a literal, un-openable string, not a
    real device -- just wastes _REOPEN_ATTEMPTS x _REOPEN_RETRY_DELAY_S on
    every single reconnect before it ever reaches the real port-scan.
    Traced from a 2026-09-18 live incident where a reconnect loop spent
    15+ minutes cycling through "could not open port 'NotConnected'"
    without ever getting to look at the actual available ports."""
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    parent = _FakeParent()
    s = _bare_serial(parent)
    s.tryToConnect = lambda port: pytest.fail(
        "must not retry the literal 'NotConnected' sentinel -- go straight "
        "to findCorrectSerialDevice()")

    def _fake_find():
        s.serialdevice = _OpenableSer()
        s.is_connected = True
        return s.serialdevice

    s.findCorrectSerialDevice = _fake_find

    try:
        ser = s.openDevice(port="NotConnected", baud_rate=115200)
        assert isinstance(ser, _OpenableSer)
    finally:
        _cleanup(s)


def test_open_device_also_skips_the_retry_loop_when_no_port_is_known_yet(monkeypatch):
    """port=None (the very first connection attempt, before any port has
    ever been recorded) is the same case as the sentinel -- nothing to
    retry against yet."""
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    parent = _FakeParent()
    s = _bare_serial(parent)
    s.tryToConnect = lambda port: pytest.fail("must not retry with no known port")

    def _fake_find():
        s.serialdevice = _OpenableSer()
        s.is_connected = True
        return s.serialdevice

    s.findCorrectSerialDevice = _fake_find

    try:
        ser = s.openDevice(port=None, baud_rate=115200)
        assert isinstance(ser, _OpenableSer)
    finally:
        _cleanup(s)


def test_reconnect_does_not_raise_when_it_genuinely_recovers(monkeypatch):
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    parent = _FakeParent()
    s = _bare_serial(parent)

    def _fake_try_to_connect(port):
        s.serialdevice = _OpenableSer()
        return True

    s.tryToConnect = _fake_try_to_connect

    try:
        s.reconnect()  # must not raise
        assert s.is_connected is True
    finally:
        _cleanup(s)
