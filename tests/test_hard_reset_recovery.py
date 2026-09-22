"""Regression tests for hard_reset() and its use inside
findCorrectSerialDevice().

Traced from a 2026-09-18 live rollout where the ESP32 link degraded
progressively over ~15 minutes until the firmware stopped answering the
handshake entirely on every reconnect attempt. Before this, the only
recovery path was reopening the serial port -- which does nothing if the
firmware itself is hung, since the port opens fine but the handshake still
gets no response. hard_reset() toggles the board's EN/reset pin via RTS
(the same mechanism esptool.py's HardReset uses for the classic
CH340/CP2102 auto-reset circuit findCorrectSerialDevice() already targets),
giving a hung-but-physically-connected board a real chance to come back
without needing anyone to touch the cable.

Once findCorrectSerialDevice() recovers a working connection this way, the
existing retry architecture (esp32_conn.py's call_with_retry(), which calls
reconnect() -> openDevice() -> findCorrectSerialDevice() between attempts)
already retries the original queued command on its next attempt -- no
separate "retry the command" logic needed here.
"""
import queue
import threading

import pytest

from uc2rest.mserial import Serial


class _RecordingLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def warning(self, message):
        self.warnings.append(message)

    def debug(self, message):
        pass


class _FakeParent:
    def __init__(self):
        self.logger = _RecordingLogger()


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


class _FakeRtsSerial:
    """Stands in for a raw serial.Serial(...) handle, just to record the
    RTS toggle sequence hard_reset() performs."""

    def __init__(self, port=None, baudrate=None):
        self.port = port
        self.baudrate = baudrate
        self.rts_history = []

    def setRTS(self, value):
        self.rts_history.append(value)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class _FakePortInfo:
    def __init__(self, device, description=""):
        self.device = device
        self.description = description


def test_hard_reset_toggles_rts_low_then_high_and_returns_true(monkeypatch):
    parent = _FakeParent()
    s = _bare_serial(parent)
    created = []

    def _fake_serial(port=None, baudrate=None):
        h = _FakeRtsSerial(port=port, baudrate=baudrate)
        created.append(h)
        return h

    monkeypatch.setattr("uc2rest.mserial.serial.Serial", _fake_serial)
    monkeypatch.setattr("uc2rest.mserial.time.sleep", lambda *_: None)

    assert s.hard_reset("COM7") is True
    # hard_reset() retries up to _REOPEN_ATTEMPTS times; all attempts that
    # succeed without exception still toggle RTS, so all created instances
    # should have the sequence. Verify at least one has it (typically all).
    assert len(created) > 0
    assert all(h.port == "COM7" for h in created)
    # EN held low (True passed to setRTS -- see hard_reset()'s own comment
    # on RTS polarity) then released (False) -- reset-then-run, in order.
    assert all(h.rts_history == [True, False] for h in created)


def test_hard_reset_returns_false_without_raising_when_the_port_cannot_be_opened(monkeypatch):
    parent = _FakeParent()
    s = _bare_serial(parent)

    def _boom(port=None, baudrate=None):
        raise OSError("port vanished")

    monkeypatch.setattr("uc2rest.mserial.serial.Serial", _boom)
    monkeypatch.setattr("uc2rest.mserial.Serial._REOPEN_RETRY_DELAY_S", 0.001)

    assert s.hard_reset("COM7") is False  # must not raise


def test_find_correct_serial_device_hard_resets_and_retries_before_giving_up(monkeypatch):
    """A matching port whose firmware handshake fails once must get a
    hard_reset() + one retried handshake before the scan gives up on it."""
    parent = _FakeParent()
    s = _bare_serial(parent)

    monkeypatch.setattr(
        "uc2rest.mserial.serial.tools.list_ports.comports",
        lambda include_links=False: [_FakePortInfo("COM7", "CH340")])

    attempts = []
    recovered_device = object()

    def _fake_try_to_connect(port):
        attempts.append(port)
        if len(attempts) == 1:
            return False  # first handshake: firmware hung, no response
        s.serialdevice = recovered_device
        return True       # second handshake, after the reset: recovered

    reset_calls = []
    s.tryToConnect = _fake_try_to_connect
    s.hard_reset = lambda port: reset_calls.append(port) or True

    result = s.findCorrectSerialDevice()

    assert reset_calls == ["COM7"]
    assert attempts == ["COM7", "COM7"]
    assert result is recovered_device
    assert s.is_connected is True
    assert s.serialport != "NotConnected"


def test_find_correct_serial_device_still_falls_through_when_reset_does_not_help(monkeypatch):
    """If the handshake fails even after a hard reset, the port is still a
    lost cause -- must not loop forever, must land on the same
    documented "NotConnected" fallback as before."""
    parent = _FakeParent()
    s = _bare_serial(parent)

    monkeypatch.setattr(
        "uc2rest.mserial.serial.tools.list_ports.comports",
        lambda include_links=False: [_FakePortInfo("COM7", "CH340")])

    s.tryToConnect = lambda port: False  # never recovers, reset or not
    s.hard_reset = lambda port: True

    result = s.findCorrectSerialDevice()

    assert result is None
    assert s.is_connected is False
    assert s.serialport == "NotConnected"


def test_find_correct_serial_device_does_not_hard_reset_when_no_port_matches(monkeypatch):
    """No candidate port at all (nothing matching the known descriptions)
    means there's nothing to reset -- must go straight to the
    "NotConnected" fallback, not call hard_reset with a bogus port."""
    parent = _FakeParent()
    s = _bare_serial(parent)

    monkeypatch.setattr(
        "uc2rest.mserial.serial.tools.list_ports.comports",
        lambda include_links=False: [_FakePortInfo("LPT1", "Some Other Chip")])

    s.tryToConnect = lambda port: pytest.fail("must not even try a non-matching port")
    s.hard_reset = lambda port: pytest.fail("must not reset a non-matching port")

    result = s.findCorrectSerialDevice()

    assert result is None
    assert s.serialport == "NotConnected"
