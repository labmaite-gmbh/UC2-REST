"""reconnect() must never leave the configured port.

2026-09-20 console capture: reopening COM4 hit PermissionError (handle not
yet released), openDevice() fell through to findCorrectSerialDevice(),
which then tried -- and attempted to hard-reset -- COM3 and COM8, ports
belonging to other instruments. A reconnect of a known port must retry
that port and, if it cannot reopen it, fail loudly; it must not go looking
elsewhere. Port scanning remains available for the FIRST connection only
(configured port unknown / "NotConnected").
"""
import queue
import threading
from types import SimpleNamespace

import pytest
import serial.tools.list_ports

import uc2rest.mserial as mserial
from uc2rest.mserial import CommunicationError, Serial


class _RecordingLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(str(message))

    def warning(self, message):
        pass

    def debug(self, message):
        pass


class _FakeParent:
    def __init__(self):
        self.logger = _RecordingLogger()


class _OpenableSer:
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
    s.ser = _OpenableSer()
    s.serialport = "COM4"
    s.configured_port = "COM4"
    s.baudrate = 115200
    s.timeout = 5
    s._parent = parent
    s.identity = "UC2_Feather"
    s.DEBUG = False
    s.is_connected = True
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


def test_reconnect_of_a_known_port_does_not_port_scan(monkeypatch):
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    monkeypatch.setattr(Serial, "_REOPEN_BUDGET_S", 0.05)   # wall-clock window
    s = _bare_serial(_FakeParent())
    s.tryToConnect = lambda port: False  # COM4 stays busy for the whole retry window
    s.findCorrectSerialDevice = lambda: pytest.fail("reconnect() must not scan other ports")

    try:
        with pytest.raises(CommunicationError):
            s.reconnect()
    finally:
        _cleanup(s)


def test_first_open_may_still_port_scan(monkeypatch):
    """The startup path (port unknown) keeps the scan: that is the only
    legitimate reason to look at other ports."""
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    s = _bare_serial(_FakeParent())
    s.serialport = "NotConnected"
    s.tryToConnect = lambda port: False
    scanned = {"n": 0}

    def _scan():
        scanned["n"] += 1
        return None

    s.findCorrectSerialDevice = _scan
    try:
        s.openDevice(port=None, baud_rate=115200)  # default allow_port_scan=True
        assert scanned["n"] == 1
    finally:
        _cleanup(s)


def test_port_scan_only_hard_resets_the_configured_port(monkeypatch):
    s = _bare_serial(_FakeParent())
    s.configured_port = "COM4"
    ports = [
        SimpleNamespace(device="COM3", description="CH340 (COM3)"),
        SimpleNamespace(device="COM4", description="CH340 (COM4)"),
        SimpleNamespace(device="COM8", description="CP2102 (COM8)"),
    ]
    monkeypatch.setattr(serial.tools.list_ports, "comports", lambda include_links=False: ports)
    s.tryToConnect = lambda port: False
    resets = []

    def _fake_hard_reset(port):
        resets.append(port)
        return False

    s.hard_reset = _fake_hard_reset

    assert s.findCorrectSerialDevice() is None
    assert resets == ["COM4"], f"hard_reset touched foreign ports: {resets}"


def test_try_to_connect_closes_the_handle_when_the_firmware_check_fails(monkeypatch):
    closed = []

    class _FakePySerial:
        def __init__(self, port=None, **kw):
            self.port = None
        def open(self):
            pass
        def close(self):
            closed.append(True)
        def readline(self):
            return b""
        def write(self, data):
            pass

    monkeypatch.setattr(mserial.serial, "Serial", _FakePySerial)
    monkeypatch.setattr(mserial, "T_SERIAL_WARMUP", 0.0)
    s = _bare_serial(_FakeParent())
    s.checkFirmware = lambda ser: False

    assert s.tryToConnect("COM4") is False
    assert closed == [True]
    assert s.serialdevice is None


def test_port_scan_records_the_port_it_actually_found(monkeypatch):
    """A startup scan that finds the board on a DIFFERENT port than the
    configured one must record that port. Otherwise self.serialport keeps
    the stale configured value, reconnect()'s scan_ok check
    (self.serialport in (None, "NotConnected")) stays False, and every
    later reconnect retries the wrong port _REOPEN_ATTEMPTS times before
    falling through to a MockSerial and a CommunicationError -- forever,
    since nothing ever updates the port either.
    """
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    s = _bare_serial(_FakeParent())
    s.serialport = "COM4"
    s.configured_port = "COM4"
    ports = [
        SimpleNamespace(device="COM4", description="CH340 (COM4)"),
        SimpleNamespace(device="COM7", description="CH340 (COM7)"),
    ]
    monkeypatch.setattr(serial.tools.list_ports, "comports", lambda include_links=False: ports)

    def _only_com7(port):
        if port == "COM7":
            s.serialdevice = _OpenableSer()
            return True
        return False

    s.tryToConnect = _only_com7
    s.hard_reset = lambda port: False

    assert isinstance(s.findCorrectSerialDevice(), _OpenableSer)
    assert s.serialport == "COM7"
    assert s.configured_port == "COM7"

    # ... and a reconnect after that scan must go back to COM7, not COM4.
    asked = []

    def _recording(port):
        asked.append(port)
        s.serialdevice = _OpenableSer()
        return True

    s.tryToConnect = _recording
    s.findCorrectSerialDevice = lambda: pytest.fail("reconnect() must not re-scan")
    try:
        s.reconnect()
        assert asked == ["COM7"], f"reconnect() retried the wrong port(s): {asked}"
    finally:
        _cleanup(s)
