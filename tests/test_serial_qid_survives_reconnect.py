"""Regression test: openDevice() must not reset identifier_counter.

Traced from the 2026-09-21 doublechip-perfusion run: after a reconnect the
counter restarted at 0, and a late acknowledgement for the *old* qid 6 was
handed to the new get_position() that had just been given qid 6 --
"get_position(): no valid response from ESP32 ({'qid': 6, 'success': 1})".
"""
import queue
import threading

from uc2rest.mserial import Serial


class _RecordingLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)

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


def test_open_device_keeps_the_qid_counter(monkeypatch):
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    s = _bare_serial(_FakeParent())
    s.identifier_counter = 41  # 41 commands already sent on this connection

    def _ok(port):
        s.serialdevice = _OpenableSer()
        return True

    s.tryToConnect = _ok
    try:
        s.openDevice(port="COM4", baud_rate=115200)
        assert s.identifier_counter == 41
        assert s._generate_identifier() == 42
    finally:
        _cleanup(s)


def test_reconnect_issues_strictly_increasing_qids(monkeypatch):
    monkeypatch.setattr(Serial, "_REOPEN_RETRY_DELAY_S", 0.001)
    s = _bare_serial(_FakeParent())
    s.ser = _OpenableSer()

    def _ok(port):
        s.serialdevice = _OpenableSer()
        return True

    s.tryToConnect = _ok
    try:
        before = s._generate_identifier()
        s.reconnect()
        after = s._generate_identifier()
        assert after == before + 1
    finally:
        _cleanup(s)
