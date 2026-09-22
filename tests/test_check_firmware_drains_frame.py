"""Regression test: checkFirmware() must consume the ENTIRE /state_get reply.

2026-09-21 22:52:56: right after a reconnect, isBusy() received
{'state': {'identifier_name': 'UC2_Feather', ...}} instead of a motor
reply. checkFirmware() had returned at "++" and _freeSerialBuffer() gave
up at the first 20 ms gap, so the frame body and its "--" were read by the
new _process_commands() thread and attributed to the next real command.
"""
import queue
import threading

from uc2rest.mserial import Serial


class _RecordingLogger:
    def error(self, message):
        pass

    def warning(self, message):
        pass

    def debug(self, message):
        pass


class _FakeParent:
    def __init__(self):
        self.logger = _RecordingLogger()


class _ScriptedSer:
    """readline() replays `lines` in order (b"" = a 20 ms gap with no
    bytes), then returns b"" forever. `remaining()` shows what a later
    reader would still receive."""
    BAUDRATES = (110, 300)

    def __init__(self, lines):
        self._lines = list(lines)
        self.written = []

    def write(self, data):
        self.written.append(data)

    def readline(self):
        if self._lines:
            return self._lines.pop(0)
        return b""

    def remaining(self):
        return [l for l in self._lines if l.strip() != b""]


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


STATE_FRAME_WITH_GAPS = [
    b"++\n",
    b"",                                    # gap > read timeout
    b'{"state": {"identifier_name": "UC2_Feather",\n',
    b"",                                    # another gap mid-frame
    b'"identifier_id": "V2.0"}}\n',
    b"",
    b"--\n",
]


def test_check_firmware_consumes_the_full_state_frame():
    s = _bare_serial(_FakeParent())
    ser = _ScriptedSer(STATE_FRAME_WITH_GAPS)

    assert s.checkFirmware(ser) is True
    assert ser.remaining() == [], (
        f"frame body left in the buffer for the next reader: {ser.remaining()}")


def test_free_serial_buffer_needs_a_quiet_period_not_a_single_empty_read():
    s = _bare_serial(_FakeParent())
    ser = _ScriptedSer([b"", b'{"late": 1}\n', b"", b"--\n"])

    s._freeSerialBuffer(ser)

    assert ser.remaining() == []


def test_drain_frame_gives_up_after_its_timeout(monkeypatch):
    monkeypatch.setattr(Serial, "_FRAME_DRAIN_TIMEOUT_S", 0.05)
    s = _bare_serial(_FakeParent())
    ser = _ScriptedSer([b'{"never": "terminated"}\n'])  # no "--" ever

    assert s._drain_frame(ser) is False  # returns, does not hang
