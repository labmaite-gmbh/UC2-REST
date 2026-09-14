"""Regression tests for the serial-timeout plumbing fix.

Serial.post_json() / UC2Client.post_json() / UC2Client.get_json() accepted a
`timeout` argument but never forwarded it to sendMessage(), which always
waited out its own hard-coded 20s default no matter what a caller (e.g.
motor.isBusy(timeout=1), polled every 1ms by wait_for_move_complete()) asked
for. That turned one dropped/garbled serial response into a stall dozens of
times longer than intended. sendMessage() also gave up silently, with no
record of which command it was waiting on.

These tests build a Serial instance directly (bypassing openDevice(), which
needs a real port) so they can run without hardware, matching how the rest
of this module already tests via MockSerial.
"""
import json
import queue
import threading
import time

from uc2rest.mserial import Serial


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


class _SilentSer:
    """Looks enough like pyserial's Serial to pass sendMessage's guard, but
    never answers -- every command is dropped, forcing every call to wait
    out its full timeout. BAUDRATES must be a tuple or sendMessage() takes
    its 'not connected' early-return instead of actually waiting."""
    BAUDRATES = (110, 300)


class _EchoingSer:
    """Answers every written command with a canned ++/qid/---framed response,
    mirroring the qid back -- enough for _process_commands to resolve a
    real sendMessage() call end to end."""
    BAUDRATES = (110, 300)

    def __init__(self):
        self._lines = []

    def write(self, data: bytes):
        text = data.decode().strip()
        if not text:
            return
        try:
            cmd = json.loads(text)
        except ValueError:
            return
        qid = cmd.get("qid")
        if qid is not None:
            self._lines += ["++", json.dumps({"qid": qid, "isbusy": 0}), "--"]

    def readline(self):
        if self._lines:
            return (self._lines.pop(0) + "\n").encode()
        return b""


def _bare_serial(ser, parent) -> Serial:
    """A Serial instance with attributes set directly, skipping openDevice()
    (which requires a real port) and the background thread (each test starts
    it itself only if it needs the request/response loop running)."""
    s = Serial.__new__(Serial)
    s.ser = ser
    s.serialport = "MOCK"
    s.baudrate = 115200
    s.timeout = 5
    s._parent = parent
    s.identity = "UC2_Feather"
    s.DEBUG = False
    s.is_connected = True
    s.write_timeout = 0.02
    s.read_timeout = 0.02
    s.cmdCallBackFct = None
    s.resetLastCommand = False
    s.command_queue = queue.Queue()
    s.responses = {}
    s.commands = {}
    s.lock = threading.Lock()
    s.callBackList = []
    s.running = True
    s.identifier_counter = 0
    return s


def test_post_json_honors_a_short_caller_timeout():
    """A 0.3s caller timeout must return in ~0.3s, not the 20s default --
    this is what let wait_for_move_complete()'s 1s-timeout busy-poll block
    up to 20x longer than intended whenever a response went missing."""
    parent = _FakeParent()
    s = _bare_serial(_SilentSer(), parent)

    t0 = time.time()
    result = s.post_json("/motor_get", {"isbusy": 1}, timeout=0.3)
    elapsed = time.time() - t0

    assert result == "communication interrupted"
    assert elapsed < 2.0, f"timeout was not honored: took {elapsed:.1f}s for a 0.3s timeout"
    assert elapsed > 0.2, f"returned suspiciously fast ({elapsed:.2f}s) for a 0.3s timeout"


def test_timeout_is_logged_with_qid_and_task():
    """A timed-out command must leave a trace -- previously it was only
    reconstructable after the fact from gaps between unrelated log lines."""
    parent = _FakeParent()
    s = _bare_serial(_SilentSer(), parent)

    s.post_json("/motor_get", {"isbusy": 1}, timeout=0.2)

    assert len(parent.logger.errors) == 1
    msg = parent.logger.errors[0]
    assert "timed out" in msg
    assert "/motor_get" in msg
    assert "qid=" in msg


def test_successful_round_trip_still_works():
    """The happy path (a command that gets a real, matching response) must
    be unaffected by the timeout/lock changes."""
    parent = _FakeParent()
    s = _bare_serial(_EchoingSer(), parent)
    s.thread = threading.Thread(target=s._process_commands, daemon=True)
    s.thread.start()
    try:
        result = s.post_json("/motor_get", {"isbusy": 1}, timeout=2)
    finally:
        s.running = False
        s.thread.join(timeout=1)

    assert isinstance(result, dict)
    assert result.get("isbusy") == 0
    assert not parent.logger.errors
