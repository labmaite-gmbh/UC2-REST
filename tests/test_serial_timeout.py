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


class _SlowThenEchoingSer:
    """The first command's response is delayed past the caller's timeout,
    but it does eventually arrive -- unlike a true drop, this does not wedge
    _process_commands()'s dispatch loop (which only advances to the next
    queued command once the current one's response is matched, so a
    genuinely dropped response would starve every command after it too).
    This matches what channel_5/cycle12 actually looked like: slow, not
    dropped. Every command after the first is answered immediately."""
    BAUDRATES = (110, 300)

    def __init__(self, delay_first_by=0.5):
        self._delay_first_by = delay_first_by
        self._first_write_seen = False
        self._pending = []  # [(ready_at, [lines])]
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
        if qid is None:
            return
        lines = ["++", json.dumps({"qid": qid, "isbusy": 0}), "--"]
        if not self._first_write_seen:
            self._first_write_seen = True
            self._pending.append((time.time() + self._delay_first_by, lines))
        else:
            self._lines += lines

    def readline(self):
        now = time.time()
        still_pending = []
        for ready_at, lines in self._pending:
            if now >= ready_at:
                self._lines.extend(lines)
            else:
                still_pending.append((ready_at, lines))
        self._pending = still_pending
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
    reconstructable after the fact from gaps between unrelated log lines.
    A silent server times out twice now (the original attempt plus the
    single retry), so both must be logged, not just the first."""
    parent = _FakeParent()
    s = _bare_serial(_SilentSer(), parent)

    s.post_json("/motor_get", {"isbusy": 1}, timeout=0.2)

    assert len(parent.logger.errors) == 2
    for msg in parent.logger.errors:
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
    assert not parent.logger.warnings


def test_post_json_retries_once_after_a_communication_interrupt():
    """A response that arrives too late for the caller's timeout -- but does
    arrive -- unblocks the dispatch queue, so a retry with a fresh qid goes
    through immediately. Cheap insurance against a transient slow-down (see
    esp32_conn.py's get_object() comment on the qid-framed protocol) that
    has already cleared up by the time the retry goes out."""
    parent = _FakeParent()
    # The delayed response must land after the first attempt's own timeout
    # (0.2s) gives up, but before the retry's own timeout (another 0.2s,
    # starting once the first gives up) also elapses.
    ser = _SlowThenEchoingSer(delay_first_by=0.3)
    s = _bare_serial(ser, parent)
    s.thread = threading.Thread(target=s._process_commands, daemon=True)
    s.thread.start()
    try:
        result = s.post_json("/motor_get", {"isbusy": 1}, timeout=0.2)
    finally:
        s.running = False
        s.thread.join(timeout=1)

    assert isinstance(result, dict)
    assert result.get("isbusy") == 0
    assert len(parent.logger.warnings) == 1
    assert "retry" in parent.logger.warnings[0].lower()
    assert "/motor_get" in parent.logger.warnings[0]


def test_post_json_gives_up_after_a_single_retry():
    """The retry is bounded: a command that never gets a response still
    fails after exactly one retry, not an unbounded loop."""
    parent = _FakeParent()
    s = _bare_serial(_SilentSer(), parent)

    t0 = time.time()
    result = s.post_json("/motor_get", {"isbusy": 1}, timeout=0.2)
    elapsed = time.time() - t0

    assert result == "communication interrupted"
    # two attempts at ~0.2s each, not more
    assert elapsed < 1.0, f"retry was not bounded: took {elapsed:.1f}s"
    assert len(parent.logger.warnings) == 1
    assert len(parent.logger.errors) == 2  # one timeout log per attempt


def test_reconnect_stops_the_old_reader_thread_before_reopening():
    """reconnect() must fully stop the existing background reader before
    opening a new connection and starting a new one -- otherwise two
    threads can briefly read the same port at once, which is exactly the
    kind of interleaving that desyncs the qid-framed protocol (see
    esp32_conn.py's get_object() comment). This is a regression test for
    a real gap: the old reconnect() closed the port and reopened it
    without ever joining the thread reading the port it just closed."""
    parent = _FakeParent()
    s = _bare_serial(_EchoingSer(), parent)
    s.thread = threading.Thread(target=s._process_commands, daemon=True)
    s.thread.start()
    old_thread = s.thread

    calls = []

    def _fake_open_device(port=None, baud_rate=115200, allow_port_scan=True):
        calls.append((port, baud_rate))
        # openDevice() always leaves a fresh reader thread running on the
        # new connection -- reproduce that much of its contract here.
        s.running = True
        new_thread = threading.Thread(target=s._process_commands, daemon=True)
        new_thread.start()
        s.thread = new_thread
        return _EchoingSer()

    s.openDevice = _fake_open_device

    s.reconnect()

    assert not old_thread.is_alive(), "old reader thread must be stopped before reconnecting"
    assert calls == [("MOCK", 115200)]

    s.running = False
    s.thread.join(timeout=1)
