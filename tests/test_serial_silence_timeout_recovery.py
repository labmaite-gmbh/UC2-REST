"""Regression test for _process_commands() permanently wedging the command
queue when a command gets ZERO bytes back -- not even a garbled frame.

Distinct from test_serial_garbled_response_recovery.py: that bug's fix marks
a command done in qeueIdSuccess once a "--"-terminated frame arrives, garbled
or not. But total silence never produces a "--" line at all, so
`reading_json` never flips True and that fix's branch is never reached --
this is a second, separate way the dispatch gate could freeze forever, only
recoverable via a full reconnect().

The fix bounds how long _process_commands() will wait for ANY response to
the command it just sent (_SILENCE_TIMEOUT_S) before marking it done anyway
(with `{}`, same shape as a garbled response) and letting the dispatch loop
move on to the next queued command.
"""
import json
import queue
import threading

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


def _bare_serial(ser, parent) -> Serial:
    """A Serial instance with attributes set directly, skipping openDevice()
    (which requires a real port) and the background thread (started by the
    test itself, when it needs the request/response loop running)."""
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


class _SilentThenEchoingSer:
    """The first command gets zero bytes back, ever -- readline() always
    returns b"" for it. Every command after that gets a normal, valid
    echoed response. Reproduces "the link went briefly silent, then came
    back" -- not a dead connection -- which is exactly the case the old
    code could never recover from on its own."""
    BAUDRATES = (110, 300)

    def __init__(self):
        self._lines = []
        self._seen_first = False

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
        if not self._seen_first:
            self._seen_first = True
            # No response at all for this one -- readline() just keeps
            # returning b"" (see below).
        else:
            self._lines += ["++", json.dumps({"qid": qid, "isbusy": 0}), "--"]

    def readline(self):
        if self._lines:
            return (self._lines.pop(0) + "\n").encode()
        return b""


def test_total_silence_does_not_freeze_later_commands():
    parent = _FakeParent()
    s = _bare_serial(_SilentThenEchoingSer(), parent)
    s._SILENCE_TIMEOUT_S = 0.2  # keep the test fast
    s.thread = threading.Thread(target=s._process_commands, daemon=True)
    s.thread.start()
    try:
        # First command: never gets a single byte back. Before the fix this
        # left the dispatch queue permanently closed -- only a full
        # reconnect() would ever clear it. After the fix, _process_commands
        # gives up on it once _SILENCE_TIMEOUT_S has passed and marks it
        # done with an empty dict, same shape as a garbled response.
        first = s.post_json("/motor_get", {"isbusy": 1}, timeout=1.0)
        assert first == {}

        # Second command, sent after the first gave up: must get its own,
        # real response -- not another timeout. Before the fix this also
        # timed out (and every command after it, forever).
        second = s.post_json("/motor_get", {"isbusy": 1}, timeout=1.0)
        assert isinstance(second, dict), (
            f"expected a real response, got {second!r} -- the queue is "
            f"still stuck after the first command's total silence"
        )
        assert second.get("isbusy") == 0
    finally:
        s.running = False
        s.thread.join(timeout=1)
