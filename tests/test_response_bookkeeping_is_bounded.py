"""self.responses / self.commands must not grow for the life of the process.

qids deliberately never restart (see identifier_counter: a reconnect must
not let a late ack for an old qid land on a brand-new command reusing that
number). The flip side is that every exchange used to leave an entry in
both dicts forever -- hundreds of thousands of them over a 24 h run, plus
one responses[-1] list that collected every unsolicited firmware broadcast
and was never read by anyone.

Three bounds: sendMessage() drops its own entries as it returns, the
"--" branch drops qid=-1 frames once the callbacks have seen them, and a
new qid prunes anything older than _RESPONSE_HISTORY.
"""
import json
import queue
import threading
import time

from uc2rest.mserial import Serial


class _RecordingLogger:
    def __init__(self):
        self.debugs = []

    def error(self, message):
        pass

    def warning(self, message):
        pass

    def debug(self, message):
        self.debugs.append(message)


class _FakeParent:
    def __init__(self):
        self.logger = _RecordingLogger()


def _bare_serial(ser, parent) -> Serial:
    s = Serial.__new__(Serial)
    s.ser = ser
    s.serialport = "MOCK"
    s.configured_port = "MOCK"
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
    s.thread = None
    s.identifier_counter = 0
    return s


class _EchoingSer:
    """Answers every command with a well-formed framed reply carrying the
    same qid."""
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
        if qid is None:
            return
        self._lines += ["++", json.dumps({"qid": qid, "isbusy": 0}), "--"]

    def readline(self):
        if self._lines:
            return (self._lines.pop(0) + "\n").encode()
        return b""


class _UnsolicitedSer:
    """Emits one unsolicited broadcast frame (qid=-1) and then nothing --
    the shape the firmware uses for async notifications, which arrive with
    no command outstanding at all."""
    BAUDRATES = (110, 300)

    def __init__(self):
        self._lines = ["++", json.dumps({"qid": -1, "steppers": []}), "--"]

    def write(self, data: bytes):
        pass

    def readline(self):
        if self._lines:
            return (self._lines.pop(0) + "\n").encode()
        return b""


def test_answered_exchanges_do_not_accumulate_in_responses():
    parent = _FakeParent()
    s = _bare_serial(_EchoingSer(), parent)
    s.thread = threading.Thread(target=s._process_commands, daemon=True)
    s.thread.start()
    try:
        answered = []
        for _ in range(40):
            r = s.sendMessage({"task": "/motor_get", "isbusy": 1}, nResponses=1, timeout=2)
            assert isinstance(r, dict) and r.get("isbusy") == 0
            answered.append(r["qid"])

        assert len(s.responses) <= s._RESPONSE_HISTORY, (
            f"responses grew to {len(s.responses)} entries")
        leaked = [q for q in answered if q in s.responses]
        assert leaked == [], f"answered qids still held in responses: {leaked}"
        leaked_cmds = [q for q in answered if q in s.commands]
        assert leaked_cmds == [], f"answered qids still held in commands: {leaked_cmds}"
    finally:
        s.running = False
        s.thread.join(timeout=1)


def test_unsolicited_frame_reaches_callbacks_but_is_not_stored():
    parent = _FakeParent()
    s = _bare_serial(_UnsolicitedSer(), parent)
    seen = []
    s.register_callback(lambda payload: seen.append(payload), "steppers")
    s.thread = threading.Thread(target=s._process_commands, daemon=True)
    s.thread.start()
    try:
        t0 = time.time()
        while not seen and time.time() - t0 < 2:
            time.sleep(0.01)
        assert seen and seen[0]["steppers"] == [], f"callback never ran: {seen!r}"
        assert -1 not in s.responses, (
            f"unsolicited frames are piling up in responses[-1]: {s.responses.get(-1)!r}")
    finally:
        s.running = False
        s.thread.join(timeout=1)


def test_a_new_qid_prunes_bookkeeping_older_than_the_history_window():
    """The paths that never collect a reply (fire-and-forget sends, callers
    that timed out) still leave entries behind; a new qid must drop the
    ones no caller can possibly be waiting for any more."""
    parent = _FakeParent()
    s = _bare_serial(_EchoingSer(), parent)
    s.identifier_counter = 5000
    for qid in range(4000, 5001):
        s.responses[qid] = [{}]
        s.commands[qid] = {"qid": qid}

    new_id = s._generate_identifier()

    assert new_id == 5001
    cutoff = new_id - s._RESPONSE_HISTORY
    assert all(q >= cutoff for q in s.responses), (
        f"stale qids survived the prune: {sorted(q for q in s.responses if q < cutoff)[:5]}")
    assert all(q >= cutoff for q in s.commands)
    assert 5000 in s.responses and 5000 in s.commands, "in-flight qids must survive"
