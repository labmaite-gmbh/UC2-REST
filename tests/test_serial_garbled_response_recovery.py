"""Regression test for _process_commands() permanently wedging the command
queue after a single garbled/unparseable serial response.

Traced from a 2026-09-17 live run (ERROR_colission.log): once one response
failed `json.loads` (logged as "Failed to load the json from serial"), every
subsequent command -- moves, position reads, everything -- timed out for the
rest of the ~18-minute log, well past the point where reconnect() should have
recovered things. The mechanism:

  1. A command is dequeued and sent: `currentIdentifier, command =
     self.command_queue.get()` sets currentIdentifier to *that* command's qid
     right away, before any response has arrived.
  2. If that response fails to parse, the handler falls into the bare
     `except: json_response = {}` branch. `qeueIdSuccess[qid] = 1` (the line
     that marks the command done) sits in the try block that just failed, so
     it never runs for this qid.
  3. `currentIdentifier = json_response["qid"]` also raises (`{}` has no
     "qid") and is swallowed by its own `except: pass` -- so currentIdentifier
     is left pointing at the same qid from step 1, not reset.
  4. The main loop's dequeue gate (`lastTransmisionSuccess = qeueIdSuccess[
     str(currentIdentifier)]`) now KeyErrors forever for that qid -- nothing
     ever sets it again -- so the gate stays closed and no further command in
     the queue is ever dequeued and sent, no matter how many arrive after it
     or how long the caller waits. Only a full reconnect() (a fresh thread,
     fresh locals) clears it.

This is what plausibly let the stage crash: once every position() read starts
timing out, colision_avoidance._move() has no reliable way to know where the
stage actually is, yet nothing stops it from continuing to issue moves.

The fix marks the outstanding command done in qeueIdSuccess whether or not its
response parsed, so one garbled frame costs exactly the response it belonged
to -- not every command queued after it.
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


class _GarbledThenEchoingSer:
    """The first command's response is unparseable garbage; every command
    after that gets a normal, valid echoed response. Reproduces "one bad
    frame, then the link is fine" -- not a dead connection -- which is
    exactly the case the old code could never recover from on its own."""
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
            # Framed like a real response (++ ... --) but the body itself is
            # not valid JSON -- this is what logs "Failed to load the json
            # from serial".
            self._lines += ["++", "{not valid json", "--"]
        else:
            self._lines += ["++", json.dumps({"qid": qid, "isbusy": 0}), "--"]

    def readline(self):
        if self._lines:
            return (self._lines.pop(0) + "\n").encode()
        return b""


def test_a_garbled_response_does_not_freeze_later_commands():
    parent = _FakeParent()
    s = _bare_serial(_GarbledThenEchoingSer(), parent)
    s.thread = threading.Thread(target=s._process_commands, daemon=True)
    s.thread.start()
    try:
        # First command: its response is garbled, so _process_commands hands
        # sendMessage() an empty dict for it (matching the qid it was sent
        # under) rather than a timeout -- this is the exact shape seen live
        # ("isBusy(): no valid response from ESP32 ({})"). That part is
        # expected and unrelated to the bug being tested here; what matters
        # is what happens to the *next* command.
        first = s.post_json("/motor_get", {"isbusy": 1}, timeout=0.3)
        assert first == {}

        # Second command, sent after the first gave up: must get its own,
        # real response -- not another timeout. Before the fix this also
        # timed out (and every command after it, forever), because the first
        # command's garbled response left the dispatch queue permanently
        # closed.
        second = s.post_json("/motor_get", {"isbusy": 1}, timeout=1.0)
        assert isinstance(second, dict), (
            f"expected a real response, got {second!r} -- the queue is "
            f"still stuck after the first command's garbled response"
        )
        assert second.get("isbusy") == 0
    finally:
        s.running = False
        s.thread.join(timeout=1)
