"""Regression tests for a 2026-09-16 production incident: mserial.py's
post_json() retry-once path calls self._parent.logger.warning(...), but the
real Logger class (uc2rest/logger.py) only ever defined error()/debug() --
no warning(). The unit tests covering that retry path used a hand-rolled
fake logger that happened to define warning(), so they passed while every
real call crashed with AttributeError. Combined with get_object()'s
exception re-raise (also added this session), that AttributeError crashed
the scan thread on every single retry -- and even crashed instrument
shutdown/cleanup when it hit the same code path turning lights off.

These tests exercise the REAL Logger class, not a double, so a missing
method here can never hide behind a mismatched test fake again.
"""
import json
import queue
import threading
import time

from uc2rest.logger import Logger
from uc2rest.mserial import Serial


def test_logger_has_a_working_warning_method():
    """The most direct regression check: call every level real code
    actually uses. This alone would have caught the incident."""
    logger = Logger()
    logger.debug("test debug")
    logger.error("test error")
    logger.warning("test warning")  # used to raise AttributeError


class _FakeParent:
    def __init__(self):
        self.logger = Logger()  # the REAL logger class, not a test double


class _SlowThenEchoingSer:
    """The first command's response is delayed past the caller's timeout
    but does eventually arrive, so post_json()'s retry-once-on-interrupt
    branch actually fires (mirrors test_serial_timeout.py's fixture of the
    same name)."""
    BAUDRATES = (110, 300)

    def __init__(self, delay_first_by=0.3):
        self._delay_first_by = delay_first_by
        self._first_write_seen = False
        self._pending = []
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


def test_post_json_retry_path_does_not_crash_with_the_real_logger():
    """End-to-end: drive post_json()'s actual retry-once-on-interrupt
    branch with the production Logger, not a test double -- the exact
    path that crashed in production."""
    parent = _FakeParent()
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
