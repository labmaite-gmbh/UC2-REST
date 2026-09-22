"""tryToConnect() must open the COM port with DTR and RTS held LOW.

pyserial asserts both lines on open(). On the CH340/CP2102 auto-reset
circuit this board has (the same one hard_reset() deliberately drives),
that pulse reboots the ESP32 -- so every reconnect() cost a firmware
reboot, and the first command after it timed out ("Serial command timed
out ... qid=1, task=/motor_get" x189 in the 2026-09-20 console capture).
pyserial lets you avoid the pulse by setting .dtr/.rts on an UNOPENED
Serial() and calling open() afterwards.
"""
import queue
import threading

import uc2rest.mserial as mserial
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


class _FakePySerial:
    """Stands in for serial.Serial. Records the order of dtr/rts assignment
    versus open(), and refuses a port passed to the constructor (that form
    opens immediately, with the pulse)."""
    instances = []

    def __init__(self, port=None, **kwargs):
        assert port is None, "port must be set after construction, not passed to Serial()"
        self.port = None
        self.baudrate = None
        self.timeout = None
        self.write_timeout = None
        self.events = []
        self.is_open = False
        _FakePySerial.instances.append(self)

    def __setattr__(self, name, value):
        if name in ("dtr", "rts"):
            self.__dict__.setdefault("events", []).append((name, value))
        object.__setattr__(self, name, value)

    def open(self):
        self.events.append(("open", None))
        self.is_open = True

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


def test_try_to_connect_holds_dtr_and_rts_low_before_opening(monkeypatch):
    monkeypatch.setattr(mserial.serial, "Serial", _FakePySerial)
    monkeypatch.setattr(mserial, "T_SERIAL_WARMUP", 0.0)
    _FakePySerial.instances.clear()
    s = _bare_serial(_FakeParent())
    s.checkFirmware = lambda ser: True

    assert s.tryToConnect("COM4") is True

    ser = _FakePySerial.instances[0]
    assert ser.port == "COM4"
    assert ser.baudrate == 115200
    open_idx = ser.events.index(("open", None))
    assert ("dtr", False) in ser.events[:open_idx]
    assert ("rts", False) in ser.events[:open_idx]
