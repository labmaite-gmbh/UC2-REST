import serial
import json
import queue
import threading
import time


class CommunicationError(Exception):
    """A serial exchange with the ESP32 could not be completed (the
    qid-framed response was lost, timed out, or came back malformed).
    Distinct from a plain bug so callers (see esp32_conn.py's
    call_with_retry) can retry specifically on this and not on a real
    programming error."""
    pass


T_SERIAL_WARMUP = 1.5
class Serial:
    # Reopening the same port immediately after closing it (exactly what
    # reconnect() does) can hit a transient failure while the OS finishes
    # releasing the handle -- a 2026-09-16 live-hardware incident hit this
    # on Windows (PermissionError: Access is denied) and, with only the
    # original 2 attempts and no delay, fell straight through to
    # findCorrectSerialDevice()'s port-scan and then a silent MockSerial
    # "dummy" fallback, with no exception raised anywhere. A short backoff
    # between attempts lets the transient case clear before giving up.
    _REOPEN_ATTEMPTS = 5
    _REOPEN_RETRY_DELAY_S = 0.3

    # Bounds how long _process_commands() will wait for ANY response at all
    # -- not just a garbled one -- to the command it just sent, before giving
    # up on it and letting the dispatch loop move on to the next queued
    # command. Total silence (zero bytes back) is a second, distinct way the
    # dispatch gate used to freeze forever alongside a garbled response: with
    # nothing ever received, reading_json never becomes True, so the
    # "--"-terminated branch that marks a command done (see qeueIdSuccess)
    # is never reached either.
    #
    # Must stay BELOW the tightest per-call timeout callers actually use in a
    # hot loop -- Motor.isBusy()'s default of 1s, polled continuously by
    # wait_for_move_complete() -- not above it. If this were >= that timeout,
    # the caller gives up and post_json()'s single retry re-queues a second
    # command under a new qid before the dispatch gate has been freed for the
    # first one, so the retry (its own fresh ~1s budget, racing a gate that
    # won't open for however much of this timeout is left) usually times out
    # too, and the caller falls through to a full reconnect() instead of the
    # plain silence recovery this exists to provide. See
    # test_silence_timeout_is_below_motor_isbusy_poll_timeout, which pins
    # this relationship so the two can't drift apart again unnoticed.
    _SILENCE_TIMEOUT_S = 0.6

    def __init__(self, port, baudrate=115200, timeout=5,
                 identity="UC2_Feather", parent=None, DEBUG=False):

        self.ser = None
        self.serialport = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._parent = parent
        self.identity = identity
        self.DEBUG = DEBUG
        self.is_connected = False
        self.write_timeout = 0.02
        self.read_timeout = 0.02

        self.cmdCallBackFct = None

        # setup command queue
        self.resetLastCommand = False
        self.command_queue = queue.Queue()
        self.responses = {}
        self.commands = {}
        self.lock = threading.Lock()

        self.callBackList = []

        # initialize serial connection
        self.thread = None
        self.ser = self.openDevice(port, baudrate)

    def breakCurrentCommunication(self):
        self.resetLastCommand = True

    def _freeSerialBuffer(self, ser, timeout=5):
        t0 = time.time()
        # free up any old data
        while True:
            try:
                readLine = ser.readline().decode('utf-8').strip()
                if self.DEBUG: self._parent.logger.debug(readLine)
                if readLine == "":
                    break
            except Exception as e:
                if self.DEBUG: self._parent.logger.debug(e)

                pass
            if time.time()-t0 > timeout:
                return

    def openDevice(self, port=None, baud_rate=115200):
        try: # try to close an eventually open serial connection
            if str(type(self.ser)) != "<class 'uc2rest.mserial.MockSerial'>":
                self.ser.close()
        except: pass

        try:
            isUC2 = False
            for i in range(self._REOPEN_ATTEMPTS):
                isUC2 = self.tryToConnect(port)
                if isUC2:
                    break
                time.sleep(self._REOPEN_RETRY_DELAY_S)
            if not isUC2:
                raise ValueError('Wrong Firmware.')
            ser = self.serialdevice
            self.is_connected = True
            
        except Exception as e:
            self._parent.logger.error(e)
            ser = self.findCorrectSerialDevice()
            if ser is None:
                ser = MockSerial(port, baud_rate, timeout=.1)
                self.is_connected = False
        ser.write_timeout = self.write_timeout
        if not ser.isOpen():
            ser.open()
        # TODO: Need to be able to auto-connect
        # need to let device warm up and flush out any old data
        self._freeSerialBuffer(ser)

        # _process_commands() (started below) reads self.ser, not this
        # function's local `ser` -- it must be assigned before the thread
        # starts, not left for the caller to assign from this function's
        # return value afterward. Otherwise the new thread's very first
        # loop iteration can see a stale or None self.ser and immediately
        # flip is_connected back to False, racing whoever just reconnected.
        self.ser = ser

        # remove any remaining thread in case there was one open
        try:
            del self.thread
        except:
            pass
        self.running = True
        self.identifier_counter = 0 # Counter for generating unique identifiers
        self.thread = threading.Thread(target=self._process_commands)
        self.thread.start()
        return ser

    def findCorrectSerialDevice(self):
        _available_ports = serial.tools.list_ports.comports(include_links=False)
        ports_to_check = ["COM", "/dev/tt", "/dev/a", "/dev/cu.SLA", "/dev/cu.wchusb"]
        descriptions_to_check = ["CH340", "CP2102"]

        for port in _available_ports:
            if any(port.device.startswith(allowed_port) for allowed_port in ports_to_check) or \
               any(port.description.startswith(allowed_description) for allowed_description in descriptions_to_check):
                if self.tryToConnect(port.device):
                    self.is_connected = True
                    return self.serialdevice

        self.is_connected = False
        self.serialport = "NotConnected"
        self.serialdevice = None
        self._parent.logger.debug("No USB device connected! Using DUMMY!")

    def tryToConnect(self, port):
        try:
            self.serialdevice = serial.Serial(port=port, baudrate=self.baudrate, timeout=self.read_timeout, write_timeout=self.write_timeout)
            time.sleep(T_SERIAL_WARMUP)
            self._freeSerialBuffer(self.serialdevice)
            if self.checkFirmware(self.serialdevice):
                self.is_connected = True
                self.NumberRetryReconnect = 0
                return True
            else:
                False

        except Exception as e:
            self._parent.logger.debug(f"Trying out port {port} failed")
            self._parent.logger.error(e)

        return False

    def checkFirmware(self, ser):
        """Check if the firmware is correct
        We do not do that inside the queue processor yet
        """
        path = "/state_get"
        payload = {"task": path}

        ser.write(json.dumps(payload).encode('utf-8'))
        ser.write(b'\n')
        # iterate a few times in case the debug mode on the ESP32 is turned on and it sends additional lines
        for i in range(10):
            # if we just want to send but not even wait for a response
            mReadline = ser.readline()
            if self.DEBUG: self._parent.logger.debug(mReadline)
            if mReadline.decode('utf-8').strip() == "++":
                self._freeSerialBuffer(ser)
                return True
        return False


    def _generate_identifier(self):
        self.identifier_counter += 1
        return self.identifier_counter

    def _process_commands(self):
        buffer = ""
        reading_json = False
        currentIdentifier = None
        nLineCountTimeout = 50 # maximum number of lines read before timeout
        lineCounter = 0
        lastTransmisionSuccess = True
        # When the outstanding command (currentIdentifier) was sent, so total
        # silence can be bounded the same way a garbled response already is
        # -- see _SILENCE_TIMEOUT_S and the check just below the read.
        t_sent = None

        qeueIdSuccess = {}
        t0 = time.time()
        while self.running:

            # Check if the last command went through successfully
            if currentIdentifier is not None:
                try: lastTransmisionSuccess = qeueIdSuccess[str(currentIdentifier)]
                except: lastTransmisionSuccess = False
            if not self.command_queue.empty() and not reading_json and lastTransmisionSuccess:
                currentIdentifier, command = self.command_queue.get()
                t_sent = time.time()

                if self.DEBUG: self._parent.logger.debug("Sending: "+ str(command))
                json_command = json.dumps(command)
                if currentIdentifier == 5:
                    self._parent.logger.debug("Sending: "+ str(command))
                try:
                    self.ser.write(json_command.encode('utf-8'))
                except Exception as e:
                    try:
                        self.ser.write_timeout = 1
                        self.ser.write(json_command.encode('utf-8'))
                        self.ser.write_timeout=self.write_timeout
                    except Exception as e:
                        self._parent.logger.error("Writing failed in serial")
                        self._parent.logger.error(e)
                try:self.ser.write(b'\n')
                except:
                    self._parent.logger.error("Break the loop in serial") 
                    break
             
            # device not ready yet
            if self.ser is None:
                self.is_connected = False
                continue
            else:
                self.is_connected = True

            # if we just want to send but not even wait for a response
            try:
                mReadline = self.ser.readline()
            except Exception as e:
                self._parent.logger.error("Failed to read the line in serial")
                self._parent.logger.error(e)
                self.is_connected = False
                break
            try:
                line = mReadline.decode('utf-8').strip()
                if self.DEBUG and line!="": self._parent.logger.debug(line)
            except:
                line = ""
            if line == "":
                # Total silence -- zero bytes back for the command we sent,
                # not even a garbled frame. reading_json never becomes True
                # in this case, so the "--"-terminated branch below (which
                # marks a command done after a garbled response) is never
                # reached either: this is a second, distinct way the
                # dispatch gate used to freeze forever. Bounded the same way,
                # via _SILENCE_TIMEOUT_S, so a truly gone-silent link cannot
                # strand every command queued after the one it swallowed.
                if (currentIdentifier is not None and not reading_json
                        and t_sent is not None
                        and time.time() - t_sent > self._SILENCE_TIMEOUT_S):
                    self._parent.logger.debug(
                        f"No response at all for qid={currentIdentifier} after "
                        f"{self._SILENCE_TIMEOUT_S:.0f}s of silence; unsticking "
                        f"the dispatch queue.")
                    with self.lock:
                        qeueIdSuccess[str(currentIdentifier)] = 1
                        try:
                            self.responses[currentIdentifier].append({})
                        except:
                            self.responses[currentIdentifier] = [{}]
                    # currentIdentifier must stay set to this qid (not be
                    # reset to None) -- the dispatch gate at the top of the
                    # loop only refreshes lastTransmisionSuccess when
                    # currentIdentifier is not None, so resetting it here
                    # would leave that gate stuck on its last (closed) value
                    # forever instead of picking up the qeueIdSuccess update
                    # just made above. Clearing t_sent alone is enough to
                    # stop this check from re-firing on every idle loop
                    # iteration once the timeout has already been handled.
                    t_sent = None
            elif line == "++":
                reading_json = True
                continue
            elif line == "--" or lineCounter>nLineCountTimeout:
                lineCounter = 0
                reading_json = False
                try:
                    json_response = json.loads(buffer)
                    if len(self.callBackList) > 0:
                        for callback in self.callBackList:
                            # check if json has key
                            try:
                                if callback["pattern"] in json_response:
                                    callback["callbackfct"](json_response)
                            except Exception as e:
                                self._parent.logger.debug(e)


                except:
                    self._parent.logger.debug("Failed to load the json from serial")
                    json_response = {}

                with self.lock:
                    # The command this exchange answers is whichever qid the
                    # response carries, or -- when it didn't parse, or parsed
                    # without a qid -- currentIdentifier, i.e. the command that
                    # was actually outstanding (set when it was dequeued and
                    # sent, above). Either way it must be marked done in
                    # qeueIdSuccess: that is what lets the dequeue gate at the
                    # top of this loop move on to the next queued command.
                    # Previously this was only ever set from inside the JSON
                    # parse's own try block, so a single garbled/unparseable
                    # response left it unset forever for that qid -- and
                    # since nothing else ever sets it, every command queued
                    # after that one waited on a gate that could never
                    # reopen, for the rest of the connection's life (only a
                    # full reconnect(), with fresh locals, cleared it). Traced
                    # from a 2026-09-17 live run where this silently stalled
                    # every stage move for the last ~18 minutes of an
                    # experiment after exactly one "Failed to load the json
                    # from serial".
                    qid = json_response.get("qid", currentIdentifier)
                    if qid is not None:
                        qeueIdSuccess[str(qid)] = 1
                        currentIdentifier = qid
                    try:
                        self.responses[currentIdentifier].append(json_response.copy())
                    except:
                        self.responses[currentIdentifier] = list()
                        self.responses[currentIdentifier].append(json_response.copy())
                buffer = ""     # reset buffer

            if reading_json:
                buffer += line
                lineCounter +=1
        self.running = False

    def get_json(self, path):
        message = {"task":path}
        message = json.dumps(message)
        return self.sendMessage(message, nResponses=0)

    def post_json(self, path, payload, getReturn=True, nResponses=1, timeout=20):
        """Make an HTTP POST request and return the JSON response.

        Retries once on "communication interrupted": a fresh call gets a new
        qid, so it costs nothing extra once a transient desync (see
        esp32_conn.py's get_object() comment on the qid-framed protocol) has
        cleared up, and it's the cheapest thing to try before a caller
        considers a full reconnect().
        """
        if payload is None:
            payload = {}
        if "task" not in payload:
            payload["task"] = path

        # write message to the serial
        if not getReturn:
            nResponses = -1
        if self.cmdCallBackFct is not None:
            self.cmdCallBackFct(payload)
            return "OK"

        writeResult = self.sendMessage(command=payload, nResponses=nResponses, timeout=timeout)
        if writeResult == "communication interrupted" and nResponses > 0:
            self._parent.logger.warning(
                f"Serial command '{payload.get('task', path)}' was interrupted; retrying once.")
            writeResult = self.sendMessage(command=payload, nResponses=nResponses, timeout=timeout)
        return writeResult

    def writeSerial(self, payload):
        return self.sendMessage(payload, nResponses=-1)

    def breakCurrentCommunication(self):
        pass # not needed anymore

    def readSerial(self, qid=0, timeout=1):
        t0 = time.time()
        while time.time()-t0<timeout:
            try:
                return self.responses[qid]
            except:
                pass
        return {"timeout": 1}

    def register_callback(self, callback, pattern):
        '''
        we need to add a callback function to a list of callbacks that will be read during the serial communication
        loop
        ''' 
        self.callBackList.append({"callbackfct":callback, "pattern":pattern})
        
    def sendMessage(self, command, nResponses=1, timeout = 20):
        '''
        Sends a command to the device and optionally waits for a response.
        If nResponses is 0, then the command is sent but no response is expected.
        If nResponses is 1, then the command is sent and the response is returned.
        If nResponses is >1, then the command is sent and a list of responses is returned.
        '''
        t0 = time.time()
        if type(command) == str:
            command = json.loads(command)
        identifier = self._generate_identifier()
        command["qid"] = identifier
        self.command_queue.put((identifier, command))
        self.commands[identifier]=command
        if nResponses <= 0 or not self.is_connected or not type(self.ser.BAUDRATES) is tuple:
            return identifier
        while self.running:
            time.sleep(0.002)
            elapsed = time.time() - t0
            if self.resetLastCommand or elapsed > timeout or not self.is_connected:
                if elapsed > timeout and not self.resetLastCommand and self.is_connected:
                    # No matching response arrived in time: the caller gives up here, but the
                    # command stays in commands/command_queue and _process_commands may still be
                    # stuck waiting on it, so every command behind it in the queue pays out this
                    # same timeout next. Logging qid/task/elapsed is what makes that visible --
                    # previously this failure mode was silent and only reconstructable after the
                    # fact from gaps between unrelated log lines.
                    task = command.get("task", "?") if isinstance(command, dict) else "?"
                    self._parent.logger.error(
                        f"Serial command timed out after {elapsed:.1f}s waiting for a response "
                        f"(qid={identifier}, task={task}, wanted {nResponses} response(s)); "
                        f"giving up.")
                self.resetLastCommand = False
                return "communication interrupted"
            with self.lock:
                if identifier in self.responses:
                    if len(self.responses[identifier])==nResponses:
                        return self.responses[identifier][-1]
                if -identifier in self.responses:
                    self._parent.logger.debug("You have sent the wrong command!")
                    return "Wrong Command"

    def interruptCurrentSerialCommunication(self):
        self.resetLastCommand = True

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()

    def closeSerial(self):
        self.stop()

    def reconnect(self):
        """Stop the current reader thread, then close and reopen the port.

        openDevice() always starts a fresh reader thread on the new
        connection, so the old one must be fully stopped first -- otherwise
        two threads can briefly read the same (or an about-to-be-replaced)
        serial object at once, which is exactly the kind of interleaving
        that desyncs the qid-framed protocol (see get_object()'s comment in
        esp32_conn.py). A bounded join keeps this from hanging forever if
        the old thread is itself stuck; reconnecting anyway at that point is
        the better failure mode than reconnect() never returning.
        """
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        try:
            self.ser.close()
        except:
            pass
        self.ser = self.openDevice(port=self.serialport, baud_rate=self.baudrate)
        # is_connected is not a reliable signal here: _process_commands()'s
        # own thread (just started by openDevice()) flips it back to True
        # for any non-None self.ser, dummy included, as soon as it runs its
        # first loop iteration -- racing this check. The object's type is
        # unambiguous instead (same idiom openDevice() itself uses above).
        if str(type(self.ser)) == "<class 'uc2rest.mserial.MockSerial'>":
            # openDevice() exhausted its own reopen retries and fell back to
            # a disconnected/dummy serial backend -- that must not look like
            # a successful reconnect to a caller (e.g. esp32_conn.py's
            # call_with_retry), which would otherwise happily retry against
            # a connection that can never respond.
            raise CommunicationError(
                f"reconnect(): could not reopen {self.serialport!r} -- "
                f"fell back to a disconnected/dummy serial backend")

    def toggleCommandOutput(self, cmdCallBackFct=None):
        # if true, all commands will be output to a callback function and stored for later use
        self.cmdCallBackFct = cmdCallBackFct

if __name__ == "__main__":
    # Usage example
    monitor = Serial('/dev/cu.SLAB_USBtoUART', baudsrate=115200)  # Change to your port

    command_to_send = {
            "task": "/state_get"
    }

    command_to_send = {"task":"/motor_act","motor":{"steppers": [{ "stepperid": 3, "position": -1000, "speed": 15000, "isabs": 0, "isaccel":0}]}}

    t0 = time.time()
    response = monitor.sendMessage(command_to_send, nResponses=2)
    print("Response:", response)
    print("time:", time.time()-t0)

    t0 = time.time()
    response = monitor.sendMessage(command_to_send, nResponses=0)
    print("Response:", response)
    print("time:", time.time()-t0)


    #response = monitor.waitForResponse(command_id)

    if response:
        print("Response:", response)

    monitor.stop()


class SerialManagerWrapper:

    def __init__(self) -> None:
        pass

import random
from threading import Thread
class MockSerial:
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = False
        self.data_buffer = []
        self.thread = Thread(target=self._simulate_data)
        self.thread.daemon = True
        self.thread.start()
        self.is_open = True
        self.manufacturer = "UC2Mock"
        self.BAUDRATES = -1

    def isOpen(self):
        return self.is_open 
    
    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def readline(self, timeout=1):
        if not self.is_open:
            raise Exception("Device not connected")
        if len(self.data_buffer) == 0:
            return b''
        data = self.data_buffer
        self.data_buffer = self.data_buffer
        time.sleep(.05)
        return bytes(data)

    def read(self, num_bytes):
        if not self.is_open:
            raise Exception("Device not connected")
        if len(self.data_buffer) == 0:
            return b''
        data = self.data_buffer[:num_bytes]
        self.data_buffer = self.data_buffer[num_bytes:]
        return bytes(data)

    def write(self, data):
        if not self.is_open:
            raise Exception("Device not connected")
        pass  # Do nothing, as it's a mock

    def _simulate_data(self):
        while self.is_open:
            if random.random() < 0.2:  # Simulate occasional data availability
                self.data_buffer.extend([random.randint(0, 255) for _ in range(10)])
            time.sleep(0.1)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
