from pyvesc.VESC.messages.setters import SetDutyCycle, SetRPM, SetCurrent, SetCurrentBrake, SetPosition, SetRotorPositionMode
from pyvesc.protocol.interface import decode, encode, encode_request
from pyvesc.VESC.messages.getters import GetValues
import time
import serial
import logging

logger = logging.getLogger(__name__)

_GET_VALUES_REQ = encode_request(GetValues)

class VESCObject:
    def __init__(self, port):
        # Short timeout: read(n) blocks until n bytes or timeout, so a long one stalls every read
        self.port = serial.Serial(port, 115200, timeout=0.005)
        self.buffer = b""

    def set_rpm(self, rpm):
        pkt = encode(SetRPM(rpm*7))
        self.port.write(pkt)

    def set_current(self, current): # Current in milli Amps
        pkt = encode(SetCurrent(current))
        self.port.write(pkt)

    def set_brake_current(self, brake_current): # Might be useful
        pkt = encode(SetCurrentBrake(brake_current))
        self.port.write(pkt)

    def request_data(self):
        """Send a GetValues request without waiting; the reply is picked up by poll_data()."""
        self.port.write(_GET_VALUES_REQ)

    def poll_data(self):
        """Return the newest reply already received, or None. Never blocks."""
        n = self.port.in_waiting
        if n:
            self.buffer += self.port.read(n)
        return self._decode_buffered()

    def get_data(self, timeout = 0.02):
        """Blocking request/reply; used at connect time."""
        try:
            self.port.reset_input_buffer()  # drop late replies so they aren't returned as fresh data
            self.buffer = b""
            self.port.write(_GET_VALUES_REQ)

            t0 = time.perf_counter()
            while time.perf_counter() - t0 < timeout:
                chunk = self.port.read(self.port.in_waiting or 1)
                if chunk:
                    self.buffer += chunk
                    msg = self._decode_buffered()
                    if msg is not None:
                        return msg
            return None
        except serial.SerialException as e:
            logger.error("VESC serial error: %s", e)
            raise

    def _decode_buffered(self):
        # Decode every complete message in the buffer, keep the newest, leave a partial one for next time
        latest = None
        while self.buffer:
            msg, consumed = decode(self.buffer)
            if consumed == 0:
                break
            self.buffer = self.buffer[consumed:]
            if msg is not None:
                latest = msg
        return latest

