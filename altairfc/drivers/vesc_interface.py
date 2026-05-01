from pyvesc.VESC.messages.setters import SetDutyCycle, SetRPM, SetCurrent, SetCurrentBrake, SetPosition, SetRotorPositionMode
from pyvesc.protocol.interface import decode, encode, encode_request
from pyvesc.VESC.messages.getters import GetValues
import time
import serial

class VESCObject:
    def __init__(self, port):
        self.port = serial.Serial(port, 115200, timeout=0.1)
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

    def get_data(self, timeout = 0.02):
        pkt = encode_request(GetValues)
        self.port.write(pkt)

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout: # Wait for
            chunk = self.port.read(512)
            if chunk:
                self.buffer += chunk
            msg, consumed = decode(self.buffer)
            if consumed > 0:
                self.buffer = self.buffer[consumed:]

            if msg is not None:
                return msg
        return None

