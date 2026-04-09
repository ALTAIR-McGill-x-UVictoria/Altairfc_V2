from pyvesc.messages.setters import SetDutyCycle, SetRPM, SetCurrent, SetCurrentBrake, SetPosition, SetRotorPositionMode
from pyvesc.interface import decode, encode, encode_request
from pyvesc.messages.getters import GetValues
import time
import serial

class VESCObject:
    def __init__(self, port):
        self.port = serial.Serial(port, 115200, timeout=0.1)

    def set_rpm(self, rpm):
        pkt = encode(SetRPM(rpm))
        self.port.write(pkt)

    def set_current(self, current): # Current in milli Amps
        pkt = encode(SetCurrent(current))
        self.port.write(pkt)

    def set_brake_current(self, brake_current): # Might be useful
        pkt = encode(SetCurrentBrake(brake_current))
        self.port.write(pkt)

    def get_data(self):
        pkt = encode_request(GetValues)
        self.port.write(pkt)
        raw = ser.read(512)
        msg, consumed = decode(raw)
        if msg:
            return msg
