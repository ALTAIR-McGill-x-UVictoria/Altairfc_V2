from pyvesc import VESC
import time


class VESCObject:
    def __init__(self, port):
        self.port = port
        self.motor = VESC(serial_port=port)

    def set_rpm(self, rpm):
        self.motor.set_rpm(rpm)

    def set_duty_cycle(self, duty_cycle): # Duty cycle between -1 and 1
        self.motor.set_duty_cycle(duty_cycle)

    def set_current(self, current): # Current in milli Amps
        self.motor.set_current(current)

    def set_brake_current(self, brake_current): # Might be useful
        self.motor.set_brake_current(brake_current)

    def get_rpm(self):
        return self.motor.get_rpm()

    def get_current(self):
        return self.motor.get_current()

    def get_voltage(self):
        return self.motor.get_voltage()