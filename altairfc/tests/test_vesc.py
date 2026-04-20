import time
from pyvesc import VESC

with VESC(serial_port='/dev/ttyACM0', baudrate=500000, start_heartbeat=False) as motor:
    for _ in range(10):
        motor.set_duty_cycle(0.1)
        time.sleep(1)
        meas = motor.get_measurements()
        print(f"RPM: {meas.rpm}, Voltage: {meas.v_in}V, Current: {meas.avg_motor_current}A")
    motor.set_current(0)
