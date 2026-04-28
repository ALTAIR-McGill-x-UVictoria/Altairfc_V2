import time
import threading
from pyvesc import VESC

DUTY = 1000
DURATION = 10

with VESC(serial_port='/dev/ttyACM0', baudrate=500000, start_heartbeat=False, timeout=0.5) as motor:
    stop_event = threading.Event()

    def keepalive():
        while not stop_event.is_set():
            motor.set_rpm(DUTY)
            time.sleep(0.05)

    t = threading.Thread(target=keepalive, daemon=True)
    t.start()

    for _ in range(DURATION):
        time.sleep(1)
        meas = motor.get_data()
        print(f"RPM: {meas.rpm}, Voltage: {meas.v_in}V, Current: {meas.avg_motor_current}A")

    stop_event.set()
    t.join()
    motor.set_rpm(0)
