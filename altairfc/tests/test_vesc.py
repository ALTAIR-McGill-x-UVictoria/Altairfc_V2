from pyvesc import VESC

with VESC(serial_port='/dev/ttyACM0', baudrate=115200, start_heartbeat=False) as motor:
    # Read telemetry
    meas = motor.get_measurements()
    print(f"RPM: {meas.rpm}, Voltage: {meas.input_voltage}V, Current: {meas.avg_motor_current}A")

    # Command modes (pick one)
    motor.set_duty_cycle(0.1)   # 10% duty — gentle test
    motor.set_rpm(2000)          # RPM control (requires RPM PID tuned in VESC Tool)
    motor.set_current(1.0)       # Current control (Amps)