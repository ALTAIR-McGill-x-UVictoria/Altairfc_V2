import pigpio
import time

SERVO_PIN_A = 16
SERVO_PIN_B = 26

# Connect to pigpio daemon
pi = pigpio.pi()

if not pi.connected:
    print("Failed to connect to pigpio daemon. Did you run 'sudo pigpiod'?")
    exit()

def set_angle(pin, angle):
    # Clamp angle
    angle = max(0, min(180, angle))

    # Convert angle to pulse width (500-2500 us typical range)
    pulsewidth = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(pin, pulsewidth)

try:
    while True:
        print("0 / 180")
        set_angle(SERVO_PIN_A, 0)
        set_angle(SERVO_PIN_B, 180)
        time.sleep(2)

        print("90 / 90")
        set_angle(SERVO_PIN_A, 90)
        set_angle(SERVO_PIN_B, 90)
        time.sleep(2)

        print("180 / 0")
        set_angle(SERVO_PIN_A, 180)
        set_angle(SERVO_PIN_B, 0)
        time.sleep(2)

except KeyboardInterrupt:
    print("Stopping...")

# Turn off servo signals
pi.set_servo_pulsewidth(SERVO_PIN_A, 0)
pi.set_servo_pulsewidth(SERVO_PIN_B, 0)
pi.stop()
