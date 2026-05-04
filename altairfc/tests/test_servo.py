import pigpio
import time

SERVO_PIN = 26

# Connect to pigpio daemon
pi = pigpio.pi()

if not pi.connected:
    print("Failed to connect to pigpio daemon. Did you run 'sudo pigpiod'?")
    exit()

def set_angle(angle):
    # Clamp angle
    angle = max(0, min(180, angle))
    
    # Convert angle to pulse width (500–2500 µs typical range)
    pulsewidth = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(SERVO_PIN, pulsewidth)

try:
    while True:
        print("0°")
        set_angle(0)
        time.sleep(1)

        print("90°")
        set_angle(90)
        time.sleep(1)

        print("180°")
        set_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping...")

# Turn off servo signal
pi.set_servo_pulsewidth(SERVO_PIN, 0)
pi.stop()