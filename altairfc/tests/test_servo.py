import RPi.GPIO as GPIO
import time

SERVO_PIN = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_angle(angle):
    # Map angle (0–180) to duty cycle (~2.5–12.5)
    duty = 2.5 + (angle / 180.0) * 10
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)

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

pwm.stop()
GPIO.cleanup()