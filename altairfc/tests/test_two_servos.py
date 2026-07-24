import argparse
import time

import pigpio

SERVO_PINS = {"a": 16, "b": 26}
STEP_PERIOD_S = 0.02  # 50 Hz update rate for slewing


def set_angle(pi, pin, angle):
    # Clamp angle
    angle = max(0, min(180, angle))

    # Convert angle to pulse width (500-2500 us typical range)
    pulsewidth = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(pin, pulsewidth)
    return angle


def move_to_angle(pi, pin, current_angle, target_angle, slew_rate):
    target_angle = max(0, min(180, target_angle))

    if current_angle is None or not slew_rate:
        return set_angle(pi, pin, target_angle)

    step = slew_rate * STEP_PERIOD_S
    angle = current_angle

    while abs(target_angle - angle) > step:
        angle += step if target_angle > angle else -step
        set_angle(pi, pin, angle)
        time.sleep(STEP_PERIOD_S)

    return set_angle(pi, pin, target_angle)


def main():
    parser = argparse.ArgumentParser(description="Manually drive a single servo by angle.")
    parser.add_argument("servo", choices=sorted(SERVO_PINS), help="Which servo to control")
    parser.add_argument(
        "--slew-rate",
        type=float,
        default=None,
        help="Max speed to move between angles, in deg/s (default: move instantly)",
    )
    args = parser.parse_args()

    pin = SERVO_PINS[args.servo]

    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon. Did you run 'sudo pigpiod'?")
        return

    print(f"Controlling servo '{args.servo}' on GPIO {pin}. Enter an angle (0-180), or 'q' to quit.")
    if args.slew_rate:
        print(f"Slew rate limited to {args.slew_rate} deg/s")

    current_angle = None

    try:
        while True:
            user_input = input("Angle: ").strip()

            if user_input.lower() in ("q", "quit", "exit"):
                break

            try:
                angle = float(user_input)
            except ValueError:
                print("Please enter a number between 0 and 180, or 'q' to quit.")
                continue

            current_angle = move_to_angle(pi, pin, current_angle, angle, args.slew_rate)
            print(f"Set to {current_angle} degrees")

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        pi.set_servo_pulsewidth(pin, 0)
        pi.stop()


if __name__ == "__main__":
    main()
