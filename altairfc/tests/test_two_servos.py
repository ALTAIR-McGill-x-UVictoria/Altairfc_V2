import argparse

import pigpio

SERVO_PINS = {"a": 16, "b": 26}


def set_angle(pi, pin, angle):
    # Clamp angle
    angle = max(0, min(180, angle))

    # Convert angle to pulse width (500-2500 us typical range)
    pulsewidth = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(pin, pulsewidth)


def main():
    parser = argparse.ArgumentParser(description="Manually drive a single servo by angle.")
    parser.add_argument("servo", choices=sorted(SERVO_PINS), help="Which servo to control")
    args = parser.parse_args()

    pin = SERVO_PINS[args.servo]

    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon. Did you run 'sudo pigpiod'?")
        return

    print(f"Controlling servo '{args.servo}' on GPIO {pin}. Enter an angle (0-180), or 'q' to quit.")

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

            set_angle(pi, pin, angle)
            print(f"Set to {max(0, min(180, angle))} degrees")

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        pi.set_servo_pulsewidth(pin, 0)
        pi.stop()


if __name__ == "__main__":
    main()
