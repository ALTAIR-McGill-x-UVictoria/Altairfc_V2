#!/usr/bin/env python3
"""
LED PWM Test Script - Raspberry Pi
Tests PWM-controlled LEDs on physical pins 28 and 38.

Physical pin 28 -> BCM GPIO 1  (ID_SC / general GPIO)
Physical pin 38 -> BCM GPIO 20

Wiring:
  Physical pin 28 (BCM 1)  -> 220Ω resistor -> LED anode -> LED cathode -> GND
  Physical pin 38 (BCM 20) -> 220Ω resistor -> LED anode -> LED cathode -> GND

Usage:
  sudo python3 test_led_pwm.py

Requires pigpio daemon:
  sudo pigpiod
"""

import pigpio
import time
import sys

# --- Pin mapping: physical -> BCM ---
# Physical 28 = BCM 1  (note: BCM 1 is normally I2C ID_SC; ensure it's free)
# Physical 38 = BCM 20
PIN_A = 1   # Physical pin 28
PIN_B = 20  # Physical pin 38

PWM_FREQUENCY = 1000  # Hz


def connect_pigpio() -> pigpio.pi:
    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] Cannot connect to pigpio daemon. Run: sudo pigpiod")
        sys.exit(1)
    print("[OK] Connected to pigpio daemon")
    return pi


def setup_pins(pi: pigpio.pi):
    for pin in (PIN_A, PIN_B):
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin, PWM_FREQUENCY)
        pi.set_PWM_dutycycle(pin, 0)
    print(f"[OK] Pins BCM {PIN_A} (phys 28) and BCM {PIN_B} (phys 38) configured")
    print(f"     PWM frequency: {PWM_FREQUENCY} Hz\n")


def set_brightness(pi: pigpio.pi, pin: int, percent: float):
    """Set LED brightness 0–100%."""
    dc = int(max(0.0, min(100.0, percent)) * 255 / 100)
    pi.set_PWM_dutycycle(pin, dc)


def test_on_off(pi: pigpio.pi):
    """Test 1: Simple on/off for both LEDs."""
    print("=== Test 1: On/Off ===")
    for label, pin in [("PIN_A (phys 28)", PIN_A), ("PIN_B (phys 38)", PIN_B)]:
        print(f"  {label} ON  (100%)")
        set_brightness(pi, pin, 100)
        time.sleep(1.0)
        print(f"  {label} OFF (0%)")
        set_brightness(pi, pin, 0)
        time.sleep(0.5)
    print()


def test_brightness_steps(pi: pigpio.pi):
    """Test 2: Step through discrete brightness levels."""
    print("=== Test 2: Brightness Steps (0 → 100% in 10% increments) ===")
    steps = range(0, 110, 10)
    for label, pin in [("PIN_A (phys 28)", PIN_A), ("PIN_B (phys 38)", PIN_B)]:
        print(f"  Stepping {label}:")
        for pct in steps:
            print(f"    {pct:3d}%", end="\r")
            set_brightness(pi, pin, pct)
            time.sleep(0.25)
        set_brightness(pi, pin, 0)
        print(f"    Done.          ")
    print()


def test_fade(pi: pigpio.pi, duration: float = 2.0):
    """Test 3: Smooth fade in/out using small steps."""
    print("=== Test 3: Smooth Fade In/Out ===")
    steps = 200
    step_time = duration / steps

    for label, pin in [("PIN_A (phys 28)", PIN_A), ("PIN_B (phys 38)", PIN_B)]:
        print(f"  Fading {label} up...")
        for i in range(steps + 1):
            set_brightness(pi, pin, i * 100 / steps)
            time.sleep(step_time)
        print(f"  Fading {label} down...")
        for i in range(steps, -1, -1):
            set_brightness(pi, pin, i * 100 / steps)
            time.sleep(step_time)
        set_brightness(pi, pin, 0)
    print()


def test_alternating(pi: pigpio.pi, cycles: int = 4):
    """Test 4: Alternate both LEDs at different brightness levels."""
    print("=== Test 4: Alternating LEDs ===")
    for i in range(cycles):
        pct_a = 100 if i % 2 == 0 else 20
        pct_b = 20  if i % 2 == 0 else 100
        print(f"  Cycle {i+1}: PIN_A={pct_a}%  PIN_B={pct_b}%")
        set_brightness(pi, PIN_A, pct_a)
        set_brightness(pi, PIN_B, pct_b)
        time.sleep(0.8)
    set_brightness(pi, PIN_A, 0)
    set_brightness(pi, PIN_B, 0)
    print()


def test_simultaneous_fade(pi: pigpio.pi, duration: float = 3.0):
    """Test 5: Both LEDs fade in opposite directions simultaneously."""
    print("=== Test 5: Simultaneous Opposing Fade ===")
    steps = 200
    step_time = duration / steps
    for i in range(steps + 1):
        set_brightness(pi, PIN_A, i * 100 / steps)
        set_brightness(pi, PIN_B, (steps - i) * 100 / steps)
        time.sleep(step_time)
    set_brightness(pi, PIN_A, 0)
    set_brightness(pi, PIN_B, 0)
    print("  Done.\n")


def cleanup(pi: pigpio.pi):
    for pin in (PIN_A, PIN_B):
        pi.set_PWM_dutycycle(pin, 0)
        pi.set_mode(pin, pigpio.INPUT)
    pi.stop()
    print("[OK] Cleanup complete. pigpio connection closed.")


def main():
    print("=" * 50)
    print(" LED PWM Test — pigpio")
    print(f" PIN_A: BCM {PIN_A}  (physical 28)")
    print(f" PIN_B: BCM {PIN_B}  (physical 38)")
    print("=" * 50)
    print()

    pi = connect_pigpio()
    setup_pins(pi)

    try:
        test_on_off(pi)
        test_brightness_steps(pi)
        test_fade(pi)
        test_alternating(pi)
        test_simultaneous_fade(pi)
        print("All tests passed.")
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Ctrl+C detected.")
    finally:
        cleanup(pi)


if __name__ == "__main__":
    main()