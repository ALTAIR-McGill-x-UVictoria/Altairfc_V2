#!/usr/bin/env python3
"""
LED PWM Test Script - Raspberry Pi
Tests PWM-controlled LED on physical pin 38.

Physical pin 38 -> BCM GPIO 20

Wiring:
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
# Physical 38 = BCM 20
PIN_B = 26  # Physical pin 38

PWM_FREQUENCY = 200  # Hz — LDD-700L max is 1 kHz; stay well below for reliable dimming


def connect_pigpio() -> pigpio.pi:
    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] Cannot connect to pigpio daemon. Run: sudo pigpiod")
        sys.exit(1)
    print("[OK] Connected to pigpio daemon")
    return pi


def setup_pins(pi: pigpio.pi):
    pi.set_mode(PIN_B, pigpio.OUTPUT)
    pi.set_PWM_frequency(PIN_B, PWM_FREQUENCY)
    pi.set_PWM_dutycycle(PIN_B, 0)
    print(f"[OK] Pin BCM {PIN_B} (phys 38) configured")
    print(f"     PWM frequency: {PWM_FREQUENCY} Hz\n")


def set_brightness(pi: pigpio.pi, pin: int, percent: float):
    """Set LED brightness 0–100%. NLDD-700H is active-high so duty cycle maps directly."""
    dc = int(max(0.0, min(100.0, percent)) * 255 / 100)
    pi.set_PWM_dutycycle(pin, dc)


def test_on_off(pi: pigpio.pi):
    """Test 1: Simple on/off."""
    print("=== Test 1: On/Off ===")
    print(f"  PIN_B (phys 38) ON  (100%)")
    set_brightness(pi, PIN_B, 100)
    time.sleep(1.0)
    print(f"  PIN_B (phys 38) OFF (0%)")
    set_brightness(pi, PIN_B, 0)
    time.sleep(0.5)
    print()


def test_brightness_steps(pi: pigpio.pi):
    """Test 2: Step through discrete brightness levels."""
    print("=== Test 2: Brightness Steps (0 → 100% in 10% increments) ===")
    steps = range(0, 110, 10)
    print(f"  Stepping PIN_B (phys 38):")
    for pct in steps:
        print(f"    {pct:3d}%", end="\r")
        set_brightness(pi, PIN_B, pct)
        time.sleep(0.25)
    set_brightness(pi, PIN_B, 0)
    print(f"    Done.          ")
    print()


def test_fade(pi: pigpio.pi, duration: float = 2.0):
    """Test 3: Smooth fade in/out using small steps."""
    print("=== Test 3: Smooth Fade In/Out ===")
    steps = 200
    step_time = duration / steps

    print(f"  Fading PIN_B (phys 38) up...")
    for i in range(steps + 1):
        set_brightness(pi, PIN_B, i * 100 / steps)
        time.sleep(step_time)
    print(f"  Fading PIN_B (phys 38) down...")
    for i in range(steps, -1, -1):
        set_brightness(pi, PIN_B, i * 100 / steps)
        time.sleep(step_time)
    set_brightness(pi, PIN_B, 0)
    print()


def test_simultaneous_fade(pi: pigpio.pi, duration: float = 3.0):
    """Test 4: LED fades in then out."""
    print("=== Test 4: Fade In/Out ===")
    steps = 200
    step_time = duration / steps
    for i in range(steps + 1):
        set_brightness(pi, PIN_B, i * 100 / steps)
        time.sleep(step_time)
    for i in range(steps, -1, -1):
        set_brightness(pi, PIN_B, i * 100 / steps)
        time.sleep(step_time)
    set_brightness(pi, PIN_B, 0)
    print("  Done.\n")


def cleanup(pi: pigpio.pi):
    pi.set_PWM_dutycycle(PIN_B, 0)
    pi.set_mode(PIN_B, pigpio.INPUT)
    pi.stop()
    print("[OK] Cleanup complete. pigpio connection closed.")


def main():
    print("=" * 50)
    print(" LED PWM Test — pigpio")
    print(f" PIN_B: BCM {PIN_B}  (physical 38)")
    print("=" * 50)
    print()

    pi = connect_pigpio()
    setup_pins(pi)

    try:
        test_on_off(pi)
        test_brightness_steps(pi)
        test_fade(pi)
        test_simultaneous_fade(pi)
        print("All tests passed.")
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Ctrl+C detected.")
    finally:
        cleanup(pi)


if __name__ == "__main__":
    main()
