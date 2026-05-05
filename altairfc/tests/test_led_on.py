#!/usr/bin/env python3
"""
LED full-on test — drives BCM 20 (physical pin 38) low to force LDD-700LS output on.
Press Ctrl+C to turn off and exit.
"""

import pigpio
import sys

PIN_B = 20

pi = pigpio.pi()
if not pi.connected:
    print("[ERROR] Cannot connect to pigpio daemon. Run: sudo pigpiod")
    sys.exit(1)

pi.set_mode(PIN_B, pigpio.OUTPUT)
pi.write(PIN_B, 0)
print(f"[OK] BCM {PIN_B} driven LOW — LED should be fully on. Press Ctrl+C to stop.")

try:
    input()
except KeyboardInterrupt:
    pass

pi.write(PIN_B, 1)
pi.set_mode(PIN_B, pigpio.INPUT)
pi.stop()
print("[OK] Done.")
