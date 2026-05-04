from __future__ import annotations

import logging
import math

import numpy as np

logger = logging.getLogger(__name__)

SERVO_PIN = 26
SOURCE_LIMIT_DEG = 22.5   # physical ±range of source from nadir (servo ±90° at 4:1 ratio)
GEAR_RATIO = 4.0          # servo moves GEAR_RATIO × more than the source


class ServoPointer:
    """
    Controls the nadir-pointing light source via a pigpio-driven servo.

    The servo at 90° points the source straight down (nadir). The 4:1 gear
    ratio means the source deflects by GEAR_RATIO° for every 1° of servo
    travel, giving a physical source range of ±22.5° from nadir.
    """

    def __init__(self) -> None:
        self._pi = None

    def connect(self) -> bool:
        try:
            import pigpio
            pi = pigpio.pi()
            if not pi.connected:
                logger.error("Failed to connect to pigpio daemon — servo disabled")
                return False
            self._pi = pi
            logger.info("pigpio connected; servo on GPIO %d", SERVO_PIN)
            return True
        except Exception as e:
            logger.error("pigpio init failed: %s — servo disabled", e)
            return False

    def set_pitch_error(self, pitch_err_rad: float) -> None:
        """Command the servo based on pitch error to the ground station (radians)."""
        if self._pi is None:
            return
        source_deflection_deg = float(np.clip(-math.degrees(pitch_err_rad), -SOURCE_LIMIT_DEG, SOURCE_LIMIT_DEG))
        servo_angle_deg = 90.0 + source_deflection_deg * GEAR_RATIO
        pulsewidth = 500 + (servo_angle_deg / 180.0) * 2000
        self._pi.set_servo_pulsewidth(SERVO_PIN, int(pulsewidth))

    def achieved_deflection_deg(self, pitch_err_rad: float) -> float:
        """Return the clamped source deflection that will actually be achieved (degrees)."""
        return float(np.clip(-math.degrees(pitch_err_rad), -SOURCE_LIMIT_DEG, SOURCE_LIMIT_DEG))

    def disconnect(self) -> None:
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(SERVO_PIN, 0)
            self._pi.stop()
            self._pi = None
