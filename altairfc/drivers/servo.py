from __future__ import annotations

import logging
import math

import numpy as np

logger = logging.getLogger(__name__)

SERVO_PIN = 26
SOURCE_MAX_DEG = 90.0   # source travel: 0° = straight down (nadir), 90° = horizontal
PULSE_NADIR_US = 500    # calibrated pulse width with the source pointing down
PULSE_SPAN_US = 2000    # pulse width change over SOURCE_MAX_DEG (500 -> 2500 us)


def nadir_angle_deg(pitch_err_rad: float) -> float:
    """Unclamped angle of the ground station from body-down (deg).

    pitch_err is the GS elevation below the body x-y plane (+90° = straight down,
    0° = on the body horizon), so the angle from nadir is 90° - pitch_err.
    """
    return 90.0 - math.degrees(pitch_err_rad)


class ServoPointer:
    """
    Controls the light source tilt via a pigpio-driven servo.

    Calibrated: 500 us points the source straight down (body +Z), 2500 us points
    it 90° away from down, horizontal and toward the body +X (nose) axis. The
    azimuth controller drives the GS into the x-z plane, so tilting by the angle
    from nadir points the source at the GS.
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
        angle = self.achieved_deflection_deg(pitch_err_rad)
        pulsewidth = PULSE_NADIR_US + (angle / SOURCE_MAX_DEG) * PULSE_SPAN_US
        self._pi.set_servo_pulsewidth(SERVO_PIN, int(pulsewidth))

    def achieved_deflection_deg(self, pitch_err_rad: float) -> float:
        """Return the clamped source angle from nadir that will actually be achieved (degrees)."""
        return float(np.clip(nadir_angle_deg(pitch_err_rad), 0.0, SOURCE_MAX_DEG))

    def disconnect(self) -> None:
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(SERVO_PIN, 0)
            self._pi.stop()
            self._pi = None
