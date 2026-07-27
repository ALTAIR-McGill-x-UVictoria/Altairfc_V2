"""Two-axis servo stage for the integrating-sphere goniometer.

The jig rotates the sphere about its exit port in two axes while a photodiode
stays fixed at range, sweeping one hemisphere of emission directions:

    polar   theta   angle from the exit port's normal, nominally +/-90 deg
    azimuth phi     rotation about that normal, nominally 0..180 deg

Angle convention matters and is not arbitrary: theta must be defined
identically to ``calculate_emission_angle()`` in the analysis pipeline (angle
from the source's local nadir to the line of sight), because the measured
I(theta, phi, lambda) curve is looked up by that same angle in flight.  See
ALTAIR-analysis/Experiment_Design/01_source_calibration.md, "Angle convention".

Both axes are hobby servos driven by pigpio software PWM (the Pi 4B hardware
PWM pins are unavailable on this board layout).  Servo travel is mapped with
the pulsewidth convention used everywhere else in this repo,
``500 + (servo_deg / 180) * 2000`` microseconds, and every move is slew-rate
limited because the jig carries the sphere.

Usage:
    stage = GoniometerStage(POLAR_DEFAULT, AZIMUTH_DEFAULT)
    if stage.connect():
        stage.move_to(polar_deg=-45.0, azimuth_deg=0.0)
        ...
        stage.park()
"""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path

logger = logging.getLogger(__name__)

STEP_PERIOD_S = 0.02  # 50 Hz slew update rate, matching tests/test_two_servos.py

# Defaults follow tests/test_two_servos.py, the bench script this stage
# replaces.  Note BCM 16 is also the buzzer (drivers/buzzer.py) and BCM 26 is
# also ServoPointer (drivers/servo.py): harmless for a standalone lab tool, but
# do not run the flight stack at the same time as a scan.
DEFAULT_POLAR_PIN = 26
DEFAULT_AZIMUTH_PIN = 16

DEFAULT_CALIBRATION_PATH = Path(__file__).resolve().parent.parent / "goniometer_home.json"


@dataclass(frozen=True)
class AxisCalibration:
    """Mechanical mapping from a stage angle to a servo command.

    ``center_deg`` is the stage angle that sits at the servo's mechanical
    centre (90 deg) — 0 for a polar axis spanning +/-90, but 90 for an azimuth
    axis spanning 0..180, since both must fit the servo's single 0-180 travel.
    ``gear_ratio`` is degrees of servo travel per degree of stage motion, 1.0
    for direct drive.  ``invert`` flips the sense of positive stage angles.
    """

    pin: int
    center_deg: float = 0.0
    gear_ratio: float = 1.0
    invert: bool = False
    min_deg: float = -90.0
    max_deg: float = 90.0

    def servo_deg(self, angle_deg: float) -> float:
        """Map a stage angle to a servo angle.

        Raises ValueError when the angle falls outside the axis' travel rather
        than clamping: a scan that quietly measured a different angle than the
        one it recorded would corrupt the curve in a way no later check catches.
        """
        if not self.min_deg <= angle_deg <= self.max_deg:
            raise ValueError(
                f"angle {angle_deg:.2f} deg is outside axis travel "
                f"[{self.min_deg:.2f}, {self.max_deg:.2f}]"
            )
        deflection = (angle_deg - self.center_deg) * self.gear_ratio
        if self.invert:
            deflection = -deflection
        servo = 90.0 + deflection
        if not 0.0 <= servo <= 180.0:
            raise ValueError(
                f"stage angle {angle_deg:.2f} deg maps to servo angle "
                f"{servo:.2f} deg, outside 0-180 — check center_deg/gear_ratio"
            )
        return servo

    def stage_deg(self, servo_deg: float) -> float:
        """Inverse of :meth:`servo_deg` — the stage angle at a given servo angle."""
        deflection = servo_deg - 90.0
        if self.invert:
            deflection = -deflection
        return self.center_deg + deflection / self.gear_ratio

    def reachable_range(self) -> tuple[float, float]:
        """Stage-angle interval actually reachable: [min_deg, max_deg] intersected
        with what the servo's 0-180 deg travel can reach given this calibration.

        A nonzero offset between the configured centre and the true mechanical
        centre (the usual result of homing) can make part of the configured
        [min_deg, max_deg] unreachable — this never raises, unlike
        :meth:`servo_deg`, so it is safe to use for reporting that to an
        operator rather than crashing.
        """
        edge_a = self.stage_deg(0.0)
        edge_b = self.stage_deg(180.0)
        lo, hi = (edge_a, edge_b) if edge_a <= edge_b else (edge_b, edge_a)
        return (max(self.min_deg, lo), min(self.max_deg, hi))

    def recentered(self, servo_deg: float, stage_deg: float) -> AxisCalibration:
        """Return a copy whose centre is set by an observed physical position.

        Used by homing: jog the servo until the stage is visibly at a known
        reference angle, then record that pairing.
        """
        deflection = servo_deg - 90.0
        if self.invert:
            deflection = -deflection
        return replace(self, center_deg=stage_deg - deflection / self.gear_ratio)


POLAR_DEFAULT = AxisCalibration(
    pin=DEFAULT_POLAR_PIN, center_deg=0.0, min_deg=-90.0, max_deg=90.0
)
AZIMUTH_DEFAULT = AxisCalibration(
    pin=DEFAULT_AZIMUTH_PIN, center_deg=90.0, min_deg=0.0, max_deg=180.0
)


def servo_deg_to_pulsewidth(servo_deg: float) -> float:
    """Repo-wide convention: 0-180 deg maps onto 500-2500 us."""
    servo_deg = max(0.0, min(180.0, servo_deg))
    return 500.0 + (servo_deg / 180.0) * 2000.0


def save_calibration(
    polar: AxisCalibration,
    azimuth: AxisCalibration,
    path: Path = DEFAULT_CALIBRATION_PATH,
) -> None:
    """Persist axis calibration so a one-time mechanical homing survives sessions."""
    path.write_text(json.dumps({"polar": asdict(polar), "azimuth": asdict(azimuth)}, indent=2))
    logger.info("GoniometerStage: wrote calibration to %s", path)


def load_calibration(
    path: Path = DEFAULT_CALIBRATION_PATH,
) -> tuple[AxisCalibration, AxisCalibration]:
    """Load a saved calibration, falling back to the direct-drive defaults."""
    if not path.exists():
        logger.info("GoniometerStage: no calibration at %s — using defaults", path)
        return POLAR_DEFAULT, AZIMUTH_DEFAULT
    data = json.loads(path.read_text())
    return AxisCalibration(**data["polar"]), AxisCalibration(**data["azimuth"])


class GoniometerStage:
    """Two servo axes moved together, slew-limited, with a settle dwell.

    Follows the ServoPointer conventions in drivers/servo.py: ``connect()``
    returns False rather than raising when pigpio is unavailable, and shutdown
    releases the servos by writing a zero pulsewidth.

    A pigpio handle may be injected so the LED driver's LDAC pin and both
    servos share one client instead of opening three.
    """

    def __init__(
        self,
        polar: AxisCalibration = POLAR_DEFAULT,
        azimuth: AxisCalibration = AZIMUTH_DEFAULT,
        *,
        pi=None,
        slew_rate_deg_s: float = 30.0,
        settle_s: float = 1.0,
    ) -> None:
        self.polar = polar
        self.azimuth = azimuth
        self.slew_rate_deg_s = slew_rate_deg_s
        self.settle_s = settle_s
        self._pi = pi
        self._owns_pi = pi is None
        self._current: tuple[float, float] | None = None

    @property
    def current_angles(self) -> tuple[float, float] | None:
        """Last commanded (polar, azimuth) in stage degrees, or None before the first move."""
        return self._current

    def connect(self) -> bool:
        if self._pi is not None:
            logger.info(
                "GoniometerStage: using injected pigpio handle; polar=BCM %d azimuth=BCM %d",
                self.polar.pin,
                self.azimuth.pin,
            )
            return True
        try:
            import pigpio

            pi = pigpio.pi()
            if not pi.connected:
                logger.error(
                    "Failed to connect to pigpio daemon (run: sudo pigpiod) — stage disabled"
                )
                return False
            self._pi = pi
            logger.info(
                "pigpio connected; goniometer polar=BCM %d azimuth=BCM %d",
                self.polar.pin,
                self.azimuth.pin,
            )
            return True
        except Exception as e:
            logger.error("pigpio init failed: %s — stage disabled", e)
            return False

    def _write(self, pin: int, servo_deg: float) -> None:
        self._pi.set_servo_pulsewidth(pin, int(servo_deg_to_pulsewidth(servo_deg)))

    def move_to(
        self,
        polar_deg: float,
        azimuth_deg: float,
        *,
        settle: bool = True,
    ) -> tuple[float, float]:
        """Slew both axes to a stage position and dwell for ``settle_s``.

        Blocks until the move completes.  Both axes step together at
        STEP_PERIOD_S so the total move takes as long as the slower axis rather
        than the sum of the two.
        """
        if self._pi is None:
            raise RuntimeError("GoniometerStage.connect() must succeed before move_to()")

        target_polar_servo = self.polar.servo_deg(polar_deg)
        target_azimuth_servo = self.azimuth.servo_deg(azimuth_deg)

        if self._current is None or not self.slew_rate_deg_s:
            self._write(self.polar.pin, target_polar_servo)
            self._write(self.azimuth.pin, target_azimuth_servo)
        else:
            start_polar_servo = self.polar.servo_deg(self._current[0])
            start_azimuth_servo = self.azimuth.servo_deg(self._current[1])
            step = self.slew_rate_deg_s * STEP_PERIOD_S
            span = max(
                abs(target_polar_servo - start_polar_servo),
                abs(target_azimuth_servo - start_azimuth_servo),
            )
            steps = max(1, math.ceil(span / step))
            for i in range(1, steps + 1):
                frac = i / steps
                self._write(
                    self.polar.pin,
                    start_polar_servo + (target_polar_servo - start_polar_servo) * frac,
                )
                self._write(
                    self.azimuth.pin,
                    start_azimuth_servo + (target_azimuth_servo - start_azimuth_servo) * frac,
                )
                time.sleep(STEP_PERIOD_S)

        self._current = (polar_deg, azimuth_deg)
        if settle and self.settle_s > 0:
            time.sleep(self.settle_s)
        return self._current

    def home(self) -> tuple[float, float]:
        """Move both axes to mid-travel, the mechanically safe reference position."""
        return self.move_to(self.polar.center_deg, self.azimuth.center_deg)

    def jog_servo(self, axis: str, servo_deg: float) -> None:
        """Drive one axis directly in servo degrees, bypassing the stage mapping.

        Only for homing, where the mapping is precisely what is being
        established. Invalidates the tracked position, so the next move_to()
        jumps rather than slewing.
        """
        if self._pi is None:
            raise RuntimeError("GoniometerStage.connect() must succeed before jog_servo()")
        cal = self.polar if axis == "polar" else self.azimuth
        self._write(cal.pin, max(0.0, min(180.0, servo_deg)))
        self._current = None

    def park(self) -> None:
        """Release both servos and, if this stage opened pigpio, close it."""
        if self._pi is None:
            return
        for pin in (self.polar.pin, self.azimuth.pin):
            try:
                self._pi.set_servo_pulsewidth(pin, 0)
            except Exception:
                logger.exception("GoniometerStage: failed to release BCM %d", pin)
        if self._owns_pi:
            self._pi.stop()
            self._pi = None
        self._current = None

    def __enter__(self) -> GoniometerStage:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.park()
