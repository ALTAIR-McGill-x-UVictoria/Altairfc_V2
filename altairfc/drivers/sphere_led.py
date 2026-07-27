"""Integrating-sphere LED source: combined-drive control plus a current-hold loop.

The sphere's R/G/B LEDs are driven together as ONE unit — there is no per-colour
control. A single MCP4728 DAC channel (0x60) sets the drive level for all three
combined; there is no independent DAC channel, switch, or driver per colour.
Feedback comes from an ADS1115 (0x4A) on the same bus: LED drive current across
a 2.2 ohm sense resistor on AIN0, and an NTC Wheatstone bridge on AIN2-AIN3 —
both are properties of the board as a whole, not of any one colour.

Practical consequences worth being explicit about:

  * Brightness/current can be commanded and held constant, but the R:G:B mix
    cannot be adjusted or isolated. "Blue only" is not physically achievable.
  * A goniometric scan with this source measures the angular response of the
    COMBINED spectrum, not a per-wavelength I(theta, phi, lambda) curve. That
    conflicts with Experiment_Design/01_source_calibration.md section 2 step 3,
    which requires each wavelength measured independently. Getting a true
    per-wavelength curve needs either a hardware change (independently
    switchable LED drivers) or a spectrally-resolving detector at the
    goniometer stage — neither exists yet. See the project memory
    ``goniometer-measurement-campaign`` for the open decision.
  * The current-hold loop and the thermistor reading are therefore both
    meaningful as-is (there is only ever one thing being measured), unlike an
    earlier version of this module that assumed independent per-colour control.

The loop has no thread of its own. ``update()`` performs one iteration and the
caller drives it, so the same object serves both the soak script's 1 Hz logging
loop and the scan script's per-point dwell.

Usage:
    import smbus2
    bus = smbus2.SMBus(1)
    src = SphereLedSource(bus=bus)
    src.set_code(2047)             # open loop
    src.hold_current(0.35)         # engage the PI loop
    while ...:
        state = src.update()
    src.close()                    # drive to 0
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from drivers.ads1115 import SENSE_RESISTOR_OHM, Ads1115
from drivers.mcp4728_driver import MAX_CODE, NUM_CHANNELS, MCP4728Driver

logger = logging.getLogger(__name__)

DEFAULT_LDAC_PIN = 20  # BCM numbering, physical pin 38

# Which MCP4728 channel drives the sphere's combined LEDs. Confirm against the
# board wiring; override with the channel argument if it differs.
DEFAULT_CHANNEL = 0


@dataclass
class LedState:
    """One sample of the source's commanded and measured state."""

    code: int
    current_a: float
    temperature_c: float
    target_current_a: float | None
    settled: bool


class SphereLedSource:
    """Own the MCP4728, its LDAC line, and the ADS1115 feedback channels."""

    def __init__(
        self,
        *,
        bus,
        channel: int = DEFAULT_CHANNEL,
        dac: MCP4728Driver | None = None,
        ads1115: Ads1115 | None = None,
        pi=None,
        ldac_pin: int = DEFAULT_LDAC_PIN,
        use_ldac: bool = True,
        i2c_dev: str = "/dev/i2c-1",
        sense_resistor_ohm: float = SENSE_RESISTOR_OHM,
    ) -> None:
        if not 0 <= channel < NUM_CHANNELS:
            raise ValueError(f"channel must be 0-3, got {channel}")
        self.channel = channel

        self._sense_resistor_ohm = sense_resistor_ohm
        self._codes = [0] * NUM_CHANNELS
        self._target_current_a: float | None = None
        self._integral = 0.0
        self._loop_engaged = False
        self._last_update_t: float | None = None

        # PI gains, in DAC codes per amp.  Defaults are deliberately gentle;
        # tune them from the open-loop soak data before trusting a long run.
        self._kp = 2000.0
        self._ki = 400.0
        self._deadband_a = 0.001
        self._max_code_step = 8

        self._dac = dac if dac is not None else MCP4728Driver(i2c_dev)
        self._ads = ads1115 if ads1115 is not None else Ads1115(bus)

        self._pi = pi
        self._owns_pi = False
        self._ldac_pin = ldac_pin
        if use_ldac:
            self._open_ldac(pi)

        # One Multi-Write forces Vref=Vdd and gain=1x on every channel so VOUT
        # tracks Vdd; every later update is a cheaper Fast Write.
        if not self._dac.set_vdd_reference(self._codes):
            raise OSError("MCP4728: initial Multi-Write failed — check I2C bus and address 0x60")

    def _open_ldac(self, pi) -> None:
        """Hold LDAC low so DAC writes reach VOUT immediately."""
        if pi is None:
            import pigpio

            pi = pigpio.pi()
            if not pi.connected:
                raise RuntimeError("Cannot connect to pigpio daemon. Run: sudo pigpiod")
            self._owns_pi = True
        import pigpio

        pi.set_mode(self._ldac_pin, pigpio.OUTPUT)
        pi.write(self._ldac_pin, 0)
        self._pi = pi
        logger.info("SphereLedSource: LDAC (BCM %d) held low", self._ldac_pin)

    # -- open-loop drive ---------------------------------------------------

    @property
    def code(self) -> int:
        return self._codes[self.channel]

    def _flush(self) -> None:
        if not self._dac.set_codes(self._codes):
            raise OSError("MCP4728: Fast Write failed")

    def set_code(self, code: int) -> None:
        """Set the combined LED drive to a DAC code (0-4095)."""
        self._codes[self.channel] = max(0, min(MAX_CODE, int(code)))
        self._flush()

    def all_off(self) -> None:
        """Drive to code 0 and disengage any current loop."""
        self._codes[self.channel] = 0
        self._target_current_a = None
        self._loop_engaged = False
        self._integral = 0.0
        self._flush()

    # -- feedback ----------------------------------------------------------

    def read_current_a(self) -> float:
        return self._ads.read_current_a(sense_resistor_ohm=self._sense_resistor_ohm)

    def read_bridge_temperature_c(self) -> float:
        return self._ads.read_bridge_temperature_c()

    # -- closed loop -------------------------------------------------------

    def hold_current(
        self,
        target_a: float,
        *,
        kp: float | None = None,
        ki: float | None = None,
        deadband_a: float | None = None,
        max_code_step: int | None = None,
    ) -> None:
        """Engage the PI loop holding measured drive current at ``target_a``."""
        self._loop_engaged = True
        self._target_current_a = float(target_a)
        self._integral = 0.0
        self._last_update_t = None
        if kp is not None:
            self._kp = kp
        if ki is not None:
            self._ki = ki
        if deadband_a is not None:
            self._deadband_a = deadband_a
        if max_code_step is not None:
            self._max_code_step = max_code_step
        logger.info(
            "SphereLedSource: holding %.4f A (kp=%.1f ki=%.1f)", target_a, self._kp, self._ki
        )

    def open_loop(self) -> None:
        """Disengage the current loop, leaving the DAC code where it is."""
        self._target_current_a = None
        self._loop_engaged = False
        self._integral = 0.0

    def update(self) -> LedState:
        """Sample current and temperature; run one PI step if a target is set.

        A no-op on the DAC when open-loop, so the soak script can log
        identically in both modes and the two runs stay directly comparable.
        """
        current_a = self.read_current_a()
        temperature_c = self.read_bridge_temperature_c()

        settled = True
        if self._loop_engaged and self._target_current_a is not None:
            error = self._target_current_a - current_a
            settled = abs(error) <= self._deadband_a
            if not settled:
                now = time.monotonic()
                dt = 0.0 if self._last_update_t is None else now - self._last_update_t
                self._last_update_t = now

                self._integral += error * dt
                delta = self._kp * error + self._ki * self._integral
                delta = max(-self._max_code_step, min(self._max_code_step, delta))

                new_code = max(0, min(MAX_CODE, int(round(self.code + delta))))
                # Anti-windup: stop integrating once the actuator is railed.
                if new_code in (0, MAX_CODE) and self.code == new_code:
                    self._integral -= error * dt
                self._codes[self.channel] = new_code
                self._flush()

        return LedState(
            code=self.code,
            current_a=current_a,
            temperature_c=temperature_c,
            target_current_a=self._target_current_a,
            settled=settled,
        )

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        """Drive to 0, then release the DAC and any pigpio handle we own."""
        try:
            self.all_off()
        except Exception:
            logger.exception("SphereLedSource: failed to zero the LED channel on close")
        try:
            self._dac.close()
        except Exception:
            logger.exception("SphereLedSource: failed to close MCP4728")
        if self._pi is not None and self._owns_pi:
            self._pi.stop()
            self._pi = None

    def __enter__(self) -> SphereLedSource:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
