"""Integrating-sphere LED source: per-colour drive plus a current-hold loop.

The sphere's LEDs are driven through an MCP4728 quad 12-bit I2C DAC (0x60),
whose outputs set the constant-current LED driver modules.  Feedback comes from
an ADS1115 (0x4A) on the same bus: LED drive current across a 2.2 ohm sense
resistor on AIN0, and an NTC Wheatstone bridge on AIN2-AIN3.

This module adds two things the bench scripts never had: a named colour ->
channel mapping (test_LED_system.py addresses channels as bare integers 0-3),
and an optional PI loop that holds drive current constant against the LED's
forward-voltage drift as it warms up.

Hardware limitation, important for interpreting any result:

    There is ONE current-sense resistor and ONE thermistor for the whole
    board.  Closed-loop current control is therefore only meaningful when a
    single LED is driven at a time — which is how the goniometric scan must
    run anyway, since angular response has to be measured per wavelength
    (Experiment_Design/01_source_calibration.md, section 2 step 3).  With
    several LEDs on at once the loop holds *total* current, and the
    temperature reading is a single board temperature, not a per-LED junction
    temperature.

The loop has no thread of its own.  ``update()`` performs one iteration and
the caller drives it, so the same object serves both the soak script's 1 Hz
logging loop and the scan script's per-point dwell.

Usage:
    import smbus2
    bus = smbus2.SMBus(1)
    src = SphereLedSource(bus=bus)
    src.set_code(Color.BLUE, 2047)          # open loop
    src.hold_current(0.35)                  # engage the PI loop
    while ...:
        state = src.update()
    src.close()                             # all channels to 0
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum

from drivers.ads1115 import SENSE_RESISTOR_OHM, Ads1115
from drivers.mcp4728_driver import MAX_CODE, NUM_CHANNELS, MCP4728Driver

logger = logging.getLogger(__name__)

DEFAULT_LDAC_PIN = 20  # BCM numbering, physical pin 38


class Color(Enum):
    RED = "red"
    GREEN = "green"
    BLUE = "blue"


# MCP4728 channel index (0=A .. 3=D) per colour.  Confirm against the board
# wiring before trusting a scan; override with the channel_map argument.
DEFAULT_CHANNEL_MAP = {Color.RED: 0, Color.GREEN: 1, Color.BLUE: 2}


@dataclass
class LedState:
    """One sample of the source's commanded and measured state."""

    codes: dict[Color, int]
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
        channel_map: dict[Color, int] | None = None,
        dac: MCP4728Driver | None = None,
        ads1115: Ads1115 | None = None,
        pi=None,
        ldac_pin: int = DEFAULT_LDAC_PIN,
        use_ldac: bool = True,
        i2c_dev: str = "/dev/i2c-1",
        sense_resistor_ohm: float = SENSE_RESISTOR_OHM,
    ) -> None:
        self.channel_map = dict(channel_map or DEFAULT_CHANNEL_MAP)
        duplicates = len(set(self.channel_map.values())) != len(self.channel_map)
        if duplicates:
            raise ValueError(f"channel_map assigns one MCP4728 channel twice: {self.channel_map}")
        for color, channel in self.channel_map.items():
            if not 0 <= channel < NUM_CHANNELS:
                raise ValueError(f"{color.value} maps to channel {channel}, must be 0-3")

        self._sense_resistor_ohm = sense_resistor_ohm
        self._codes = [0] * NUM_CHANNELS
        self._target_current_a: float | None = None
        self._integral = 0.0
        self._loop_color: Color | None = None
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
    def codes(self) -> dict[Color, int]:
        return {color: self._codes[channel] for color, channel in self.channel_map.items()}

    def _flush(self) -> None:
        if not self._dac.set_codes(self._codes):
            raise OSError("MCP4728: Fast Write failed")

    def set_code(self, color: Color, code: int) -> None:
        """Set one colour's DAC code (0-4095), leaving the others untouched."""
        self._codes[self.channel_map[color]] = max(0, min(MAX_CODE, int(code)))
        self._flush()

    def set_codes(self, codes: dict[Color, int]) -> None:
        """Set several colours at once — one I2C transaction, so they step together."""
        for color, code in codes.items():
            self._codes[self.channel_map[color]] = max(0, min(MAX_CODE, int(code)))
        self._flush()

    def all_off(self) -> None:
        """Drive every channel to code 0 and disengage any current loop."""
        self._codes = [0] * NUM_CHANNELS
        self._target_current_a = None
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
        color: Color | None = None,
        kp: float | None = None,
        ki: float | None = None,
        deadband_a: float | None = None,
        max_code_step: int | None = None,
    ) -> None:
        """Engage the PI loop holding measured drive current at ``target_a``.

        ``color`` names the channel the loop actuates; it defaults to the only
        channel currently driven above zero.  With more than one LED lit the
        loop regulates total current — see the module docstring.
        """
        if color is None:
            lit = [c for c, code in self.codes.items() if code > 0]
            if len(lit) != 1:
                raise ValueError(
                    f"cannot infer which channel to actuate (lit channels: "
                    f"{[c.value for c in lit]}) — pass color= explicitly"
                )
            color = lit[0]
        if len([c for c, code in self.codes.items() if code > 0]) > 1:
            logger.warning(
                "SphereLedSource: more than one LED is lit — the current loop regulates "
                "TOTAL board current, not %s alone",
                color.value,
            )

        self._loop_color = color
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
            "SphereLedSource: holding %s at %.4f A (kp=%.1f ki=%.1f)",
            color.value,
            target_a,
            self._kp,
            self._ki,
        )

    def open_loop(self) -> None:
        """Disengage the current loop, leaving the DAC codes where they are."""
        self._target_current_a = None
        self._integral = 0.0

    def update(self) -> LedState:
        """Sample current and temperature; run one PI step if a target is set.

        A no-op on the DAC when open-loop, so the soak script can log
        identically in both modes and the two runs stay directly comparable.
        """
        current_a = self.read_current_a()
        temperature_c = self.read_bridge_temperature_c()

        settled = True
        if self._target_current_a is not None and self._loop_color is not None:
            error = self._target_current_a - current_a
            settled = abs(error) <= self._deadband_a
            if not settled:
                now = time.monotonic()
                dt = 0.0 if self._last_update_t is None else now - self._last_update_t
                self._last_update_t = now

                self._integral += error * dt
                delta = self._kp * error + self._ki * self._integral
                delta = max(-self._max_code_step, min(self._max_code_step, delta))

                channel = self.channel_map[self._loop_color]
                new_code = max(0, min(MAX_CODE, int(round(self._codes[channel] + delta))))
                # Anti-windup: stop integrating once the actuator is railed.
                if new_code in (0, MAX_CODE) and self._codes[channel] == new_code:
                    self._integral -= error * dt
                self._codes[channel] = new_code
                self._flush()

        return LedState(
            codes=self.codes,
            current_a=current_a,
            temperature_c=temperature_c,
            target_current_a=self._target_current_a,
            settled=settled,
        )

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        """Zero every channel, then release the DAC and any pigpio handle we own."""
        try:
            self.all_off()
        except Exception:
            logger.exception("SphereLedSource: failed to zero LED channels on close")
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
