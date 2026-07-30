"""Integrating-sphere LED source: single-channel drive control plus a current-hold loop.

The sphere's R/G/B LEDs are driven together as ONE combined optical output —
there is no per-colour control, no independent DAC channel/switch/driver per
colour. This source uses one MCP4728 DAC channel (0x60):

    HIGH_POWER_CHANNEL = 0  (MCP4728 channel A)

MCP4728 channel B (index 1) is no longer a second sphere drive stage — it is
now wired to a physically separate, non-sphere external LED used to let
ground observers visually spot the payload (see drivers/beacon_led.py). The
sphere source and that beacon must never be energized at the same time; both
channels now share a single drivers/led_board.LedBoard, which owns the MCP4728
Multi-Write and enforces that interlock.

Feedback comes from an ADS1115 (0x4A) on the same bus: this channel's drive
current across its own 2.2 ohm sense resistor (AIN0), and an NTC Wheatstone
bridge on AIN2-AIN3, both properties of the sphere source board.

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

DAC codes are hard-limited to MAX_SAFE_CODE by this driver, unconditionally,
regardless of what a caller requests — see MAX_SAFE_CODE below for why.

The loop has no thread of its own. ``update()`` performs one iteration and the
caller drives it, so the same object serves both the soak script's 1 Hz logging
loop and the scan script's per-point dwell.

Usage:
    import smbus2
    bus = smbus2.SMBus(1)
    src = SphereLedSource(bus=bus)                      # owns its own LedBoard
    src.set_code(1400)             # open loop, clamped to MAX_SAFE_CODE regardless
    src.hold_current(0.35)         # engage the PI loop
    while ...:
        state = src.update()
    src.close()                    # drive to 0

To share the MCP4728 with a BeaconChannel (flight configuration), construct a
LedBoard once and pass it to both:
    board = LedBoard(bus=bus)
    src = SphereLedSource(board=board)
    beacon = BeaconChannel(board=board)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from drivers.ads1115 import SENSE_RESISTOR_OHM, Ads1115
from drivers.led_board import DEFAULT_LDAC_PIN, SPHERE_CHANNEL, LedBoard
from drivers.mcp4728_driver import MAX_CODE

logger = logging.getLogger(__name__)

# The one drive channel this source uses. Channel 1 (MCP4728 B) now belongs to
# the external spotter beacon — see drivers/beacon_led.py.
HIGH_POWER_CHANNEL = SPHERE_CHANNEL  # MCP4728 channel A
VALID_CHANNELS = (HIGH_POWER_CHANNEL,)

DEFAULT_CHANNEL = HIGH_POWER_CHANNEL

# Hard ceiling on the DAC code, enforced by this driver regardless of what a
# caller requests (--code, hold_current's PI output, anything). This is a
# safety limit, not a suggestion — do not raise it without confirming what it
# is actually protecting against.
MAX_SAFE_CODE = 1400
assert MAX_SAFE_CODE <= MAX_CODE, "MAX_SAFE_CODE must not exceed the DAC's own 12-bit range"


@dataclass
class LedState:
    """One sample of the source's commanded and measured state."""

    code: int
    current_a: float
    temperature_c: float
    target_current_a: float | None
    settled: bool


class SphereLedSource:
    """Drive the sphere source channel through a shared LedBoard."""

    def __init__(
        self,
        *,
        bus=None,
        channel: int = DEFAULT_CHANNEL,
        board: LedBoard | None = None,
        dac=None,
        ads1115: Ads1115 | None = None,
        pi=None,
        ldac_pin: int = DEFAULT_LDAC_PIN,
        use_ldac: bool = True,
        i2c_dev: str = "/dev/i2c-1",
        sense_resistor_ohm: float = SENSE_RESISTOR_OHM,
    ) -> None:
        if channel not in VALID_CHANNELS:
            raise ValueError(f"channel must be HIGH_POWER_CHANNEL ({HIGH_POWER_CHANNEL}), got {channel}")
        self.channel = channel

        self._sense_resistor_ohm = sense_resistor_ohm
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

        if board is not None:
            self._board = board
            self._owns_board = False
        else:
            self._board = LedBoard(
                bus=bus, dac=dac, ads1115=ads1115, pi=pi,
                ldac_pin=ldac_pin, use_ldac=use_ldac, i2c_dev=i2c_dev,
            )
            self._owns_board = True

    # -- open-loop drive ---------------------------------------------------

    @property
    def code(self) -> int:
        return self._board.code(self.channel)

    def set_code(self, code: int) -> None:
        """Set this channel's drive to a DAC code, hard-clamped to MAX_SAFE_CODE."""
        self._board.write_channel(self.channel, max(0, min(MAX_SAFE_CODE, int(code))))

    def all_off(self) -> None:
        """Drive to code 0 and disengage any current loop."""
        self._target_current_a = None
        self._loop_engaged = False
        self._integral = 0.0
        self._board.all_off(self.channel)

    # -- feedback ----------------------------------------------------------

    def read_current_a(self) -> float:
        return self._board.ads.read_current_a(
            channel=self.channel, sense_resistor_ohm=self._sense_resistor_ohm
        )

    def read_bridge_temperature_c(self) -> float:
        return self._board.ads.read_bridge_temperature_c()

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

                new_code = max(0, min(MAX_SAFE_CODE, int(round(self.code + delta))))
                # Anti-windup: stop integrating once the actuator is railed.
                # MAX_SAFE_CODE, not the DAC's full range, is the true ceiling
                # this loop can reach.
                if new_code in (0, MAX_SAFE_CODE) and self.code == new_code:
                    self._integral -= error * dt
                self._board.write_channel(self.channel, new_code)

        return LedState(
            code=self.code,
            current_a=current_a,
            temperature_c=temperature_c,
            target_current_a=self._target_current_a,
            settled=settled,
        )

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        """Drive to 0. Also releases the shared LedBoard, but only if this
        instance created it (a board passed in explicitly outlives this
        object — e.g. a BeaconChannel sharing the same board)."""
        try:
            self.all_off()
        except Exception:
            logger.exception("SphereLedSource: failed to zero the LED channel on close")
        if self._owns_board:
            self._board.close()

    def __enter__(self) -> SphereLedSource:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
