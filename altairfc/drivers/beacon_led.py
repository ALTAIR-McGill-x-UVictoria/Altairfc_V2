"""External spotter beacon: a non-sphere, high-power LED for visual tracking.

Lets ground observers visually spot the payload when the sphere source is too
dim to see. Two MCP4728 channels are involved (0x60), with different jobs:

    BEACON_CHANNEL = 1  brightness setpoint, driven by a current-hold PI loop
                         (mirrors drivers/sphere_led.py's) while the beacon is
                         lit; on its own this emits no light.
    RELAY_CHANNEL  = 3  the actual ON/OFF switch -- a 2N2222A-gated relay that
                         connects BEACON_CHANNEL's LED supply, added as a hard
                         fix for RF-coupled gate disturbance (see
                         tests/test_LED_system.py). Wired NO (normally-open),
                         confirmed on hardware 2026-08-03: code 0 (de-energized)
                         leaves the contact open -> beacon OFF; MAX_CODE
                         (energized) closes it -> beacon ON. See
                         led_board.RELAY_CODE_BEACON_ON/_OFF.

Flashing the beacon is therefore: set_target_current() once, then on()/off()
to toggle the relay for each flash -- not repeated set_target_current() calls.

on()/off() never let the relay's contacts make or break while current is
flowing through the LED. Closing the relay while channel 1 is already driven
hot-switches the contacts onto a live circuit -- the same family of transient
(arcing, RF/EMI injection) this relay was added to eliminate in the first
place. Instead, on() forces channel 1 to 0, energizes the relay onto a
currentless circuit, waits for the contact to physically settle, then ramps
the drive code up to its last-known operating point (BEACON_STARTUP_CODE the
first time, so a flash doesn't waste most of its on-time crawling up from a
dark 0) before handing off to the PI loop; off() ramps channel 1 back down to
0 before de-energizing the relay, so the break happens currentless too.

Current sense is on ADS1115 AIN1, across its own 1.5 ohm (nominal) sense
resistor -- same nominal value as the sphere's 1.5 ohm on AIN0, but a
physically separate resistor (see tests/test_LED_system.py's
CURRENT_SENSE_RESISTOR_OHM and config/settings.toml's
[tasks.lighting].beacon_sense_resistor_ohm); do not reuse
drivers.ads1115.SENSE_RESISTOR_OHM (that is the sphere's value) as the
beacon's default.

Closed-loop constant current, mirroring drivers/sphere_led.py's PI loop: the
beacon's job is visual strobing, not calibrated optical output, but the same
current-hold loop keeps its brightness consistent across temperature and
supply drift while lit. The caller (tasks/lighting_task.py) must call
update() once per tick while the beacon is lit, same as SphereLedSource.
Must share the same LedBoard as SphereLedSource, which is the single writer
to this MCP4728 -- never open a second MCP4728Driver against this chip. The
sphere and beacon can be energized simultaneously (no electrical constraint);
keeping the beacon off during actual sphere calibration exposures is a
scheduling concern handled by tasks/lighting_task.py's beacon_windows, not
something this driver enforces.

Usage:
    board = LedBoard(bus=bus)
    beacon = BeaconChannel(board=board)
    beacon.set_target_current(0.35)   # PI target, engaged once lit
    beacon.on()                       # energizes the relay, ramps to last code, engages PI loop
    beacon.update()                   # call once per tick while lit
    beacon.off()                      # ramps to 0, disengages PI loop, de-energizes the relay
    beacon.all_off()                  # zeros brightness and de-energizes the relay -- beacon dark
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from drivers.led_board import (
    BEACON_CHANNEL,
    RELAY_CHANNEL,
    RELAY_CODE_BEACON_OFF,
    RELAY_CODE_BEACON_ON,
    LedBoard,
)

logger = logging.getLogger(__name__)

# Hard ceiling on the beacon's brightness code, enforced by this driver
# regardless of what a caller requests. This is a different physical LED than
# the sphere source (MAX_SAFE_CODE=2100 there) — measured/rated limit for this
# LED. Must stay >= 2100: the new LED driver hardware (both channels) requires
# a DAC output of at least 2100 to reach its operating point.
BEACON_MAX_SAFE_CODE = 3000

# Starting code for on()'s ramp the first time the PI loop engages (before any
# prior flash has taught it a settled operating point). The new LED driver
# hardware needs at least 2100 to produce any output at all, so ramping from a
# cold 0 would leave the beacon dark for most of a short beacon_windows flash
# while the PI loop's max_code_step-limited climb crawls up from zero. Later
# flashes reuse the last settled code instead (see BeaconChannel._last_code).
BEACON_STARTUP_CODE = 2100

# AIN1's sense resistor for the beacon's drive current — a separate physical
# resistor from the sphere's (drivers.ads1115.SENSE_RESISTOR_OHM, channel 0
# only), though both are nominally 1.5 ohm. See config/settings.toml
# [tasks.lighting].beacon_sense_resistor_ohm for the flight-configurable value.
BEACON_SENSE_RESISTOR_OHM = 1.5

# Time given to the relay's contact to physically close before any brightness
# current is asked to flow through it. RY5W-K's typical operate time is a few
# ms; this is a margin on top of that, not a measured value -- if hardware
# testing shows the beacon still flickers right at turn-on, increase this
# first.
RELAY_SETTLE_S = 0.008

# Total time (and step count) to move brightness between 0 and its setpoint
# once the relay contact is settled. Deliberately short -- "ramp" here is
# about softening the current step at the now-closed contact, not a visible
# fade; it must stay short relative to the beacon's flash on-time.
BRIGHTNESS_RAMP_S = 0.008
BRIGHTNESS_RAMP_STEPS = 4


@dataclass
class BeaconState:
    """One sample of the beacon's commanded and measured state."""

    code: int
    current_a: float
    target_current_a: float | None
    settled: bool


class BeaconChannel:
    """Constant-current drive for the external spotter LED: brightness (ch 1,
    PI current-hold loop while lit) + relay on/off (ch 3)."""

    def __init__(self, *, board: LedBoard, sense_resistor_ohm: float = BEACON_SENSE_RESISTOR_OHM) -> None:
        self._board = board
        self._sense_resistor_ohm = sense_resistor_ohm
        # The last-driven code survives independently of channel 1's live DAC
        # code, because on()/off() deliberately drive channel 1 to 0 around each
        # relay transition (see module docstring) -- self.code alone can't be
        # used to recover "what code should this come back on at" before the PI
        # loop re-settles. Starts at BEACON_STARTUP_CODE (not 0) so the very
        # first flash ramps straight to near the driver's required operating
        # point instead of crawling up from a dark start -- see its docstring.
        self._last_code = BEACON_STARTUP_CODE

        self._target_current_a: float | None = None
        self._loop_engaged = False
        self._integral = 0.0
        self._last_update_t: float | None = None

        # PI gains, in DAC codes per amp — same defaults as SphereLedSource;
        # tune from soak data before trusting a long run.
        self._kp = 2000.0
        self._ki = 400.0
        self._deadband_a = 0.001
        self._max_code_step = 8

    @property
    def code(self) -> int:
        return self._board.code(BEACON_CHANNEL)

    @property
    def lit(self) -> bool:
        """Whether the beacon LED is actually emitting light right now."""
        return self._board.code(RELAY_CHANNEL) == RELAY_CODE_BEACON_ON

    def set_target_current(
        self,
        target_a: float,
        *,
        kp: float | None = None,
        ki: float | None = None,
        deadband_a: float | None = None,
        max_code_step: int | None = None,
    ) -> None:
        """Set the PI loop's target current, clamped nowhere (the loop's code output is
        clamped to BEACON_MAX_SAFE_CODE regardless). Takes effect once on() engages the loop."""
        self._target_current_a = float(target_a)
        if kp is not None:
            self._kp = kp
        if ki is not None:
            self._ki = ki
        if deadband_a is not None:
            self._deadband_a = deadband_a
        if max_code_step is not None:
            self._max_code_step = max_code_step

    def on(self) -> None:
        """Turn the beacon LED on and engage the current-hold PI loop.

        Energizes the relay (channel 3) onto a currentless circuit, then ramps
        channel 1 up to its last-known drive code before update() takes over —
        see module docstring for why the ramp exists.
        """
        self._board.write_channel(BEACON_CHANNEL, 0)
        self._board.write_channel(RELAY_CHANNEL, RELAY_CODE_BEACON_ON)
        time.sleep(RELAY_SETTLE_S)
        self._ramp_brightness(0, self._last_code)
        self._integral = 0.0
        self._last_update_t = None
        self._loop_engaged = self._target_current_a is not None

    def off(self) -> None:
        """Turn the beacon LED off and disengage the current-hold PI loop.

        Ramps channel 1 back down to 0 before de-energizing the relay (channel
        3), so the contact breaks currentless too — see module docstring.
        """
        self._loop_engaged = False
        self._ramp_brightness(self.code, 0)
        self._board.write_channel(RELAY_CHANNEL, RELAY_CODE_BEACON_OFF)

    def update(self) -> BeaconState:
        """Sample current; run one PI step if lit and a target is set. Call once per tick
        while lit, same as SphereLedSource.update()."""
        current_a = self.read_current_a()

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

                new_code = max(0, min(BEACON_MAX_SAFE_CODE, int(round(self.code + delta))))
                self._board.write_channel(BEACON_CHANNEL, new_code)
                self._last_code = new_code

        return BeaconState(
            code=self.code,
            current_a=current_a,
            target_current_a=self._target_current_a,
            settled=settled,
        )

    def _ramp_brightness(
        self, start: int, end: int, *, ramp_s: float = BRIGHTNESS_RAMP_S, steps: int = BRIGHTNESS_RAMP_STEPS
    ) -> None:
        if start == end:
            return
        dt = ramp_s / steps
        for i in range(1, steps + 1):
            self._board.write_channel(BEACON_CHANNEL, round(start + (end - start) * i / steps))
            if i < steps:
                time.sleep(dt)

    def all_off(self) -> None:
        """Zero the live drive code (channel 1) and turn the beacon LED off (channel 3),
        disengaging the current loop. Does NOT reset the remembered last-settled code
        (BeaconChannel._last_code) -- the next on() should still ramp to near the driver's
        operating point rather than cold-starting from 0 again."""
        self._loop_engaged = False
        self._integral = 0.0
        self._board.all_off(BEACON_CHANNEL)
        self._board.all_off(RELAY_CHANNEL)

    def read_current_a(self) -> float:
        return self._board.ads.read_current_a(
            channel=BEACON_CHANNEL, sense_resistor_ohm=self._sense_resistor_ohm
        )
