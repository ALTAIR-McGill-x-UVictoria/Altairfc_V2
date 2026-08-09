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
from drivers.mcp4728_driver import MAX_CODE

logger = logging.getLogger(__name__)

# Hard ceiling on the beacon's brightness code, enforced by this driver
# regardless of what a caller requests. Previously 3000 -- but on real
# hardware the loop climbed past 3000 without settling at a 0.4 A target
# (still only ~0.19 A at code=2956, still rising), meaning 2100 is this
# driver's minimum output threshold, not close to its actual operating point,
# and the old 3000 ceiling didn't leave enough headroom either. No new
# measured/rated ceiling exists yet, so this now equals MAX_CODE (the DAC's
# own 12-bit limit) and relies entirely on the PI loop's max_code_step rate
# limiting, not a fixed ceiling, to avoid overdriving -- replace with a real
# measured limit once one exists.
BEACON_MAX_SAFE_CODE = MAX_CODE

# Starting code for on()'s ramp the first time the PI loop engages (before any
# prior flash has taught it a settled operating point). The new LED driver
# hardware needs at least 2100 to produce any output at all, so ramping from a
# cold 0 would leave the beacon dark for most of a short beacon_windows flash
# while the PI loop's max_code_step-limited climb crawls up from zero. This is
# only the floor, not the real operating point (see BEACON_MAX_SAFE_CODE) --
# the PI loop still has to climb further from here. Later flashes reuse the
# last settled code instead (see BeaconChannel._last_code).
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

# Settling time after a beacon DAC write (update()'s PI step or _ramp_brightness)
# before returning control to the caller. Both channels share one physical
# ADS1115 and one MCP4728 (see drivers/led_board.py) -- a beacon current step
# lands on the same supply rail / I2C bus the sphere reads from, and at the
# lighting task's ~30-35 Hz tick rate (~28-33 ms apart) that transient hadn't
# always settled by the next tick's sphere ADC read, showing up as jumps in
# the sphere's logged current mean whenever the beacon is actively stepping.
# Not a measured value -- if bench testing still shows sphere-side jumps
# correlated with beacon activity, increase this first.
BEACON_WRITE_SETTLE_S = 0.003

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
        # See SphereLedSource's identical comment: 0.001 was tighter than the
        # sensor's actual single-sample noise floor, so "settled" flickered
        # on ADC noise alone even once genuinely converged.
        self._deadband_a = 0.005
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
                # Anti-windup: clamp the I-term itself to the same per-step
                # authority as max_code_step (not just the final delta), and
                # claw back the stored integral to match whenever that clamp
                # bites. Without this, a long ramp (cold start, or climbing
                # from BEACON_STARTUP_CODE to the real operating point) keeps
                # accumulating the *unclamped* error the whole time, so the
                # integral term ends up demanding far more correction than
                # the actuator could ever apply in one step and overshoots
                # badly unwinding it afterward -- see SphereLedSource.update()
                # for the mirrored logic.
                i_term = self._ki * self._integral
                i_term_clamped = max(-self._max_code_step, min(self._max_code_step, i_term))
                if i_term_clamped != i_term and self._ki != 0:
                    self._integral = i_term_clamped / self._ki

                delta = self._kp * error + i_term_clamped
                delta = max(-self._max_code_step, min(self._max_code_step, delta))

                new_code = max(0, min(BEACON_MAX_SAFE_CODE, int(round(self.code + delta))))
                # Also stop accumulating once railed and the correction can't
                # actually move the DAC any further this step.
                if new_code == self.code and new_code in (0, BEACON_MAX_SAFE_CODE):
                    self._integral -= error * dt
                if new_code != self.code:
                    self._board.write_channel(BEACON_CHANNEL, new_code)
                    self._last_code = new_code
                    # Let the shared rail/I2C bus settle before returning --
                    # see BEACON_WRITE_SETTLE_S.
                    time.sleep(BEACON_WRITE_SETTLE_S)

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
