"""External spotter beacon: a non-sphere, high-power LED for visual tracking.

Lets ground observers visually spot the payload when the sphere source is too
dim to see. Two MCP4728 channels are involved (0x60), with different jobs:

    BEACON_CHANNEL = 1  brightness setpoint. Held at a constant code while the
                         beacon is in use; on its own this emits no light.
    RELAY_CHANNEL  = 3  the actual ON/OFF switch -- a 2N2222A-gated relay that
                         connects BEACON_CHANNEL's LED supply, added as a hard
                         fix for RF-coupled gate disturbance (see
                         tests/test_LED_system.py). Wired NO (normally-open),
                         confirmed on hardware 2026-08-03: code 0 (de-energized)
                         leaves the contact open -> beacon OFF; MAX_CODE
                         (energized) closes it -> beacon ON. See
                         led_board.RELAY_CODE_BEACON_ON/_OFF.

Flashing the beacon is therefore: set_brightness() once, then on()/off() to
toggle the relay for each flash -- not repeated set_brightness() calls.

on()/off() never let the relay's contacts make or break while current is
flowing through the LED. Closing the relay while channel 1 is already at its
brightness setpoint hot-switches the contacts onto a live circuit -- the same
family of transient (arcing, RF/EMI injection) this relay was added to
eliminate in the first place. Instead, on() forces channel 1 to 0, energizes
the relay onto a currentless circuit, waits for the contact to physically
settle, then ramps brightness up quickly; off() ramps brightness back down to
0 before de-energizing the relay, so the break happens currentless too.

Current sense is on ADS1115 AIN1, across its own 1.5 ohm (nominal) sense
resistor -- same nominal value as the sphere's 1.5 ohm on AIN0, but a
physically separate resistor (see tests/test_LED_system.py's
CURRENT_SENSE_RESISTOR_OHM and config/settings.toml's
[tasks.lighting].beacon_sense_resistor_ohm); do not reuse
drivers.ads1115.SENSE_RESISTOR_OHM (that is the sphere's value) as the
beacon's default.

Open-loop only: no current-hold PI loop, since the beacon's job is visual
strobing, not calibrated optical output. Must share the same LedBoard as
SphereLedSource, which is the single writer to this MCP4728 -- never open a
second MCP4728Driver against this chip. The sphere and beacon can be
energized simultaneously (no electrical constraint); keeping the beacon off
during actual sphere calibration exposures is a scheduling concern handled by
tasks/lighting_task.py's beacon_windows, not something this driver enforces.

Usage:
    board = LedBoard(bus=bus)
    beacon = BeaconChannel(board=board)
    beacon.set_brightness(1500)   # clamped to BEACON_MAX_SAFE_CODE regardless
    beacon.on()                   # energizes the relay -- beacon lit
    beacon.off()                  # de-energizes the relay -- beacon dark
    beacon.all_off()              # zeros brightness and de-energizes the relay -- beacon dark
"""

from __future__ import annotations

import logging
import time

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
# the sphere source (MAX_SAFE_CODE=1400 there) — measured/rated limit for this LED.
BEACON_MAX_SAFE_CODE = 3000

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


class BeaconChannel:
    """Open-loop drive for the external spotter LED: brightness (ch 1) + relay on/off (ch 3)."""

    def __init__(self, *, board: LedBoard, sense_resistor_ohm: float = BEACON_SENSE_RESISTOR_OHM) -> None:
        self._board = board
        self._sense_resistor_ohm = sense_resistor_ohm
        # The brightness setpoint survives independently of channel 1's live DAC
        # code, because on()/off() deliberately drive channel 1 to 0 around each
        # relay transition (see module docstring) -- self.code alone can't be
        # used to recover "what brightness should this come back on at".
        self._target_brightness = 0

    @property
    def code(self) -> int:
        return self._board.code(BEACON_CHANNEL)

    @property
    def lit(self) -> bool:
        """Whether the beacon LED is actually emitting light right now."""
        return self._board.code(RELAY_CHANNEL) == RELAY_CODE_BEACON_ON

    def set_brightness(self, code: int) -> None:
        """Set the beacon's brightness setpoint, clamped to BEACON_MAX_SAFE_CODE.

        This alone does not energize the LED — see on()/off(). If the beacon is
        already lit, applies immediately; otherwise takes effect the next time
        on() is called (channel 1 stays at 0 while dark — see module docstring).
        """
        self._target_brightness = max(0, min(BEACON_MAX_SAFE_CODE, int(code)))
        if self.lit:
            self._board.write_channel(BEACON_CHANNEL, self._target_brightness)

    def on(self) -> None:
        """Turn the beacon LED on.

        Energizes the relay (channel 3) onto a currentless circuit, then ramps
        channel 1 up to its brightness setpoint — see module docstring for why.
        """
        self._board.write_channel(BEACON_CHANNEL, 0)
        self._board.write_channel(RELAY_CHANNEL, RELAY_CODE_BEACON_ON)
        time.sleep(RELAY_SETTLE_S)
        self._ramp_brightness(0, self._target_brightness)

    def off(self) -> None:
        """Turn the beacon LED off.

        Ramps channel 1 back down to 0 before de-energizing the relay (channel
        3), so the contact breaks currentless too — see module docstring.
        """
        self._ramp_brightness(self.code, 0)
        self._board.write_channel(RELAY_CHANNEL, RELAY_CODE_BEACON_OFF)

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
        """Zero the brightness setpoint (channel 1) and turn the beacon LED off (channel 3)."""
        self._target_brightness = 0
        self._board.all_off(BEACON_CHANNEL)
        self._board.all_off(RELAY_CHANNEL)

    def read_current_a(self) -> float:
        return self._board.ads.read_current_a(
            channel=BEACON_CHANNEL, sense_resistor_ohm=self._sense_resistor_ohm
        )
