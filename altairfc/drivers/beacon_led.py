"""External spotter beacon: a non-sphere, high-power LED for visual tracking.

Lets ground observers visually spot the payload when the sphere source is too
dim to see. Two MCP4728 channels are involved (0x60), with different jobs:

    BEACON_CHANNEL = 1  brightness setpoint. Held at a constant code while the
                         beacon is in use; on its own this emits no light.
    RELAY_CHANNEL  = 3  the actual ON/OFF switch -- a 2N2222A-gated relay that
                         connects BEACON_CHANNEL's LED supply, added as a hard
                         fix for RF-coupled gate disturbance (see
                         tests/test_LED_system.py). Wired NC (normally-closed),
                         confirmed on hardware 2026-08-03: code 0 (de-energized)
                         closes the contact -> beacon ON; MAX_CODE (energized)
                         opens it -> beacon OFF. See led_board.RELAY_CODE_BEACON_ON/_OFF.

Flashing the beacon is therefore: set_brightness() once, then on()/off() to
toggle the relay for each flash -- not repeated set_brightness() calls.

Current sense is on ADS1115 AIN1, across its own 3 ohm (nominal) sense
resistor -- a different value from the sphere's 2.2 ohm on AIN0 (see
tests/test_LED_system.py's CURRENT_SENSE_RESISTOR_OHM); do not reuse
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
    beacon.on()                   # de-energizes the relay (NC wiring) -- beacon lit
    beacon.off()                  # energizes the relay -- beacon dark
    beacon.all_off()              # zeros brightness, energizes the relay -- beacon dark
"""

from __future__ import annotations

import logging

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

# AIN1's sense resistor for the beacon's drive current — NOT the same value as
# the sphere's (drivers.ads1115.SENSE_RESISTOR_OHM = 2.2 ohm, channel 0 only).
BEACON_SENSE_RESISTOR_OHM = 3.0


class BeaconChannel:
    """Open-loop drive for the external spotter LED: brightness (ch 1) + relay on/off (ch 3)."""

    def __init__(self, *, board: LedBoard, sense_resistor_ohm: float = BEACON_SENSE_RESISTOR_OHM) -> None:
        self._board = board
        self._sense_resistor_ohm = sense_resistor_ohm

    @property
    def code(self) -> int:
        return self._board.code(BEACON_CHANNEL)

    @property
    def lit(self) -> bool:
        """Whether the beacon LED is actually emitting light right now."""
        return self._board.code(RELAY_CHANNEL) == RELAY_CODE_BEACON_ON

    def set_brightness(self, code: int) -> None:
        """Hold the beacon's brightness setpoint (channel 1), clamped to BEACON_MAX_SAFE_CODE.

        This alone does not energize the LED — see on()/off().
        """
        self._board.write_channel(BEACON_CHANNEL, max(0, min(BEACON_MAX_SAFE_CODE, int(code))))

    def on(self) -> None:
        """Turn the beacon LED on — de-energizes the relay (channel 3); see RELAY_CODE_BEACON_ON."""
        self._board.write_channel(RELAY_CHANNEL, RELAY_CODE_BEACON_ON)

    def off(self) -> None:
        """Turn the beacon LED off — energizes the relay (channel 3); see RELAY_CODE_BEACON_OFF.
        Leaves the brightness setpoint (channel 1) untouched."""
        self._board.write_channel(RELAY_CHANNEL, RELAY_CODE_BEACON_OFF)

    def all_off(self) -> None:
        """Zero the brightness setpoint (channel 1) and turn the beacon LED off (channel 3)."""
        self._board.all_off(BEACON_CHANNEL)
        self._board.write_channel(RELAY_CHANNEL, RELAY_CODE_BEACON_OFF)

    def read_current_a(self) -> float:
        return self._board.ads.read_current_a(
            channel=BEACON_CHANNEL, sense_resistor_ohm=self._sense_resistor_ohm
        )
