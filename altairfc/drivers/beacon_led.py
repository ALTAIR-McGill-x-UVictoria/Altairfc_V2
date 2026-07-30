"""External spotter beacon: a non-sphere, high-power LED for visual tracking.

Lets ground observers visually spot the payload when the sphere source is too
dim to see. Wired to MCP4728 channel 1 (0x60), with its own current-sense
resistor on ADS1115 AIN1 — a physically separate LED from the sphere source
on channel 0, not a second sphere drive stage (see drivers/sphere_led.py for
the history of that repurposing).

Open-loop only: no current-hold PI loop, since the beacon's job is visual
strobing, not calibrated optical output. Must share the same LedBoard as
SphereLedSource so the sphere/beacon interlock (drivers/led_board.py) is
enforced on every write — never open a second MCP4728Driver against this chip.

Usage:
    board = LedBoard(bus=bus)
    beacon = BeaconChannel(board=board)
    beacon.set_code(3000)   # clamped to BEACON_MAX_SAFE_CODE regardless
    beacon.all_off()
"""

from __future__ import annotations

import logging

from drivers.ads1115 import SENSE_RESISTOR_OHM
from drivers.led_board import BEACON_CHANNEL, LedBoard

logger = logging.getLogger(__name__)

# Hard ceiling on the beacon's DAC code, enforced by this driver regardless of
# what a caller requests. This is a different physical LED than the sphere
# source (MAX_SAFE_CODE=1400 there) — measured/rated limit for this LED.
BEACON_MAX_SAFE_CODE = 3000


class BeaconChannel:
    """Open-loop drive for the external spotter LED (MCP4728 channel 1)."""

    def __init__(self, *, board: LedBoard, sense_resistor_ohm: float = SENSE_RESISTOR_OHM) -> None:
        self._board = board
        self._sense_resistor_ohm = sense_resistor_ohm

    @property
    def code(self) -> int:
        return self._board.code(BEACON_CHANNEL)

    def set_code(self, code: int) -> None:
        """Set the beacon's drive to a DAC code, hard-clamped to BEACON_MAX_SAFE_CODE.

        Raises drivers.led_board.InterlockViolation if the sphere channel is
        currently energized — callers must turn the sphere off first.
        """
        self._board.write_channel(BEACON_CHANNEL, max(0, min(BEACON_MAX_SAFE_CODE, int(code))))

    def all_off(self) -> None:
        self._board.all_off(BEACON_CHANNEL)

    def read_current_a(self) -> float:
        return self._board.ads.read_current_a(
            channel=BEACON_CHANNEL, sense_resistor_ohm=self._sense_resistor_ohm
        )
