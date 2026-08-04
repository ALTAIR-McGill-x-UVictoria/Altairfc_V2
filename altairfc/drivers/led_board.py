"""Shared owner of the MCP4728 DAC + LDAC + ADS1115 feedback for both LED channels.

The board now drives two physically distinct LEDs from the same MCP4728 (0x60):

    SPHERE_CHANNEL = 0  (MCP4728 channel A)  -- the integrating-sphere source,
                                                 driven by SphereLedSource
    BEACON_CHANNEL = 1  (MCP4728 channel B)  -- brightness setpoint for a
                                                 separate, non-sphere, high-power
                                                 LED used only so ground observers
                                                 can visually spot the payload when
                                                 the sphere source is too dim to see.
                                                 Holding this channel at a nonzero
                                                 code does NOT itself emit light --
                                                 see RELAY_CHANNEL.
    RELAY_CHANNEL  = 3  (MCP4728 channel D)  -- the beacon's actual ON/OFF switch:
                                                 a 2N2222A-gated relay that connects
                                                 BEACON_CHANNEL's LED supply, added
                                                 as a hard fix for RF-coupled gate
                                                 disturbance (see tests/test_LED_system.py).
                                                 Wired NO (normally-open), confirmed on
                                                 hardware: code 0 (coil de-energized) leaves
                                                 the contact open -> beacon OFF; MAX_CODE
                                                 (energized) closes it -> beacon ON. See
                                                 RELAY_CODE_BEACON_ON/_OFF below. Driven by
                                                 BeaconChannel, which is the only thing that
                                                 should write this channel.

Each of channel 0 and channel 1 has its own ADS1115 current-sense input (AIN0
and AIN1 respectively -- see drivers/ads1115.py), but all four channels are
written through the SAME MCP4728 Multi-Write call, which updates them
atomically from one local codes array (see MCP4728Driver.set_vdd_reference).
Two independent driver instances against the same chip would silently clobber
each other's channel on every write. LedBoard is the single writer both
SphereLedSource and BeaconChannel must share -- never construct two
independent MCP4728Driver instances against this board.

The sphere source and the beacon CAN be energized at the same time -- there is
no electrical constraint against it, only an imaging one: the beacon must be
off during actual sphere calibration exposures so it doesn't contaminate them.
That's a scheduling concern (see tasks/lighting_task.py's beacon_windows,
which are the *only* times the beacon is allowed on, precisely because they're
defined as the non-observation windows), not something this driver enforces.
"""

from __future__ import annotations

import logging

from drivers.ads1115 import Ads1115
from drivers.mcp4728_driver import MAX_CODE, NUM_CHANNELS, MCP4728Driver

logger = logging.getLogger(__name__)

DEFAULT_LDAC_PIN = 20  # BCM numbering, physical pin 38

SPHERE_CHANNEL = 0  # MCP4728 channel A -- sphere source drive current
BEACON_CHANNEL = 1  # MCP4728 channel B -- beacon brightness setpoint (no light without RELAY_CHANNEL)
RELAY_CHANNEL  = 3  # MCP4728 channel D -- beacon on/off switch; this is what actually energizes it

# RELAY_CHANNEL is wired NO (normally-open) -- confirmed on hardware 2026-08-03: energizing
# the relay coil (MAX_CODE) closes the contact and powers the beacon; de-energizing it
# (code 0) leaves it open.
RELAY_CODE_BEACON_ON  = MAX_CODE   # coil energized -> NO contact closed -> beacon lit
RELAY_CODE_BEACON_OFF = 0          # coil de-energized -> contact open -> beacon dark


class LedBoard:
    """Owns the single MCP4728, its LDAC line, and the shared ADS1115 feedback."""

    def __init__(
        self,
        *,
        bus,
        dac: MCP4728Driver | None = None,
        ads1115: Ads1115 | None = None,
        pi=None,
        ldac_pin: int = DEFAULT_LDAC_PIN,
        use_ldac: bool = True,
        i2c_dev: str = "/dev/i2c-1",
    ) -> None:
        self._codes = [0] * NUM_CHANNELS  # 0 is every channel's off state, including
                                           # RELAY_CHANNEL (NO-wired) -- see RELAY_CODE_BEACON_OFF
        self._dac = dac if dac is not None else MCP4728Driver(i2c_dev)
        self._ads = ads1115 if ads1115 is not None else Ads1115(bus)

        self._pi = pi
        self._owns_pi = False
        self._ldac_pin = ldac_pin
        if use_ldac:
            self._open_ldac(pi)

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
        logger.info("LedBoard: LDAC (BCM %d) held low", self._ldac_pin)

    @property
    def ads(self) -> Ads1115:
        return self._ads

    def code(self, channel: int) -> int:
        return self._codes[channel]

    def write_channel(self, channel: int, code: int) -> None:
        """Set one channel's DAC code."""
        code = max(0, min(MAX_CODE, int(code)))
        self._codes[channel] = code
        self._flush()

    def all_off(self, channel: int) -> None:
        """Zero one channel's DAC code -- 0 is the off state for every channel on this board."""
        self._codes[channel] = 0
        self._flush()

    def _flush(self) -> None:
        """Push self._codes to the DAC via Multi-Write, not Fast Write.

        Fast Write (MCP4728Driver.set_codes / mcp4728_fast_write) failed with
        an I2C-level ioctl error the first time it was ever exercised against
        real hardware. Multi-Write is confirmed working and costs 3 extra I2C
        transactions per update, irrelevant at this board's update rate, so it
        is used unconditionally.
        """
        if not self._dac.set_vdd_reference(self._codes):
            raise OSError("MCP4728: Multi-Write failed")

    def close(self) -> None:
        """Drive all channels to 0 (every channel's off state), then release the DAC and any
        pigpio handle we own."""
        for ch in range(NUM_CHANNELS):
            self._codes[ch] = 0
        try:
            self._flush()
        except Exception:
            logger.exception("LedBoard: failed to zero channels on close")
        try:
            self._dac.close()
        except Exception:
            logger.exception("LedBoard: failed to close MCP4728")
        if self._pi is not None and self._owns_pi:
            self._pi.stop()
            self._pi = None

    def __enter__(self) -> "LedBoard":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
