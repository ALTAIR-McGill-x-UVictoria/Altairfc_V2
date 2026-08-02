"""ADS1115 16-bit ADC on the LED driver board.

One-shot conversions on an arbitrary MUX setting, plus the Wheatstone-bridge
and NTC math for the LED thermistor.  This code previously existed as four
near-identical copies across tests/test_LED_system.py,
tests/test_led_driver_thermistor.py, tests/test_ads1115.py and
tests/stream_UVICPDRO_calibration.py; this module is the single version the
goniometer rig uses.

Wiring on this board, which constrains what can be measured:

    AIN0        single-ended  MCP4728 channel 0 (sphere source) drive current,
                              across a 2.2 ohm sense resistor
    AIN1        single-ended  MCP4728 channel 1 (external spotter beacon) drive
                              current, across its own 2.2 ohm sense resistor
    AIN2-AIN3   differential  NTC thermistor Wheatstone bridge

AIN2 and AIN3 are therefore *not* usable single-ended — reading either alone
returns a bridge node voltage, not an independent signal.
"""

from __future__ import annotations

import logging
import math
import time

logger = logging.getLogger(__name__)

DEFAULT_ADDR = 0x4A  # ADDR pin strapped to SDA

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

MUX_SINGLE = {0: 0b100, 1: 0b101, 2: 0b110, 3: 0b111}
MUX_DIFF_2_3 = 0b011  # AIN2-AIN3: thermistor bridge

DR_128SPS = 0b100
DR_860SPS = 0b111  # fastest one-shot rate; used for oversampled current-loop reads
_COMP_QUE_DISABLE = 0b11

_DR_SPS = {
    0b000: 8, 0b001: 16, 0b010: 32, 0b011: 64,
    0b100: 128, 0b101: 250, 0b110: 475, 0b111: 860,
}

# A single transient I2C fault (e.g. errno 121 "Remote I/O error") shouldn't be
# allowed to take down an entire caller-level control loop — see read_volts().
_MAX_I2C_RETRIES = 3
_I2C_RETRY_DELAY_S = 0.002

# gain key -> (PGA register bits, full-scale range in volts)
GAIN_FSR = {
    0: (0b000, 6.144),
    1: (0b001, 4.096),
    2: (0b010, 2.048),  # reset default, matches Vdd=3.3V single-ended range
    4: (0b011, 1.024),
    8: (0b100, 0.512),
    16: (0b101, 0.256),
}

# Thermistor bridge on the LED driver board
BRIDGE_R = 10000.0
VEXC = 3.3
THERM_R25 = 10000.0
THERM_B = 3380.0
T0_KELVIN = 298.15

# LED drive current sense
SENSE_RESISTOR_OHM = 2.2


def bridge_volts_to_resistance(vdiff: float, r: float = BRIDGE_R, vexc: float = VEXC) -> float:
    """TH1 = R * (Vexc/2 + Vdiff) / (Vexc/2 - Vdiff)"""
    half_vexc = vexc / 2.0
    denom = half_vexc - vdiff
    if denom == 0:
        return float("inf")
    return r * (half_vexc + vdiff) / denom


def resistance_to_celsius(
    r: float, r25: float = THERM_R25, b: float = THERM_B, t0: float = T0_KELVIN
) -> float:
    if r <= 0:
        return float("nan")
    t_kelvin = 1.0 / (1.0 / t0 + (1.0 / b) * math.log(r / r25))
    return t_kelvin - 273.15


def _to_int16(raw: int) -> int:
    val = raw & 0xFFFF
    return val - 0x10000 if val & 0x8000 else val


class Ads1115:
    """One-shot reader for the ADS1115 on the LED driver board.

    Takes an already-open ``smbus2.SMBus`` so it can share the I2C bus with
    the MCP4728 sitting at 0x60 on the same board.

    Usage:
        bus = smbus2.SMBus(1)
        adc = Ads1115(bus)
        current_a = adc.read_current_a()
        temp_c = adc.read_bridge_temperature_c()
    """

    def __init__(self, bus, addr: int = DEFAULT_ADDR) -> None:
        self._bus = bus
        self._addr = addr

    def read_volts(self, mux_bits: int, gain: int = 2, data_rate: int = DR_128SPS) -> float:
        """Trigger one conversion on an arbitrary MUX setting and return volts.

        Retries the whole conversion up to _MAX_I2C_RETRIES times on a transient
        I2C fault (OSError, e.g. errno 121 "Remote I/O error") before giving up
        — a single dropped transaction on a real bus shouldn't be allowed to
        kill an entire caller-level control loop (e.g. SphereLedSource.update(),
        called ~13x/sec, previously took the whole current-hold loop down and
        forced a full ramp-from-zero restart on one bad NACK). Still raises
        OSError/TimeoutError after exhausting retries — a persistent fault must
        not be swallowed.
        """
        pga_bits, fsr = GAIN_FSR[gain]

        cfg = 0
        cfg |= 1 << 15  # OS: start conversion
        cfg |= (mux_bits & 0x07) << 12
        cfg |= (pga_bits & 0x07) << 9
        cfg |= 1 << 8  # MODE: single-shot
        cfg |= (data_rate & 0x07) << 5
        cfg |= _COMP_QUE_DISABLE & 0x03

        for attempt in range(1, _MAX_I2C_RETRIES + 1):
            try:
                self._bus.write_i2c_block_data(self._addr, REG_CONFIG, [(cfg >> 8) & 0xFF, cfg & 0xFF])

                time.sleep((1.0 / _DR_SPS[data_rate]) * 1.5)
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    raw = self._bus.read_i2c_block_data(self._addr, REG_CONFIG, 2)
                    status = (raw[0] << 8) | raw[1]
                    if status & 0x8000:  # OS=1 means conversion complete / ADC idle
                        break
                    time.sleep(0.001)
                else:
                    raise TimeoutError(f"ADS1115 at 0x{self._addr:02X} conversion did not complete")

                raw = self._bus.read_i2c_block_data(self._addr, REG_CONVERSION, 2)
                code = _to_int16((raw[0] << 8) | raw[1])
                return code * fsr / 32768.0
            except OSError as exc:
                if attempt == _MAX_I2C_RETRIES:
                    raise
                logger.warning(
                    "Ads1115: I2C fault on attempt %d/%d (%s), retrying",
                    attempt, _MAX_I2C_RETRIES, exc,
                )
                time.sleep(_I2C_RETRY_DELAY_S)

        raise AssertionError("unreachable")  # loop always returns or raises

    def read_single_ended(self, channel: int, gain: int = 2) -> float:
        """Read AIN<channel> against GND, in volts.

        Channels 2 and 3 share pins with the AIN2-AIN3 thermistor bridge and do
        not carry an independent signal; see the module docstring.
        """
        if channel not in MUX_SINGLE:
            raise ValueError(f"channel must be 0-3, got {channel}")
        return self.read_volts(MUX_SINGLE[channel], gain=gain)

    def read_bridge_temperature_c(self, gain: int = 2) -> float:
        """Read the NTC bridge differentially (AIN2-AIN3) and convert to degrees C."""
        vdiff = self.read_volts(MUX_DIFF_2_3, gain=gain)
        return resistance_to_celsius(bridge_volts_to_resistance(vdiff))

    def read_current_a(
        self, channel: int = 0, sense_resistor_ohm: float = SENSE_RESISTOR_OHM, gain: int = 2
    ) -> float:
        """Read an LED channel's drive current from its AIN<channel> sense resistor, in amps.

        channel 0 = sphere source (MCP4728 A), channel 1 = spotter beacon (MCP4728 B).
        """
        return self.read_single_ended(channel, gain=gain) / sense_resistor_ohm

    def read_current_a_oversampled(
        self,
        channel: int = 0,
        sense_resistor_ohm: float = SENSE_RESISTOR_OHM,
        gain: int = 2,
        oversample: int = 16,
    ) -> float:
        """Trimmed-mean current reading over `oversample` 860 SPS conversions, in amps.

        The RF transceiver induces a persistent, broadband disturbance onto the
        LED MOSFET gate drive -- present on nearly every raw sample, not
        occasional spikes -- that a single one-shot conversion (read_current_a)
        mostly passes straight through to a control loop. Averaging several 860
        SPS conversions, dropping the highest/lowest so one corrupted I2C read
        can't skew the mean, is what actually rejects it (see the closed-loop
        notes in tests/test_LED_system.py, the origin of this approach).
        """
        if channel not in MUX_SINGLE:
            raise ValueError(f"channel must be 0-3, got {channel}")
        volts_samples = [
            self.read_volts(MUX_SINGLE[channel], gain=gain, data_rate=DR_860SPS)
            for _ in range(max(1, oversample))
        ]
        if len(volts_samples) >= 5:
            volts_samples.sort()
            trim = max(1, len(volts_samples) // 8)
            volts_samples = volts_samples[trim: len(volts_samples) - trim]
        avg_v = sum(volts_samples) / len(volts_samples)
        return avg_v / sense_resistor_ohm
