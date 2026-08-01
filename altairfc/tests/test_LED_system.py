"""
LED system integration test: MCP4728 DAC drive + ADS1115 thermistor bridge
+ ADS1115 single-ended channel monitor, all streamed live.

Sets one MCP4728 output channel (0-3 = A-D) to a fixed DAC code and holds
it there, then continuously streams two things per LED channel, both read
from the same ADS1115 (the ADS1113 previously used for the thermistor
bridge has been rewired away and is no longer part of this circuit):

  1. The thermistor bridge temperature — now read as an ADS1115
     differential conversion on AIN2-AIN3 (MUX=011), the only ADS1115 MUX
     setting that pairs AIN3. Same Wheatstone bridge/NTC math as
     tests/test_led_driver_thermistor.py, just moved off the ADS1113.
  2. The live single-ended voltage on the ADS1115 channel matching the
     MCP4728 channel index (MCP4728 channel 0 -> ADS1115 AIN0, etc.) —
     see tests/test_ads1115.py for the register-level driver this reuses.

Both reads are one-shot conversions time-multiplexed on the single
ADS1115, so each streamed line reflects two sequential conversions.

LDAC (BCM 20 / physical pin 38) must be driven LOW for MCP4728 writes to
reach VOUT immediately, same as tests/test_led_driver_thermistor.py; pass
--no-ldac if it's hardwired to GND on this board.

Closed-loop current hold (--target-current):
    AIN0/AIN1 sit across their own sense resistor on the sphere/beacon
    drive current respectively — 2.2 ohm on channel 0, 3 ohm nominal on
    channel 1, NOT the same value (see drivers/ads1115.py's wiring notes) —
    channels 2/3 have no independent current-sense signal, only the
    thermistor bridge, so closed-loop mode is restricted to channel 0/1.
    The RF transceiver on this payload induces a persistent, broadband
    disturbance onto the MOSFET gate drive — not occasional spikes but a
    jitter present on essentially every sample while the radio is active
    (confirmed: current settles rock-solid with the radio unplugged) —
    that a fixed --code cannot reject and that a single 860 SPS sample per
    control step mostly passes straight through to the actuator. 860 SPS
    is already the ADS1115's fastest one-shot rate, so there's no faster
    raw sampling to reach for; the fix is deeper averaging, not more speed.
    Each control iteration therefore averages --oversample raw conversions
    (trimmed mean, dropping the highest/lowest to also reject any single
    corrupted I2C read) before that sample even reaches the EMA filter
    (--filter-alpha), which itself runs a longer time constant than the
    naive "filter the raw 860 SPS stream directly" approach — that combo
    is what actually rejects a disturbance this size, not a faster loop.
    The slower thermistor-bridge read / status print stays on the
    --interval cadence so it doesn't throttle the correction rate.

Usage:
    python tests/test_LED_system.py --channel 0 --code 2047
    python tests/test_LED_system.py --channel 2 --code 4095 --interval 0.5
    python tests/test_LED_system.py --channel 0 --code 0 --no-ldac
    python tests/test_LED_system.py --channel 0 --target-current 0.25
    python tests/test_LED_system.py --channel 1 --target-current 0.10 --kp 4000 --ki 1200
"""

import argparse
import math
import sys
import time

MCP4728_ADDR = 0x60
MCP4728_MAX_CODE = 4095

ADS1115_ADDR = 0x4A  # ADDR pin strapped to SDA
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG = 0x01
_ADS1115_MUX_SINGLE = {0: 0b100, 1: 0b101, 2: 0b110, 3: 0b111}
_ADS1115_MUX_DIFF_2_3 = 0b011  # AIN2-AIN3: thermistor bridge (rewired off the ADS1113)
_ADS1115_DR_128SPS = 0b100
_ADS1115_DR_860SPS = 0b111  # fastest one-shot rate; used for the current-hold control loop
_ADS1115_COMP_QUE_DISABLE = 0b11

# LED drive current sense: AIN0/AIN1 each sit across their own sense resistor
# (sphere/beacon respectively — see drivers/ads1115.py), but the two are NOT
# the same value: channel 0 is 2.2 ohm, channel 1 is 3 ohm nominal. AIN2/AIN3
# are the thermistor bridge and carry no independent current signal.
CURRENT_SENSE_RESISTOR_OHM = {0: 2.2, 1: 3.0}
CURRENT_SENSE_CHANNELS = tuple(CURRENT_SENSE_RESISTOR_OHM)
_ADS1115_GAIN_FSR = {
    0: (0b000, 6.144),
    1: (0b001, 4.096),
    2: (0b010, 2.048),  # reset default, matches Vdd=3.3V single-ended range
    4: (0b011, 1.024),
    8: (0b100, 0.512),
    16: (0b101, 0.256),
}

DEFAULT_LDAC_PIN = 20  # BCM numbering, physical pin 38

# Bridge / thermistor constants (same bridge as test_led_driver_thermistor.py)
BRIDGE_R = 10000.0
VEXC = 3.3
THERM_R25 = 10000.0
THERM_B = 3380.0
T0_KELVIN = 298.15


class Ldac:
    """Drives the MCP4728 LDAC pin via pigpio so DAC writes reach VOUT immediately."""

    def __init__(self, pin, enabled):
        self._pin = pin
        self._enabled = enabled
        self._pi = None
        if not enabled:
            return

        import pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Cannot connect to pigpio daemon. Run: sudo pigpiod")
        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.write(pin, 0)  # idle low: outputs track input registers directly
        print(f"[INFO] LDAC (BCM {pin}) held LOW — DAC writes will update VOUT immediately")

    def close(self):
        if self._enabled and self._pi is not None:
            self._pi.write(self._pin, 0)
            self._pi.stop()


def mcp4728_multi_write_channel(bus, addr, channel, code, vref_vdd=True, gain=1):
    """Write one MCP4728 channel via Multi-Write, forcing Vref/gain so VOUT tracks Vdd."""
    code = max(0, min(MCP4728_MAX_CODE, code))
    vref_bit = 0 if vref_vdd else 1
    gain_bit = 1 if gain == 2 else 0

    cmd = 0x40 | (channel << 1)
    upper = (vref_bit << 7) | (0 << 5) | (gain_bit << 4) | ((code >> 8) & 0x0F)
    lower = code & 0xFF
    bus.write_i2c_block_data(addr, cmd, [upper, lower])


def bridge_volts_to_resistance(vdiff, r=BRIDGE_R, vexc=VEXC):
    """TH1 = R * (Vexc/2 + Vdiff) / (Vexc/2 - Vdiff)"""
    half_vexc = vexc / 2.0
    denom = half_vexc - vdiff
    if denom == 0:
        return float("inf")
    return r * (half_vexc + vdiff) / denom


def resistance_to_celsius(r, r25=THERM_R25, b=THERM_B, t0=T0_KELVIN):
    if r <= 0:
        return float("nan")
    t_kelvin = 1.0 / (1.0 / t0 + (1.0 / b) * math.log(r / r25))
    return t_kelvin - 273.15


_ADS1115_DR_SPS = {
    0b000: 8, 0b001: 16, 0b010: 32, 0b011: 64,
    0b100: 128, 0b101: 250, 0b110: 475, 0b111: 860,
}


def _ads1115_to_int16(raw):
    val = raw & 0xFFFF
    return val - 0x10000 if val & 0x8000 else val


def ads1115_one_shot_read_mux(bus, addr, mux_bits, gain=2, data_rate=_ADS1115_DR_128SPS):
    """Trigger a single-shot conversion on an arbitrary ADS1115 MUX setting and block until ready."""
    pga_bits, _fsr = _ADS1115_GAIN_FSR[gain]

    cfg = 0
    cfg |= 1 << 15  # OS: start conversion
    cfg |= (mux_bits & 0x07) << 12
    cfg |= (pga_bits & 0x07) << 9
    cfg |= 1 << 8  # MODE: single-shot
    cfg |= (data_rate & 0x07) << 5
    cfg |= _ADS1115_COMP_QUE_DISABLE & 0x03

    bus.write_i2c_block_data(addr, ADS1115_REG_CONFIG, [(cfg >> 8) & 0xFF, cfg & 0xFF])

    # Scale the settle wait to the requested rate so --target-current's 860 SPS
    # loop isn't stuck waiting out the 128 SPS default on every iteration.
    time.sleep((1.0 / _ADS1115_DR_SPS[data_rate]) * 1.5)
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        raw = bus.read_i2c_block_data(addr, ADS1115_REG_CONFIG, 2)
        status = (raw[0] << 8) | raw[1]
        if status & 0x8000:  # OS=1 means conversion complete / ADC idle
            break
        time.sleep(0.001)
    else:
        raise TimeoutError("ADS1115 conversion did not complete in time")

    raw = bus.read_i2c_block_data(addr, ADS1115_REG_CONVERSION, 2)
    code = _ads1115_to_int16((raw[0] << 8) | raw[1])
    return code


def ads1115_code_to_volts(code, gain=2):
    _pga_bits, fsr = _ADS1115_GAIN_FSR[gain]
    return code * fsr / 32768.0


def volts_to_current_a(volts, sense_resistor_ohm):
    return volts / sense_resistor_ohm


def read_current_a_oversampled(bus, addr, mux_bits, gain, oversample, sense_resistor_ohm):
    """Average several raw 860 SPS conversions (trimmed mean) into one current sample.

    A single raw conversion mostly passes the RF-coupled gate disturbance
    straight through — it's a persistent broadband jitter present on nearly
    every sample, not occasional spikes, so raising the raw sample rate
    doesn't help (860 SPS is already the ADS1115's ceiling). Averaging
    multiple conversions per control step, with the extremes dropped so one
    corrupted I2C read can't skew the mean, is what actually reduces the
    noise reaching the EMA filter and the PI loop.
    """
    volts_samples = [
        ads1115_code_to_volts(
            ads1115_one_shot_read_mux(bus, addr, mux_bits, gain=gain, data_rate=_ADS1115_DR_860SPS),
            gain=gain,
        )
        for _ in range(max(1, oversample))
    ]
    if len(volts_samples) >= 5:
        volts_samples.sort()
        trim = max(1, len(volts_samples) // 8)
        volts_samples = volts_samples[trim: len(volts_samples) - trim]
    avg_v = sum(volts_samples) / len(volts_samples)
    return volts_to_current_a(avg_v, sense_resistor_ohm)


class CurrentHoldLoop:
    """PI loop that adjusts an MCP4728 DAC code to hold measured LED current at
    a target, rejecting the RF-coupled disturbance on the MOSFET gate signal
    rather than a fixed --code, which just rides the disturbance out.

    Expects an already oversampled/trimmed current reading (see
    read_current_a_oversampled) and applies a further EMA filter on top,
    since the same RF pickup that jitters the gate also shows up as noise on
    the ADS1115 reading — without both stages the loop chases noise instead
    of correcting the real operating point.
    """

    def __init__(self, target_a, kp, ki, deadband_a, max_code_step, filter_alpha):
        self.target_a = target_a
        self.kp = kp
        self.ki = ki
        self.deadband_a = deadband_a
        self.max_code_step = max_code_step
        self.filter_alpha = filter_alpha

        self._integral = 0.0
        self._filtered_a = None
        self._last_t = None

    def step(self, raw_current_a, current_code):
        """Feed one raw current sample; returns (new_code, filtered_current_a, settled)."""
        if self._filtered_a is None:
            self._filtered_a = raw_current_a
        else:
            self._filtered_a += self.filter_alpha * (raw_current_a - self._filtered_a)

        error = self.target_a - self._filtered_a
        settled = abs(error) <= self.deadband_a

        now = time.monotonic()
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now

        if settled:
            return current_code, self._filtered_a, settled

        self._integral += error * dt
        delta = self.kp * error + self.ki * self._integral
        delta = max(-self.max_code_step, min(self.max_code_step, delta))

        new_code = max(0, min(MCP4728_MAX_CODE, int(round(current_code + delta))))
        # Anti-windup: stop accumulating once railed and the correction can't
        # actually move the DAC any further this step.
        if new_code == current_code and new_code in (0, MCP4728_MAX_CODE):
            self._integral -= error * dt

        return new_code, self._filtered_a, settled


def main():
    parser = argparse.ArgumentParser(
        description="Hold an MCP4728 output channel at a fixed code while streaming the "
                    "ADS1115 thermistor bridge temperature (AIN2-AIN3) and the matching "
                    "ADS1115 single-ended channel voltage")
    parser.add_argument("--channel", type=int, choices=[0, 1, 2, 3], required=True,
                         help="MCP4728 output channel (0-3 = A-D); also selects the ADS1115 "
                              "single-ended input channel to monitor (0->AIN0, 1->AIN1, etc.)")
    parser.add_argument("--code", type=int, default=None,
                         help="12-bit DAC code (0-4095) to hold on the selected MCP4728 channel. "
                              "Required unless --target-current is given, in which case it's just "
                              "the starting point for the current loop (default: 0)")
    parser.add_argument("--target-current", type=float, default=None,
                         help="Engage a PI current-hold loop against the AIN current-sense reading "
                              "instead of holding a fixed --code, in amps. Only channel 0/1 have a "
                              "current-sense sense resistor wired up.")
    parser.add_argument("--kp", type=float, default=3000.0,
                         help="Current loop proportional gain, DAC codes per amp of error (--target-current)")
    parser.add_argument("--ki", type=float, default=800.0,
                         help="Current loop integral gain, DAC codes per amp-second of error (--target-current)")
    parser.add_argument("--deadband-a", type=float, default=0.001,
                         help="Current loop deadband, amps (--target-current)")
    parser.add_argument("--max-code-step", type=int, default=8,
                         help="Max DAC code change per control iteration — bounds how much a single "
                              "residual-noise correction can move the actuator (--target-current)")
    parser.add_argument("--oversample", type=int, default=16,
                         help="Raw 860 SPS conversions averaged (trimmed mean) into each current-loop "
                              "sample before the EMA filter — the RF disturbance is a persistent jitter "
                              "on nearly every sample, not occasional spikes, so this (not a faster raw "
                              "sample rate — 860 SPS is already the ADS1115's ceiling) is what actually "
                              "rejects it. Higher = smoother but slower to settle (--target-current)")
    parser.add_argument("--filter-alpha", type=float, default=0.08,
                         help="EMA smoothing (0-1] applied on top of the oversampled reading before the "
                              "PI loop sees it. Lower = smoother but slower to settle (--target-current)")
    parser.add_argument("--sense-resistor-ohm", type=float, default=None,
                         help="AIN current-sense resistor value, ohms (--target-current). Defaults "
                              f"to the per-channel nominal value {CURRENT_SENSE_RESISTOR_OHM}")
    parser.add_argument("--bus", default="/dev/i2c-1", help="Shared I2C device node for both chips")
    parser.add_argument("--mcp4728-addr", default=hex(MCP4728_ADDR), help="MCP4728 I2C address")
    parser.add_argument("--ads1115-addr", default=hex(ADS1115_ADDR), help="ADS1115 I2C address")
    parser.add_argument("--ads1115-gain", type=int, choices=sorted(_ADS1115_GAIN_FSR), default=2,
                         help="ADS1115 PGA gain for the single-ended LED-channel read")
    parser.add_argument("--therm-gain", type=int, choices=sorted(_ADS1115_GAIN_FSR), default=2,
                         help="ADS1115 PGA gain for the AIN2-AIN3 thermistor-bridge differential read")
    parser.add_argument("--ldac-pin", type=int, default=DEFAULT_LDAC_PIN,
                         help="BCM pin driving LDAC (default: 20 / physical pin 38)")
    parser.add_argument("--no-ldac", action="store_true",
                         help="Don't drive LDAC (use if it's hardwired to GND)")
    parser.add_argument("--interval", type=float, default=1.0,
                         help="Seconds between status prints (also the legacy open-loop sample period; "
                              "the --target-current control loop itself runs far faster than this)")
    args = parser.parse_args()

    closed_loop = args.target_current is not None

    if closed_loop and args.channel not in CURRENT_SENSE_CHANNELS:
        print(f"[FAIL] --target-current needs a current-sense channel {CURRENT_SENSE_CHANNELS}, "
              f"got --channel {args.channel} (AIN2/AIN3 only carry the thermistor bridge)")
        sys.exit(1)

    if not closed_loop and args.code is None:
        print("[FAIL] --code is required unless --target-current is given")
        sys.exit(1)

    if not closed_loop and not 0 <= args.code <= MCP4728_MAX_CODE:
        print(f"[FAIL] --code must be 0-{MCP4728_MAX_CODE}, got {args.code}")
        sys.exit(1)

    if closed_loop and not 0 < args.filter_alpha <= 1:
        print(f"[FAIL] --filter-alpha must be in (0, 1], got {args.filter_alpha}")
        sys.exit(1)

    if closed_loop and args.oversample < 1:
        print(f"[FAIL] --oversample must be >= 1, got {args.oversample}")
        sys.exit(1)

    start_code = 0 if args.code is None else max(0, min(MCP4728_MAX_CODE, args.code))
    sense_resistor_ohm = (
        args.sense_resistor_ohm if args.sense_resistor_ohm is not None
        else CURRENT_SENSE_RESISTOR_OHM.get(args.channel)
    )

    try:
        import smbus2
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2")
        sys.exit(1)

    try:
        ldac = Ldac(args.ldac_pin, enabled=not args.no_ldac)
    except RuntimeError as e:
        print(f"[FAIL] {e}")
        sys.exit(1)

    mcp4728_addr = int(args.mcp4728_addr, 0)
    ads1115_addr = int(args.ads1115_addr, 0)

    try:
        bus = smbus2.SMBus(int(args.bus.replace("/dev/i2c-", "")))
    except Exception as e:
        print(f"[FAIL] Could not open {args.bus}: {e}")
        ldac.close()
        sys.exit(1)

    try:
        mcp4728_multi_write_channel(bus, mcp4728_addr, args.channel, start_code,
                                     vref_vdd=True, gain=1)
    except OSError as e:
        print(f"[FAIL] Could not write MCP4728 at 0x{mcp4728_addr:02X}: {e}")
        ldac.close()
        bus.close()
        sys.exit(1)

    ch_letter = chr(ord('A') + args.channel)
    if args.channel in (2, 3):
        print(f"[WARN] --channel {args.channel} reads AIN{args.channel} single-ended, which shares "
              f"a physical pin with the AIN2-AIN3 thermistor bridge — that reading will reflect "
              f"the bridge node voltage, not an independent signal.")

    mux_bits = _ADS1115_MUX_SINGLE[args.channel]

    if closed_loop:
        print(f"[OK] MCP4728 channel {ch_letter} starting at code {start_code}/{MCP4728_MAX_CODE}, "
              f"current-hold engaged: target={args.target_current:.4f} A "
              f"(sense_r={sense_resistor_ohm} ohm kp={args.kp} ki={args.ki} "
              f"deadband={args.deadband_a} A max_step={args.max_code_step} "
              f"oversample={args.oversample} filter_alpha={args.filter_alpha})")
        print(f"Control loop averages {args.oversample}x 860 SPS AIN{args.channel} conversions "
              f"(trimmed mean) per iteration to reject the RF-coupled gate noise; thermistor bridge + "
              f"status print throttled to interval={args.interval}s, Ctrl+C to stop\n")

        current_code = start_code
        loop = CurrentHoldLoop(
            target_a=args.target_current, kp=args.kp, ki=args.ki,
            deadband_a=args.deadband_a, max_code_step=args.max_code_step,
            filter_alpha=args.filter_alpha,
        )
        last_print = 0.0

        try:
            while True:
                try:
                    sampled_a = read_current_a_oversampled(
                        bus, ads1115_addr, mux_bits, gain=args.ads1115_gain,
                        oversample=args.oversample, sense_resistor_ohm=sense_resistor_ohm,
                    )
                except (OSError, TimeoutError) as e:
                    print(f"[FAIL] ADS1115 current read error: {e}")
                    time.sleep(args.interval)
                    continue

                new_code, filtered_a, settled = loop.step(sampled_a, current_code)
                if new_code != current_code:
                    try:
                        mcp4728_multi_write_channel(bus, mcp4728_addr, args.channel, new_code,
                                                     vref_vdd=True, gain=1)
                        current_code = new_code
                    except OSError as e:
                        print(f"[FAIL] Could not write MCP4728 at 0x{mcp4728_addr:02X}: {e}")
                        continue

                now = time.monotonic()
                if now - last_print >= args.interval:
                    last_print = now
                    try:
                        therm_code = ads1115_one_shot_read_mux(bus, ads1115_addr, _ADS1115_MUX_DIFF_2_3,
                                                                gain=args.therm_gain)
                        vdiff = ads1115_code_to_volts(therm_code, gain=args.therm_gain)
                        t_c = resistance_to_celsius(bridge_volts_to_resistance(vdiff))
                        therm_str = f"T={t_c:6.2f} C"
                    except (OSError, TimeoutError) as e:
                        therm_str = f"T=ERR({e})"

                    ts = time.strftime("%H:%M:%S")
                    error_a = args.target_current - filtered_a
                    status = "settled" if settled else "correcting"
                    print(f"[{ts}] CH{ch_letter} code={current_code:4d}  "
                          f"target={args.target_current:.4f} A measured={filtered_a:.4f} A "
                          f"(sample={sampled_a:.4f}) err={error_a:+.4f} A [{status}]  {therm_str}")
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            try:
                mcp4728_multi_write_channel(bus, mcp4728_addr, args.channel, 0,
                                             vref_vdd=True, gain=1)
                print(f"CH{ch_letter} reset to code 0")
            except OSError as e:
                print(f"[FAIL] Could not reset MCP4728 channel {ch_letter} to 0: {e}")
            ldac.close()
            bus.close()
            print("Done")
        return

    print(f"[OK] MCP4728 channel {ch_letter} set to code {start_code}/{MCP4728_MAX_CODE}")
    print(f"Monitoring ADS1115 at 0x{ads1115_addr:02X}: thermistor bridge (AIN2-AIN3, differential) "
          f"and AIN{args.channel} (single-ended), interval={args.interval}s, Ctrl+C to stop\n")

    try:
        while True:
            try:
                therm_code = ads1115_one_shot_read_mux(bus, ads1115_addr, _ADS1115_MUX_DIFF_2_3,
                                                        gain=args.therm_gain)
                vdiff = ads1115_code_to_volts(therm_code, gain=args.therm_gain)
                r = bridge_volts_to_resistance(vdiff)
                t_c = resistance_to_celsius(r)
            except (OSError, TimeoutError) as e:
                print(f"[FAIL] Thermistor bridge read error: {e}")
                time.sleep(args.interval)
                continue

            try:
                led_code = ads1115_one_shot_read_mux(bus, ads1115_addr, mux_bits, gain=args.ads1115_gain)
                led_v = ads1115_code_to_volts(led_code, gain=args.ads1115_gain)
            except (OSError, TimeoutError) as e:
                print(f"[FAIL] ADS1115 read error: {e}")
                time.sleep(args.interval)
                continue

            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] CH{ch_letter} code={start_code:4d}  "
                  f"TH1: Vdiff={vdiff:+.6f} V R={r:8.1f} ohm T={t_c:6.2f} C  |  "
                  f"AIN{args.channel}: {led_v:.4f} V")

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            mcp4728_multi_write_channel(bus, mcp4728_addr, args.channel, 0,
                                         vref_vdd=True, gain=1)
            print(f"CH{ch_letter} reset to code 0")
        except OSError as e:
            print(f"[FAIL] Could not reset MCP4728 channel {ch_letter} to 0: {e}")
        ldac.close()
        bus.close()
        print("Done")


if __name__ == "__main__":
    main()
