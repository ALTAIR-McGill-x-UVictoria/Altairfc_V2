"""
INA3221 hardware verification script.

Runs directly on the Pi with the INA3221 wired at I2C address 0x40.
Requires libina3221_driver.so to be built first:
    bash altairfc/drivers/build_ina3221.sh

Usage:
    python tests/test_ina3221.py [--bus /dev/i2c-1] [--samples 5]

Channel map:
    CH1 → 24 V rail
    CH2 → 12 V rail
    CH3 →  5 V rail

Checks performed:
    1. Device responds at 0x40
    2. Manufacturer ID reads 0x5449 ("TI")
    3. Die ID reads 0x2210 (INA3221)
    4. Config register round-trip (write then read back)
    5. N consecutive samples: voltages and currents are finite,
       voltages are in plausible ranges, no read errors
"""

import argparse
import sys
import time

ADDR        = 0x40
REG_CONFIG  = 0x00
REG_MANUF   = 0xFE
REG_DIE     = 0xFF

EXPECTED_MANUF_ID = 0x5449   # "TI"
EXPECTED_DIE_ID   = 0x2210   # INA3221

INA3221_CONFIG = 0x7127      # same value written by the C driver

RAIL_NAMES = ("24V", "12V", "5V")
VOLTAGE_RANGES = (
    (18.0, 30.0),   # 24 V rail: warn outside this window
    ( 9.0, 14.0),   # 12 V rail
    ( 4.5,  5.5),   # 5 V rail
)


def read_reg16(bus, reg):
    """Read a big-endian 16-bit register via smbus2."""
    raw = bus.read_i2c_block_data(ADDR, reg, 2)
    return (raw[0] << 8) | raw[1]


def write_reg16(bus, reg, value):
    bus.write_i2c_block_data(ADDR, reg, [(value >> 8) & 0xFF, value & 0xFF])


def shunt_raw_to_uv(raw):
    """Bits [15:3], sign-extended, × 40 µV."""
    signed = (raw & 0xFFF8)
    if signed & 0x8000:
        signed -= 0x10000
    return (signed >> 3) * 40.0


def bus_raw_to_mv(raw):
    """Bits [15:3], unsigned, × 8 mV."""
    return ((raw & 0xFFF8) >> 3) * 8.0


def check_device_present(bus):
    try:
        bus.read_byte(ADDR)
        print(f"[OK] Device responds at 0x{ADDR:02X}")
        return True
    except OSError:
        print(f"[FAIL] No device at 0x{ADDR:02X} — check wiring and I2C bus")
        return False


def check_manuf_id(bus):
    manuf = read_reg16(bus, REG_MANUF)
    if manuf == EXPECTED_MANUF_ID:
        print(f"[OK] Manufacturer ID = 0x{manuf:04X} (Texas Instruments)")
        return True
    print(f"[FAIL] Manufacturer ID = 0x{manuf:04X}, expected 0x{EXPECTED_MANUF_ID:04X}")
    return False


def check_die_id(bus):
    die = read_reg16(bus, REG_DIE)
    if die == EXPECTED_DIE_ID:
        print(f"[OK] Die ID = 0x{die:04X} (INA3221)")
        return True
    print(f"[FAIL] Die ID = 0x{die:04X}, expected 0x{EXPECTED_DIE_ID:04X}")
    return False


def check_config_roundtrip(bus):
    write_reg16(bus, REG_CONFIG, INA3221_CONFIG)
    time.sleep(0.01)
    readback = read_reg16(bus, REG_CONFIG)
    if readback == INA3221_CONFIG:
        print(f"[OK] Config register round-trip = 0x{readback:04X}")
        return True
    print(f"[FAIL] Config round-trip: wrote 0x{INA3221_CONFIG:04X}, read 0x{readback:04X}")
    return False


def check_samples(bus, n_samples, shunt_ohms=0.01):
    """Read N samples from all channels and verify plausibility."""
    print(f"\n--- Taking {n_samples} sample(s) (shunt = {shunt_ohms} Ω) ---")

    shunt_regs = [0x01, 0x03, 0x05]
    bus_regs   = [0x02, 0x04, 0x06]

    errors = 0
    range_warnings = 0

    for i in range(n_samples):
        sample_ok = True
        for ch in range(3):
            try:
                shunt_raw = read_reg16(bus, shunt_regs[ch])
                bus_raw   = read_reg16(bus, bus_regs[ch])
            except OSError as e:
                print(f"  [FAIL] Sample {i+1} CH{ch+1} I2C error: {e}")
                errors += 1
                sample_ok = False
                continue

            shunt_uv = shunt_raw_to_uv(shunt_raw)
            bus_mv   = bus_raw_to_mv(bus_raw)
            voltage  = bus_mv / 1000.0
            current  = (shunt_uv / 1e6) / shunt_ohms

            lo, hi = VOLTAGE_RANGES[ch]
            in_range = lo <= voltage <= hi
            flag = "" if in_range else f"  ← WARN expected {lo}–{hi} V"
            if not in_range:
                range_warnings += 1

            print(f"  Sample {i+1}  CH{ch+1} ({RAIL_NAMES[ch]:>3s}): "
                  f"{voltage:6.3f} V  {current:7.3f} A{flag}")

        if sample_ok and i < n_samples - 1:
            time.sleep(0.05)

    all_ok = errors == 0
    if all_ok:
        print(f"[OK] All {n_samples * 3} channel reads succeeded")
    else:
        print(f"[FAIL] {errors} read error(s) across {n_samples} samples")
    if range_warnings:
        print(f"[WARN] {range_warnings} channel reading(s) outside expected voltage range "
              f"(rails may be unpowered or lightly loaded)")
    return all_ok


def main():
    parser = argparse.ArgumentParser(description="INA3221 hardware verification")
    parser.add_argument("--bus",     default="/dev/i2c-1", help="I2C device node")
    parser.add_argument("--samples", default=5, type=int,  help="Number of read samples")
    args = parser.parse_args()

    print(f"=== INA3221 verification at 0x{ADDR:02X} on {args.bus} ===\n")

    try:
        import smbus2
        bus = smbus2.SMBus(int(args.bus.replace("/dev/i2c-", "")))
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2")
        sys.exit(1)
    except Exception as e:
        print(f"[FAIL] Could not open {args.bus}: {e}")
        sys.exit(1)

    results = []

    results.append(check_device_present(bus))
    if not results[-1]:
        print("\nDevice not found, aborting.")
        bus.close()
        sys.exit(1)

    results.append(check_manuf_id(bus))
    results.append(check_die_id(bus))

    if not all(results):
        print("\nID checks failed — may not be an INA3221, aborting.")
        bus.close()
        sys.exit(1)

    results.append(check_config_roundtrip(bus))
    results.append(check_samples(bus, args.samples))

    bus.close()

    print(f"\n=== Results: {sum(results)}/{len(results)} checks passed ===")
    if all(results):
        print("INA3221 verified OK")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
