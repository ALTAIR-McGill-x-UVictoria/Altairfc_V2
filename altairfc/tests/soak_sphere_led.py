"""Phase 1: quantify the sphere LEDs' brightness and temperature stability.

The sphere's R/G/B LEDs have no per-colour control — they are always driven
together as one combined optical output, with one current sense and one
thermistor for the whole board (see drivers/sphere_led.py). What DOES exist is
two independently addressable drive channels for that combined output: a
high-power channel (MCP4728 A) and a low-power channel (MCP4728 B) — select
with --channel. DAC codes on both are hard-limited to 1400 by the driver
itself, unconditionally, regardless of what --code requests.

This soaks the source at a fixed operating point for a long run and logs, once
a second, everything needed to quantify its drift:

    dac_code, led_current_a       what the driver is actually delivering
    bridge_temperature_c          the LED board's temperature
    sgt_tia_v, sol_tia_v          the sphere photodiodes — optical output
    external_pd_v                 the goniometer detector, if it is set up

Run the same operating point twice, once in each mode, to see what the current
loop buys you:

    python tests/soak_sphere_led.py --code 700 --mode open \\
        --duration 3600 --csv lab_data/soak_open.csv

    python tests/soak_sphere_led.py --code 700 --mode current \\
        --target-current 0.35 --duration 3600 --csv lab_data/soak_pi.csv

The warm-up transient in the first run gives the dI/dT that
Experiment_Design/01_source_calibration.md requires under "Source stability",
and pairs (bridge_temperature_c, sgt_tia_v) are what the fit is over.

Periodic LED-off dark frames (--dark-interval, default every 300 s) let the
reducer track and subtract a *drifting* background rather than assume a static
one. That matters because the external photodiode looks through a 2 mm aperture
at 50 cm, so its signal is small enough for ambient light to be a large
fractional contamination — and over an hour, room light is rarely constant.
Set --dark-interval 0 to disable.

The CSV schema is a superset of the goniometric scan's — the angle columns are
simply blank — so reduce_goniometer.py can be validated against five minutes
of soak data before committing to a multi-hour scan.

Run from altairfc/.  Requires: sudo pigpiod
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.sphere_led import (  # noqa: E402
    HIGH_POWER_CHANNEL,
    LOW_POWER_CHANNEL,
    MAX_SAFE_CODE,
)
from tests.sphere_rig import LED_DARK_LABEL, LED_LIT_LABEL, RigCsvWriter, SphereRig  # noqa: E402

logger = logging.getLogger(__name__)

_CHANNEL_CHOICES = {"high": HIGH_POWER_CHANNEL, "low": LOW_POWER_CHANNEL}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Soak the sphere's combined LED source at a fixed operating point "
        "and log its stability."
    )
    parser.add_argument(
        "--channel", choices=sorted(_CHANNEL_CHOICES), default="high",
        help="Which drive channel to soak: high power (MCP4728 A) or low power (MCP4728 B)",
    )
    parser.add_argument(
        "--code", type=int, default=700,
        help=f"Starting DAC code (0-{MAX_SAFE_CODE}; hard-clamped by the driver "
        "regardless of what's passed here)",
    )
    parser.add_argument(
        "--mode",
        choices=["open", "current"],
        default="open",
        help="open: hold the DAC code fixed. current: PI-hold the measured drive current",
    )
    parser.add_argument(
        "--target-current",
        type=float,
        default=None,
        help="Current setpoint in amps for --mode current. "
        "Defaults to whatever the LED draws once settled at --code",
    )
    parser.add_argument("--duration", type=float, default=3600.0, help="Run length in seconds")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between samples")
    parser.add_argument("--csv", type=Path, required=True, help="Output CSV path")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number")
    parser.add_argument("--i2c-dev", default="/dev/i2c-1", help="I2C device node for the MCP4728")
    parser.add_argument(
        "--no-ldac", action="store_true", help="Don't drive LDAC (use if hardwired to GND)"
    )
    parser.add_argument(
        "--no-pdro", action="store_true", help="Skip the two sphere photodiodes (UVIC PDRO)"
    )
    parser.add_argument(
        "--no-external", action="store_true", help="Skip the external ADS1220 photodiode"
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=10.0,
        help="Seconds to hold --code before reading the default current setpoint",
    )
    parser.add_argument("--kp", type=float, default=None, help="PI proportional gain, codes/A")
    parser.add_argument("--ki", type=float, default=None, help="PI integral gain, codes/(A*s)")
    parser.add_argument(
        "--dark-interval",
        type=float,
        default=300.0,
        help="Seconds between LED-off dark frames; 0 disables them. Lets the reducer "
        "track and subtract a drifting background instead of assuming a static one",
    )
    parser.add_argument(
        "--dark-samples", type=int, default=5, help="Samples per dark frame"
    )
    parser.add_argument(
        "--dark-settle",
        type=float,
        default=2.0,
        help="Seconds to wait after switching the LED off, and again after switching it "
        "back on, before sampling",
    )
    return parser


def sample_dark(rig, writer, *, args, start) -> None:
    """Take and log one set of background samples, reporting the level live.

    Printed as well as written so the background can be judged at the bench
    without running the reducer — for an "are the room lights a problem?" check
    the answer is the dark level next to the lit level on the previous line.
    """
    channels: dict[str, list[float]] = {"ext": [], "sgt": [], "sol": []}
    for i in range(args.dark_samples):
        try:
            reading = rig.sample()
        except (OSError, TimeoutError) as e:
            print(f"[WARN] dark frame read error: {e}", file=sys.stderr)
            continue
        writer.write(
            reading,
            elapsed_s=time.monotonic() - start,
            mode=args.mode,
            led=LED_DARK_LABEL,
            sample_index=i,
            target_current_a=None,
        )
        for key, value in (
            ("ext", reading.external_pd_v),
            ("sgt", reading.sgt_tia_v),
            ("sol", reading.sol_tia_v),
        ):
            if value is not None:
                channels[key].append(value)

    means = {k: (sum(v) / len(v) if v else None) for k, v in channels.items()}
    print(
        f"[{time.monotonic() - start:8.1f}s] DARK  "
        f"sgt={_fmt(means['sgt'])} sol={_fmt(means['sol'])} ext={_fmt(means['ext'])}"
    )


def dark_frame(rig, writer, *, code, target, args, start) -> None:
    """Switch the LED off, sample the background, and restore the operating point.

    A dark frame perturbs the very thermal equilibrium the soak is measuring, so
    keep it short relative to the LED's thermal time constant: the defaults are
    ~9 s of darkness every 300 s, roughly a 3% duty cycle. The DAC code is saved
    and restored rather than re-derived so the source returns to exactly where
    it was, and the current loop is re-engaged afterwards.
    """
    rig.led.all_off()  # also disengages the current loop
    time.sleep(args.dark_settle)

    sample_dark(rig, writer, args=args, start=start)

    rig.led.set_code(code)
    if target is not None:
        rig.led.hold_current(target, kp=args.kp, ki=args.ki)
    time.sleep(args.dark_settle)


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    args = build_parser().parse_args()

    if not 0 <= args.code <= MAX_SAFE_CODE:
        print(f"[FAIL] --code must be 0-{MAX_SAFE_CODE}, got {args.code}", file=sys.stderr)
        return 1

    try:
        import smbus2
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2", file=sys.stderr)
        return 1

    bus = smbus2.SMBus(args.bus)

    try:
        rig = SphereRig(
            bus=bus,
            channel=_CHANNEL_CHOICES[args.channel],
            i2c_dev=args.i2c_dev,
            use_ldac=not args.no_ldac,
            use_pdro=not args.no_pdro,
            use_external=not args.no_external,
        )
    except Exception as e:
        print(f"[FAIL] Could not open the rig: {e}", file=sys.stderr)
        bus.close()
        return 1

    writer = RigCsvWriter(args.csv)
    start = time.monotonic()

    try:
        if args.dark_interval > 0:
            # Free baseline: the LEDs are still off at this point, so this frame
            # costs no thermal disturbance at all.
            print(f"[OK] Taking initial dark frame ({args.dark_samples} samples)")
            sample_dark(rig, writer, args=args, start=start)

        rig.led.set_code(args.code)
        print(f"[OK] LEDs set to code {args.code}; settling {args.settle_s:.0f} s")
        time.sleep(args.settle_s)

        target = None
        if args.mode == "current":
            target = args.target_current
            if target is None:
                target = rig.led.read_current_a()
                print(f"[OK] Using settled draw as setpoint: {target:.4f} A")
            rig.led.hold_current(target, kp=args.kp, ki=args.ki)

        print(
            f"Logging every {args.interval:.2f} s for {args.duration:.0f} s "
            f"to {args.csv} — Ctrl+C to stop early\n"
        )

        sample_index = 0
        deadline = start + args.duration
        next_dark = (
            time.monotonic() + args.dark_interval if args.dark_interval > 0 else float("inf")
        )
        while time.monotonic() < deadline:
            loop_start = time.monotonic()

            if loop_start >= next_dark:
                dark_frame(
                    rig, writer,
                    code=args.code, target=target, args=args, start=start,
                )
                next_dark = time.monotonic() + args.dark_interval
                continue

            try:
                reading = rig.sample()
            except (OSError, TimeoutError) as e:
                # A flaky I2C/SPI read should cost one sample, not the run.
                print(f"[WARN] read error, skipping sample: {e}", file=sys.stderr)
                time.sleep(args.interval)
                continue

            elapsed = loop_start - start
            writer.write(
                reading,
                elapsed_s=elapsed,
                mode=args.mode,
                led=LED_LIT_LABEL,
                sample_index=sample_index,
                target_current_a=target,
            )

            print(
                f"[{elapsed:8.1f}s] code={reading.dac_code:4d} "
                f"I={reading.led_current_a:.4f} A  T={reading.bridge_temperature_c:6.2f} C  "
                f"sgt={_fmt(reading.sgt_tia_v)} sol={_fmt(reading.sol_tia_v)} "
                f"ext={_fmt(reading.external_pd_v)}"
            )

            sample_index += 1
            remaining = args.interval - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nStopping early...")
    finally:
        rig.close()
        writer.close()
        bus.close()
        print(f"Done — {writer.rows} rows written to {args.csv}")

    return 0


def _fmt(value: float | None) -> str:
    return "  --  " if value is None else f"{value:.5f}"


if __name__ == "__main__":
    raise SystemExit(main())
