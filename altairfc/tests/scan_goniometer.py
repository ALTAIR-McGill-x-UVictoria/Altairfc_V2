"""Far-field goniometric scan of the integrating sphere's exit port.

Sweeps the sphere through one hemisphere of emission directions on the
two-axis servo jig while a fixed photodiode measures the port's brightness,
producing the raw data for I(theta, phi). Requirements come from
ALTAIR-analysis/Experiment_Design/01_source_calibration.md; the ones that
shape this script:

  * Both angles are scanned. Polar theta from the port normal and azimuth phi
    about it. Scanning several azimuths is a symmetry *check*, not padding —
    if it fails, the deliverable becomes a 2D map and the flight pipeline needs
    gondola roll at each flash.
  * Every point carries an error bar, so --samples repeats per angle and each
    sample is its own CSV row.

IMPORTANT LIMITATION, not fixable in software: the sphere's R/G/B LEDs have no
per-colour control (see drivers/sphere_led.py) — they are always driven
together as one combined unit. Experiment_Design/01 section 2 step 3 requires
angular response measured PER WAVELENGTH, independently. This scan cannot do
that; it measures I(theta, phi) of the combined spectrum only. Getting a true
per-wavelength curve needs either independently switchable LED drivers (a
board change) or a spectrally-resolving detector at the goniometer stage,
neither of which exists yet. Treat any curve this script produces as the
combined-spectrum response, not a per-wavelength one, until that changes.

Three things the requirements imply but do not spell out, all done here:

  * A dark frame opens every polar (nadir) arm (LEDs off, same sample count,
    led="dark"). At a 2 mm aperture and 50 cm this is not optional.
  * The first angle is repeated at the end of the scan, bounding source drift
    across the run — the within-scan version of the doc's before/after check.
  * Rows are flushed as they are measured, so an interrupted scan keeps its data.

Move order: polar (nadir) is the OUTER loop, azimuth is the INNER loop — for
each nadir angle, sweep every azimuth before moving nadir again. The polar
axis (servo b / BCM 26 by default) is the harder motion on this jig, so it
only moves once per nadir angle instead of once per sample; azimuth (servo a
/ BCM 16) absorbs the frequent moves.

Usage:
    python tests/scan_goniometer.py --dry-run --polar -90:90:15 --azimuth -90:90:90
    python tests/scan_goniometer.py --home
    python tests/scan_goniometer.py --polar -90:90:5 --azimuth -90:90:45 \\
        --settle 2.0 --samples 20 --csv scan_2026-07-27.csv

Budget the run. ads1220_read_single_shot() is not DRDY-driven: it sleeps a
fixed ~75 ms per conversion at 20 SPS (ads1220_driver.c), so 20 samples costs
~1.5 s of external-photodiode reads alone before settle time. --dry-run
prints the resulting estimate; check it before starting.

Run from altairfc/. Requires: sudo pigpiod
"""

from __future__ import annotations

import argparse
import logging
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.goniometer import (  # noqa: E402
    DEFAULT_CALIBRATION_PATH,
    GoniometerStage,
    load_calibration,
    save_calibration,
)
from drivers.sphere_led import (  # noqa: E402
    HIGH_POWER_CHANNEL,
    LOW_POWER_CHANNEL,
    MAX_SAFE_CODE,
)
from tests.sphere_rig import LED_DARK_LABEL, LED_LIT_LABEL, RigCsvWriter, SphereRig  # noqa: E402

_CHANNEL_CHOICES = {"high": HIGH_POWER_CHANNEL, "low": LOW_POWER_CHANNEL}

logger = logging.getLogger(__name__)

REPEAT_LABEL = f"{LED_LIT_LABEL}_repeat"  # must match reduce_goniometer.py in NRC-calibration
EXTERNAL_READ_S = 0.075  # fixed sleep inside ads1220_read_single_shot()


def parse_range(spec: str) -> list[float]:
    """Parse 'start:stop:step' into an inclusive list of angles.

    A single value ('0') or a zero/absent step yields one angle, so a
    single-point scan is 'start:start:1'.
    """
    parts = spec.split(":")
    if len(parts) == 1:
        return [float(parts[0])]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"expected 'start:stop:step', got {spec!r}")
    start, stop, step = (float(p) for p in parts)
    if step <= 0:
        raise argparse.ArgumentTypeError(f"step must be positive, got {step}")
    if start == stop:
        return [start]
    if stop < start:
        raise argparse.ArgumentTypeError(f"stop must be >= start, got {spec!r}")

    values = []
    n = int(round((stop - start) / step))
    for i in range(n + 1):
        values.append(start + i * step)
    # Include the endpoint when the step does not divide the span evenly.
    if abs(values[-1] - stop) > 1e-9:
        values.append(stop)
    return values


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Far-field goniometric scan of the sphere exit port's combined-spectrum "
        "angular response. See the module docstring: this cannot resolve per-wavelength "
        "response, since the sphere's LEDs have no independent colour control."
    )
    # argparse's built-in check for "does this token look like a negative
    # number rather than an unknown option" only matches plain integers/
    # decimals (e.g. -90), not our 'start:stop:step' range syntax (e.g.
    # -90:90:5). Without this, `--polar -90:90:5` (the natural, space-
    # separated form used everywhere in this repo's docs) is misparsed as an
    # unrecognized option and argparse fails with "expected one argument" —
    # only `--polar=-90:90:5` would work. Broadening the matcher to any
    # '-<digit>...' token fixes the space-separated form; no option name here
    # starts with a digit, so nothing else is affected.
    parser._negative_number_matcher = re.compile(r"^-\d.*$")
    parser.add_argument(
        "--polar", type=parse_range, default="-90:90:15",
        help="Polar theta range from the port normal, 'start:stop:step' in degrees",
    )
    parser.add_argument(
        "--azimuth", type=parse_range, default="-90:90:45",
        help="Azimuth phi range about the normal, 'start:stop:step' in degrees",
    )
    parser.add_argument(
        "--channel", choices=sorted(_CHANNEL_CHOICES), default="high",
        help="Which drive channel to scan with: high power (MCP4728 A) or low power (MCP4728 B)",
    )
    parser.add_argument(
        "--code", type=int, default=700,
        help=f"DAC code (0-{MAX_SAFE_CODE}; hard-clamped by the driver regardless of "
        "what's passed here)",
    )
    parser.add_argument(
        "--target-current", type=float, default=None,
        help="Hold this drive current (amps) with the PI loop during the scan "
        "instead of a fixed DAC code — recommended for long scans",
    )
    parser.add_argument("--samples", type=int, default=20, help="Samples per angle")
    parser.add_argument("--settle", type=float, default=2.0, help="Dwell after each move, seconds")
    parser.add_argument(
        "--warmup", type=float, default=60.0,
        help="Seconds to hold the LEDs lit before the first measurement",
    )
    parser.add_argument("--slew-rate", type=float, default=30.0, help="Stage slew rate, deg/s")
    parser.add_argument("--csv", type=Path, default=None, help="Output CSV path")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number")
    parser.add_argument("--i2c-dev", default="/dev/i2c-1", help="I2C device node for the MCP4728")
    parser.add_argument("--no-ldac", action="store_true", help="Don't drive LDAC")
    parser.add_argument("--no-pdro", action="store_true", help="Skip the two sphere photodiodes")
    parser.add_argument("--no-external", action="store_true", help="Skip the external photodiode")
    parser.add_argument(
        "--no-dark", action="store_true", help="Skip the per-arm dark frames (not recommended)"
    )
    parser.add_argument(
        "--no-repeat", action="store_true", help="Skip the end-of-scan drift-check repeat"
    )
    parser.add_argument(
        "--calibration", type=Path, default=DEFAULT_CALIBRATION_PATH,
        help="Axis calibration JSON written by --home",
    )
    parser.add_argument(
        "--home", action="store_true",
        help="Interactive axis homing: set the mechanical zeros and save them, then exit",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print the sequence, row count and duration estimate; touch no hardware",
    )
    return parser


def estimate_seconds(args, n_points: int, n_dark: int) -> float:
    """Rough wall-clock estimate: moves, settles, and per-sample read cost."""
    per_sample = 0.0
    if not args.no_external:
        per_sample += EXTERNAL_READ_S
    if not args.no_pdro:
        per_sample += 0.02  # two ADS124S08 conversions at SPS_100 plus relay overhead
    per_sample += 0.05  # ADS1115 current + bridge, one-shot each

    dwell = args.samples * per_sample
    moves = (n_points + n_dark) * args.settle
    return moves + (n_points + n_dark) * dwell + args.warmup


def describe(args) -> tuple[int, int]:
    """Print the estimated run time up front, then the planned sequence.

    Returns (measurement points, dark frames).
    """
    n_points = len(args.azimuth) * len(args.polar)
    n_dark = 0 if args.no_dark else len(args.polar)  # one dark frame per nadir arm
    if not args.no_repeat:
        n_points += 1

    print(f"Estimated run time: {estimate_seconds(args, n_points, n_dark) / 60.0:.1f} minutes\n")

    print(
        f"Source    : combined R+G+B, {args.channel}-power channel, "
        f"code {args.code} (hard-capped at {MAX_SAFE_CODE}) — no per-colour control"
    )
    print(f"Azimuth   : {len(args.azimuth)} x {_fmt_list(args.azimuth)} deg")
    print(f"Polar     : {len(args.polar)} x {_fmt_list(args.polar)} deg")
    print(f"Samples   : {args.samples} per angle")
    print(f"Points    : {n_points} measurement + {n_dark} dark")
    print(f"Rows      : {(n_points + n_dark) * args.samples}")
    if len(args.azimuth) < 2:
        print(
            "\n[WARN] Only one azimuth. The azimuthal-symmetry check requires at least two,\n"
            "       and without it the I(theta) collapse cannot be justified\n"
            "       (Experiment_Design/01_source_calibration.md, section 2 step 4)."
        )
    print(
        "\n[NOTE] This measures the ANGULAR RESPONSE OF THE COMBINED RGB SPECTRUM.\n"
        "       Experiment_Design/01 section 2 step 3 requires per-wavelength curves;\n"
        "       the current LED board cannot isolate a single colour to provide that."
    )
    return n_points, n_dark


def _fmt_list(values: list[float]) -> str:
    if len(values) <= 4:
        return ", ".join(f"{v:g}" for v in values)
    return f"{values[0]:g} .. {values[-1]:g}"


HOME_REFERENCE = {
    "polar": (0.0, "the exit port normal pointing straight at the detector"),
    "azimuth": (0.0, "the jig's azimuth fiducial mark"),
}


def run_home(args) -> int:
    """Interactively establish each axis' mechanical mapping and persist it.

    You jog in raw servo degrees until the stage is physically at a known
    reference angle, then accept it. That pairing fixes ``center_deg``, so the
    stage angles the scan records afterwards mean what they say.
    """
    polar_cal, azimuth_cal = load_calibration(args.calibration)
    stage = GoniometerStage(polar_cal, azimuth_cal, slew_rate_deg_s=args.slew_rate, settle_s=0.0)
    if not stage.connect():
        return 1

    print(
        "\nHoming. Each axis starts at its servo centre (90 deg).\n"
        "Enter a servo angle (0-180) to jog to, 'a' to accept the current physical\n"
        "position as this axis' reference, or 'q' to leave the axis unchanged.\n"
        "Watch the jig; keep --slew-rate low on the first run.\n"
    )
    try:
        for name in ("polar", "azimuth"):
            cal = stage.polar if name == "polar" else stage.azimuth
            reference_deg, description = HOME_REFERENCE[name]
            servo = 90.0
            stage.jog_servo(name, servo)
            print(f"\n{name}: jog until the stage is at {reference_deg:+.1f} deg — {description}")

            while True:
                raw = input(f"  {name} servo [{servo:.1f}]> ").strip().lower()
                if raw in ("q", "quit"):
                    break
                if raw in ("a", "accept"):
                    updated = cal.recentered(servo, reference_deg)
                    if name == "polar":
                        stage.polar = updated
                    else:
                        stage.azimuth = updated
                    lo, hi = updated.reachable_range()
                    clipped = lo > updated.min_deg + 1e-9 or hi < updated.max_deg - 1e-9
                    print(
                        f"  {name} centre set to {updated.center_deg:+.2f} deg; "
                        f"reachable travel [{lo:+.1f}, {hi:+.1f}] deg"
                        + (
                            f"  (clipped from configured [{updated.min_deg:+.1f}, "
                            f"{updated.max_deg:+.1f}] by servo 0-180 limits — a mechanical "
                            "offset was found during homing; use a --polar/--azimuth range "
                            "inside the reachable interval above)"
                            if clipped
                            else ""
                        )
                    )
                    break
                try:
                    servo = max(0.0, min(180.0, float(raw)))
                except ValueError:
                    print("  enter a servo angle 0-180, 'a' to accept, or 'q' to skip")
                    continue
                stage.jog_servo(name, servo)
    except KeyboardInterrupt:
        print("\nHoming interrupted — saving whatever axes were accepted before this point")
        return 1
    except Exception:
        logger.exception("Unexpected error during homing — saving whatever was accepted so far")
        return 1
    finally:
        # Whatever got accepted (via 'a') is already on stage.polar/stage.azimuth
        # regardless of how this block exits, so save unconditionally: losing a
        # jig homing session to an unrelated bug is expensive to redo.
        save_calibration(stage.polar, stage.azimuth, args.calibration)
        print(f"Saved to {args.calibration}")
        stage.park()
    return 0


def measure_point(
    rig: SphereRig,
    writer: RigCsvWriter,
    *,
    led_label: str,
    polar_deg: float,
    azimuth_deg: float,
    samples: int,
    start: float,
    mode: str,
    target_current_a: float | None,
) -> None:
    """Take ``samples`` readings at one stage position and write them all."""
    for i in range(samples):
        try:
            reading = rig.sample()
        except (OSError, TimeoutError) as e:
            print(f"[WARN] read error at theta={polar_deg:+.1f} phi={azimuth_deg:.1f}: {e}",
                  file=sys.stderr)
            continue
        writer.write(
            reading,
            elapsed_s=time.monotonic() - start,
            mode=mode,
            led=led_label,
            sample_index=i,
            polar_deg=polar_deg,
            azimuth_deg=azimuth_deg,
            target_current_a=target_current_a,
        )


def run_scan(args) -> int:
    try:
        import smbus2
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2", file=sys.stderr)
        return 1

    if args.csv is None:
        print("[FAIL] --csv is required for a real scan", file=sys.stderr)
        return 1

    polar_cal, azimuth_cal = load_calibration(args.calibration)
    stage = GoniometerStage(
        polar_cal, azimuth_cal, slew_rate_deg_s=args.slew_rate, settle_s=args.settle
    )
    if not stage.connect():
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
        stage.park()
        bus.close()
        return 1

    mode = "current" if args.target_current is not None else "open"
    writer = RigCsvWriter(args.csv)
    start = time.monotonic()

    try:
        print(f"\n=== combined R+G+B source, {args.channel}-power channel "
              "(no per-colour isolation available) ===")

        rig.led.all_off()
        rig.led.set_code(args.code)
        if args.warmup > 0:
            print(f"Warming up {args.warmup:.0f} s...")
            time.sleep(args.warmup)
        if args.target_current is not None:
            rig.led.hold_current(args.target_current)

        # Drift-check reference: the polar angle nearest normal incidence,
        # first azimuth. Deliberately not the first point measured, which is
        # the grazing edge of the sweep where there is least signal and a
        # drift ratio would be dominated by noise.
        reference_position = (min(args.polar, key=abs), args.azimuth[0])

        for polar_deg in args.polar:
            if not args.no_dark:
                # Dark frame for this arm: same optics, same detector, LEDs off.
                # Azimuth-only move — nadir (servo b) is already at polar_deg.
                stage.move_to(polar_deg, args.azimuth[0])
                rig.led.all_off()  # also disengages the current loop
                time.sleep(args.settle)
                measure_point(
                    rig, writer,
                    led_label=LED_DARK_LABEL,
                    polar_deg=polar_deg, azimuth_deg=args.azimuth[0],
                    samples=args.samples, start=start, mode=mode, target_current_a=None,
                )
                rig.led.set_code(args.code)
                if args.target_current is not None:
                    rig.led.hold_current(args.target_current)
                time.sleep(args.settle)

            for azimuth_deg in args.azimuth:
                stage.move_to(polar_deg, azimuth_deg)
                measure_point(
                    rig, writer,
                    led_label=LED_LIT_LABEL,
                    polar_deg=polar_deg, azimuth_deg=azimuth_deg,
                    samples=args.samples, start=start, mode=mode,
                    target_current_a=args.target_current,
                )
                print(
                    f"  theta={polar_deg:+7.2f} phi={azimuth_deg:6.2f}  "
                    f"({writer.rows} rows, {(time.monotonic() - start) / 60.0:5.1f} min)"
                )

        if not args.no_repeat:
            # Return to the reference angle: any change against the same point
            # measured at the start is source drift over the scan, not angular
            # response.
            print(
                f"  repeating theta={reference_position[0]:+g} "
                f"phi={reference_position[1]:g} for the drift check..."
            )
            stage.move_to(*reference_position)
            measure_point(
                rig, writer,
                led_label=REPEAT_LABEL,
                polar_deg=reference_position[0], azimuth_deg=reference_position[1],
                samples=args.samples, start=start, mode=mode,
                target_current_a=args.target_current,
            )

    except KeyboardInterrupt:
        print("\nScan interrupted — data written so far is intact")
        return 0
    except ValueError as e:
        # validate_reachable() should catch this before any motion starts;
        # this is a backstop (e.g. the calibration file changed mid-run), so
        # it fails clearly instead of with a raw traceback. Data already
        # written is intact either way, same as KeyboardInterrupt above.
        print(f"\n[FAIL] Scan stopped: {e}", file=sys.stderr)
        return 1
    finally:
        rig.close()
        stage.park()
        writer.close()
        bus.close()
        print(
            f"\nDone — {writer.rows} rows to {args.csv} "
            f"in {(time.monotonic() - start) / 60.0:.1f} minutes"
        )

    return 0


def validate_reachable(args) -> bool:
    """Check every requested --polar/--azimuth angle against the calibrated
    reachable range, printing a clear diagnosis for anything outside it.

    A mechanical offset found during --home (see AxisCalibration.reachable_range)
    can leave part of an axis's nominal travel unreachable. Without this check
    that only surfaces as GoniometerStage.move_to() raising ValueError partway
    through a real scan -- after some hardware motion and possibly minutes of
    good data already collected. Running this at describe()/--dry-run time
    catches it before anything moves.
    """
    polar_cal, azimuth_cal = load_calibration(args.calibration)
    ok = True
    for axis_name, cal, requested in (
        ("polar", polar_cal, args.polar),
        ("azimuth", azimuth_cal, args.azimuth),
    ):
        lo, hi = cal.reachable_range()
        bad = [a for a in requested if not lo <= a <= hi]
        if bad:
            ok = False
            print(
                f"[FAIL] --{axis_name} includes angle(s) outside the calibrated reachable "
                f"range [{lo:+.1f}, {hi:+.1f}] deg: {', '.join(f'{a:+g}' for a in bad)}\n"
                f"       Narrow --{axis_name} to stay inside that range, or re-run --home "
                "if the mechanical offset should no longer be there.",
                file=sys.stderr,
            )
    return ok


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    args = build_parser().parse_args()

    if args.home:
        return run_home(args)

    if not 0 <= args.code <= MAX_SAFE_CODE:
        print(f"[FAIL] --code must be 0-{MAX_SAFE_CODE}, got {args.code}", file=sys.stderr)
        return 1

    if not validate_reachable(args):
        return 1

    describe(args)
    if args.dry_run:
        return 0
    return run_scan(args)


if __name__ == "__main__":
    raise SystemExit(main())
