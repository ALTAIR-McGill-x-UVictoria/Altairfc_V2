"""Production tests for the complete UVIC PDRO and sphere LED system.

Run the automated, launch-day test sequence with::

    python tests/test_UVICPDRO_ADC.py --prelaunch
    python tests/test_UVICPDRO_ADC.py --prelaunch --manual-illumination

The sequence verifies ADC configuration readback and both temperature sensors,
checks for photodiode dark current at 3 V bias on each high-gain TIA, then
checks both low-gain TIA responses using either the integrating-sphere LEDs or
10 seconds of user-provided illumination.
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.uvic_pdro import (  # noqa: E402
    DataRate,
    Input,
    Readout,
    SignalPath,
    ThermistorReading,
    UVICPDRO,
)
from drivers.sphere_led import MAX_SAFE_CODE, SphereLedSource  # noqa: E402


READOUT_NAMES = {
    Readout.SERGEANT: "Sergeant",
    Readout.SOLDIER: "Soldier",
}

DEFAULT_BIAS_V = 3.0
DEFAULT_LED_CODE = 700
DEFAULT_SAMPLES = 5
DEFAULT_MIN_BIAS_DROP_V = 0.0
DEFAULT_MIN_LED_DROP_V = 0.1
DEFAULT_ADC_RAIL_MARGIN_V = 0.05
DEFAULT_SETTLE_S = 0.5
DEFAULT_MANUAL_ILLUMINATION_S = 10.0
DEFAULT_MONITOR_INTERVAL_S = 0.1
VOLTAGE_COMPARISON_EPSILON_V = 1e-6
ADC_MIN_V = 0.0
ADC_MAX_V = 5.0


def readout_name(readout: Readout) -> str:
    return f"{READOUT_NAMES[readout]} readout"


def check_range(name, val, expected_min, expected_max, unit="V"):
    if val is None:
        actual_str = "None"
        passed = False
    else:
        actual_str = f"{val:.8f} {unit}"
        passed = expected_min <= val <= expected_max

    expected_str = f"[{expected_min:.8f}, {expected_max:.8f}] {unit}"
    status = "PASS" if passed else "FAIL - OUT OF RANGE"
    color = "\033[92m" if passed else "\033[91m\033[1m"
    reset = "\033[0m"
    print(
        f"  {name:25} | Actual: {actual_str:18} | "
        f"Expected: {expected_str:30} | {color}[{status}]{reset}"
    )
    return passed


def check_thermistor(name, therm_out, expected_min, expected_max):
    unit = "C"
    if therm_out is None:
        actual_str = "None"
        passed = False
    else:
        val = therm_out.temperature_c
        actual_str = f"{val:.2f} {unit}"
        passed = expected_min <= val <= expected_max

    expected_str = f"[{expected_min:.2f}, {expected_max:.2f}] {unit}"
    status = "PASS" if passed else "FAIL - OUT OF RANGE"
    color = "\033[92m" if passed else "\033[91m\033[1m"
    reset = "\033[0m"
    print(
        f"  {name:25} | Actual: {actual_str:18} | "
        f"Expected: {expected_str:30} | {color}[{status}]{reset}"
    )
    return passed


def check_condition(name, actual, expected, passed):
    """Print one boolean production-test result and return it."""

    status = "PASS" if passed else "FAIL"
    color = "\033[92m" if passed else "\033[91m\033[1m"
    reset = "\033[0m"
    print(
        f"  {name:25} | Actual: {actual:18} | "
        f"Expected: {expected:30} | {color}[{status}]{reset}"
    )
    return passed


def _mean_voltage(pdro, readout, input_channel, signal_path, samples, sleep_fn):
    """Select a relay path once, let it settle, and average ADC conversions."""

    pdro.set_signal_paths(readout, signal_path)
    sleep_fn(pdro.RELAY_SETTLE_S)
    readings = [
        pdro.read_voltage(readout, input_channel, select_signal_path=False)
        for _ in range(samples)
    ]
    if any(value is None for value in readings):
        return None
    return statistics.fmean(readings)


def _check_low_gain_response(
    readout,
    dark_v,
    response_v,
    *,
    min_drop_v,
    rail_margin_v,
    response_label,
):
    """Check one low-gain response against the drop and ADC-rail limits."""

    print(f"\n--- {readout_name(readout)} ---")
    if dark_v is None or response_v is None:
        response_ok = check_condition(
            response_label,
            "Read failed",
            f"> {min_drop_v:.3f} V drop",
            False,
        )
        rail_ok = check_condition(
            "Low-gain not saturated", "Read failed", "Inside ADC rails", False
        )
        return response_ok and rail_ok

    drop_v = dark_v - response_v
    response_ok = check_condition(
        response_label,
        f"drop {drop_v:.6f} V",
        f"> {min_drop_v:.3f} V",
        drop_v > min_drop_v + VOLTAGE_COMPARISON_EPSILON_V,
    )
    lower_limit = ADC_MIN_V + rail_margin_v
    upper_limit = ADC_MAX_V - rail_margin_v
    rail_ok = check_condition(
        "Low-gain not saturated",
        f"{response_v:.6f} V",
        f"{lower_limit:.3f} < V < {upper_limit:.3f}",
        lower_limit < response_v < upper_limit,
    )
    print(f"    Baseline={dark_v:.6f} V, minimum observed={response_v:.6f} V")
    return response_ok and rail_ok


def _monitor_manual_illumination(
    pdro,
    *,
    duration_s,
    interval_s,
    sleep_fn,
    monotonic_fn,
):
    """Display both low-gain outputs and retain each minimum over the window."""

    observed: dict[Readout, list[float]] = {
        readout: [] for readout in pdro.readouts
    }
    print(
        f"\n  Shine a light into the sphere now. Monitoring both outputs for "
        f"{duration_s:.1f} seconds..."
    )
    deadline = monotonic_fn() + duration_s
    while monotonic_fn() < deadline:
        current: dict[Readout, float | None] = {}
        for readout in pdro.readouts:
            value = pdro.read_voltage(
                readout, Input.TIA_LOW_GAIN, select_signal_path=False
            )
            current[readout] = value
            if value is not None:
                observed[readout].append(value)

        remaining_s = max(0.0, deadline - monotonic_fn())
        values = "  ".join(
            f"{READOUT_NAMES[readout]}="
            + (
                f"{current[readout]:.6f} V"
                if current[readout] is not None
                else "FAILED"
            )
            for readout in pdro.readouts
        )
        print(f"  {remaining_s:4.1f} s remaining | {values}", end="\r", flush=True)
        sleep_fn(min(interval_s, remaining_s))
    print()
    return {
        readout: min(values) if values else None
        for readout, values in observed.items()
    }


def run_prelaunch_sequence(
    pdro,
    led=None,
    *,
    bias_v=DEFAULT_BIAS_V,
    led_code=DEFAULT_LED_CODE,
    samples=DEFAULT_SAMPLES,
    min_bias_drop_v=DEFAULT_MIN_BIAS_DROP_V,
    min_led_drop_v=DEFAULT_MIN_LED_DROP_V,
    adc_rail_margin_v=DEFAULT_ADC_RAIL_MARGIN_V,
    settle_s=DEFAULT_SETTLE_S,
    manual_illumination=False,
    manual_duration_s=DEFAULT_MANUAL_ILLUMINATION_S,
    monitor_interval_s=DEFAULT_MONITOR_INTERVAL_S,
    sleep_fn=time.sleep,
    monotonic_fn=time.monotonic,
):
    """Exercise the complete flight photodiode/LED signal chain.

    Returns ``True`` only when every check on every open readout passes.  The
    caller must open both readouts for the launch test.  Cleanup is performed
    here as well as by the drivers' context managers so an interrupted or
    failed check does not leave optical power or photodiode bias applied.
    """

    if set(pdro.readouts) != set(Readout):
        raise ValueError("the prelaunch sequence requires both PDRO readouts")
    if samples < 1:
        raise ValueError("samples must be at least 1")
    if not 0 < bias_v <= pdro.MAX_BIAS_V:
        raise ValueError(
            f"bias_v must be greater than 0 and at most {pdro.MAX_BIAS_V}"
        )
    if not manual_illumination and led is None:
        raise ValueError(
            "an LED source is required unless manual illumination is selected"
        )
    if not manual_illumination and not 0 < led_code <= MAX_SAFE_CODE:
        raise ValueError(f"led_code must be between 1 and {MAX_SAFE_CODE}")
    if min_bias_drop_v < 0 or min_led_drop_v < 0:
        raise ValueError("minimum voltage drops cannot be negative")
    if settle_s < 0:
        raise ValueError("settle_s cannot be negative")
    if manual_duration_s <= 0 or monitor_interval_s <= 0:
        raise ValueError("manual illumination duration and interval must be positive")
    if not 0 <= adc_rail_margin_v < (ADC_MAX_V - ADC_MIN_V) / 2:
        raise ValueError("adc_rail_margin_v is outside the ADC input range")

    passed = True
    try:
        if led is not None:
            led.all_off()
        for readout in pdro.readouts:
            pdro.set_bias_voltage(readout, 0.0)
            pdro.set_signal_paths(readout, SignalPath.NONE)

        print("\n" + "=" * 105)
        print(" " * 34 + "PRELAUNCH PHOTODIODE / LED TEST")
        print("=" * 105)

        print("\n1. ADC configuration readback and temperature ranges")
        for readout in pdro.readouts:
            print(f"\n--- {readout_name(readout)} ---")
            expected_config = pdro.configure_input(
                readout, Input.VGND, DataRate.SPS_1000
            )
            read_config = pdro.read_adc_config(readout)
            config_ok = expected_config is not None and read_config == expected_config
            passed &= check_condition(
                "Config Read/Write",
                "Match" if config_ok else "Mismatch",
                "Match",
                config_ok,
            )
            passed &= check_thermistor(
                "Board Thermistor",
                pdro.read_board_thermistor(readout),
                20.0,
                40.0,
            )
            passed &= check_thermistor(
                "Photodiode Thermistor",
                pdro.read_photodiode_thermistor(readout),
                20.0,
                40.0,
            )

        print(f"\n2. High-gain dark-current response at {bias_v:.3f} V bias")
        for readout in pdro.readouts:
            actual_bias = pdro.set_bias_voltage(readout, bias_v)
            print(f"  {readout_name(readout)} bias set to {actual_bias:.6f} V")
        sleep_fn(settle_s)

        for readout in pdro.readouts:
            print(f"\n--- {readout_name(readout)} ---")
            vgnd_v = _mean_voltage(
                pdro,
                readout,
                Input.VGND,
                SignalPath.NONE,
                samples,
                sleep_fn,
            )
            tia_v = _mean_voltage(
                pdro,
                readout,
                Input.TIA,
                SignalPath.TIA,
                samples,
                sleep_fn,
            )
            if vgnd_v is None or tia_v is None:
                passed &= check_condition(
                    "Biased high-gain TIA",
                    "Read failed",
                    f"> {min_bias_drop_v:.3f} V drop",
                    False,
                )
                continue
            drop_v = vgnd_v - tia_v
            passed &= check_condition(
                "Biased high-gain TIA",
                f"drop {drop_v:.6f} V",
                f"> {min_bias_drop_v:.3f} V below VGND",
                drop_v > min_bias_drop_v + VOLTAGE_COMPARISON_EPSILON_V,
            )
            print(f"    VGND={vgnd_v:.6f} V, biased TIA={tia_v:.6f} V")

        if manual_illumination:
            print("\n3. Low-gain manual illumination response")
        else:
            print(
                f"\n3. Low-gain integrating-sphere LED response (DAC code {led_code})"
            )
            led.all_off()
        sleep_fn(settle_s)
        dark: dict[Readout, float | None] = {}
        for readout in pdro.readouts:
            dark[readout] = _mean_voltage(
                pdro,
                readout,
                Input.TIA_LOW_GAIN,
                SignalPath.TIA_LOW_GAIN,
                samples,
                sleep_fn,
            )

        if manual_illumination:
            response = _monitor_manual_illumination(
                pdro,
                duration_s=manual_duration_s,
                interval_s=monitor_interval_s,
                sleep_fn=sleep_fn,
                monotonic_fn=monotonic_fn,
            )
            response_label = "Manual light response"
        else:
            led.set_code(led_code)
            sleep_fn(settle_s)
            response = {
                readout: _mean_voltage(
                    pdro,
                    readout,
                    Input.TIA_LOW_GAIN,
                    SignalPath.TIA_LOW_GAIN,
                    samples,
                    sleep_fn,
                )
                for readout in pdro.readouts
            }
            response_label = "LED response"

        for readout in pdro.readouts:
            passed &= _check_low_gain_response(
                readout,
                dark[readout],
                response[readout],
                min_drop_v=min_led_drop_v,
                rail_margin_v=adc_rail_margin_v,
                response_label=response_label,
            )
    finally:
        if led is not None:
            try:
                led.all_off()
            except Exception as exc:
                print(
                    f"  [FAIL] Could not turn sphere LEDs off: {exc}",
                    file=sys.stderr,
                )
                passed = False
        for readout in pdro.readouts:
            try:
                pdro.set_bias_voltage(readout, 0.0)
                pdro.set_signal_paths(readout, SignalPath.NONE)
            except Exception as exc:
                print(
                    f"  [FAIL] Could not make {readout_name(readout)} safe: {exc}",
                    file=sys.stderr,
                )
                passed = False

    print("\n" + "=" * 105)
    print("PRELAUNCH RESULT: " + ("PASS" if passed else "FAIL"))
    safe_state = "Both photodiode biases set to 0 V; all PDRO relays open."
    if led is not None:
        safe_state = "Sphere LEDs off; " + safe_state.lower()
    print(safe_state)
    print("=" * 105 + "\n")
    return passed


def run_all_checks(pdro: UVICPDRO):
    print("\n" + "=" * 105)
    print(" " * 42 + "RUNNING BATCH CHECKS")
    print("=" * 105)
    for readout in pdro.readouts:
        print(f"\n--- {readout_name(readout)} ---")
        print("-" * 105)

        expected_config = pdro.configure_input(
            readout, Input.VGND, DataRate.SPS_1000
        )
        read_config = pdro.read_adc_config(readout)
        passed = read_config == expected_config
        status = "PASS" if passed else "FAIL - MISMATCH"
        color = "\033[92m" if passed else "\033[91m\033[1m"
        reset = "\033[0m"
        actual_str = "Match" if passed else "Mismatch"
        print(
            f"  {'Config Read/Write':25} | Actual: {actual_str:18} | "
            f"Expected: {'Match':30} | {color}[{status}]{reset}"
        )

        check_range(
            "VGND Voltage",
            pdro.read_voltage(readout, Input.VGND, select_signal_path=False),
            4.85,
            4.95,
        )
        check_range(
            "TIA Voltage",
            pdro.read_voltage(readout, Input.TIA, select_signal_path=False),
            4.85,
            4.95,
        )
        check_range(
            "IVC Level Shifter",
            pdro.read_voltage(readout, Input.IVC, select_signal_path=False),
            0.0,
            0.1,
        )
        check_range(
            "ACF Level Shifter",
            pdro.read_voltage(readout, Input.ACF, select_signal_path=False),
            0.0,
            0.4,
        )

        therm_out: ThermistorReading | None = pdro.read_board_thermistor(readout)
        check_thermistor("Board Thermistor", therm_out, 20.0, 40.0)
        therm_out = pdro.read_photodiode_thermistor(readout)
        check_thermistor("Photodiode Thermistor", therm_out, 20.0, 40.0)

    print("\n" + "=" * 105)
    print(" " * 44 + "CHECKS COMPLETE")
    print("=" * 105 + "\n")


def select_readout(pdro: UVICPDRO) -> Readout | None:
    if len(pdro.readouts) == 1:
        return pdro.readouts[0]

    print("\nSelect readout:")
    for index, readout in enumerate(pdro.readouts, start=1):
        print(f"{index}: {readout_name(readout)}")
    try:
        index = int(input("Enter choice: ").strip()) - 1
        if not 0 <= index < len(pdro.readouts):
            raise ValueError
        return pdro.readouts[index]
    except ValueError:
        print("Invalid readout choice.")
        return None


def stream_input(pdro: UVICPDRO):
    channels = (
        ("VGND", Input.VGND),
        ("TIA", Input.TIA),
        ("TIA_LOW_GAIN", Input.TIA_LOW_GAIN),
        ("IVC", Input.IVC),
        ("ACF", Input.ACF),
        ("BOARD_TMP", "board_temperature"),
        ("PD_TMP", "photodiode_temperature"),
    )
    print("\nSelect input to stream:")
    for index, (name, _) in enumerate(channels, start=1):
        print(f"{index}: {name}")
    try:
        index = int(input("Enter choice: ").strip()) - 1
        if not 0 <= index < len(channels):
            raise ValueError
    except ValueError:
        print("Invalid choice.")
        return

    readout = select_readout(pdro)
    if readout is None:
        return
    channel_name, channel = channels[index]
    print(
        f"\nStreaming {channel_name} on {readout_name(readout)}. "
        "Press Ctrl+C to stop."
    )
    try:
        while True:
            if channel == "board_temperature":
                reading = pdro.read_board_thermistor(readout)
                value = None if reading is None else reading.temperature_c
                print(
                    f"{channel_name}: {value:.2f} C"
                    if value is not None
                    else f"{channel_name}: Read failed"
                )
            elif channel == "photodiode_temperature":
                reading = pdro.read_photodiode_thermistor(readout)
                value = None if reading is None else reading.temperature_c
                print(
                    f"{channel_name}: {value:.2f} C"
                    if value is not None
                    else f"{channel_name}: Read failed"
                )
            else:
                value = pdro.read_voltage(readout, channel)
                print(
                    f"{channel_name}: {value:.8f} V"
                    if value is not None
                    else f"{channel_name}: Read failed"
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped streaming.")


def manually_actuate_relays(pdro: UVICPDRO):
    print("\nSelect relays to actuate (numbers separated by spaces, or 0 for all off):")
    print("1: ACF")
    print("2: IVC")
    print("3: TIA")
    print("4: TIA_LOW_GAIN")
    print("0: All off")
    relay_map = {
        "1": SignalPath.ACF,
        "2": SignalPath.IVC,
        "3": SignalPath.TIA,
        "4": SignalPath.TIA_LOW_GAIN,
    }
    value = input("Enter choices: ").strip()
    if value == "0":
        paths = SignalPath.NONE
    else:
        paths = SignalPath.NONE
        for choice in value.split():
            if choice not in relay_map:
                print(f"Invalid option: {choice}")
                return
            paths |= relay_map[choice]

    for readout in pdro.readouts:
        print(f"Setting relays for {readout_name(readout)} (bitmask {paths.value})")
        pdro.set_signal_paths(readout, paths)


def set_bias_voltage(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    try:
        volts = float(input("Enter voltage to set (0 to 5.1): ").strip())
        actual_v = pdro.set_bias_voltage(readout, volts)
        print(f"Set {readout_name(readout)} bias to {actual_v:.8f} V")
    except ValueError as exc:
        print(f"Invalid voltage: {exc}")


def integrator_read(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    print("\nSelect integrator:")
    print("1. IVC")
    print("2. ACF")
    choice = input("Enter choice: ").strip()
    if choice == "1":
        channel = Input.IVC
    elif choice == "2":
        channel = Input.ACF
    else:
        print("Invalid integrator choice.")
        return

    try:
        time_us = float(input("Enter integration time in microseconds: ").strip())
        reading = pdro.integrate_and_read(readout, channel, time_us)
    except ValueError as exc:
        print(f"Invalid integration time: {exc}")
        return

    print(
        f"Integration duration: {reading.actual_time_us:.2f} us "
        f"(requested {reading.requested_time_us:.2f} us)"
    )
    if reading.voltage_v is None:
        print("Read failed")
    else:
        print(f"Read voltage: {reading.voltage_v:.8f} V")


def read_adc_register(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    try:
        address = int(input("Enter register address (hex or int): ").strip(), 0)
        if not 0 <= address <= 0x1F:
            raise ValueError("address must be between 0 and 0x1F")
    except ValueError as exc:
        print(f"Invalid register address: {exc}")
        return
    value = pdro.read_adc_register(readout, address)
    if value is None:
        print("Failed to read register.")
    else:
        print(f"Register 0x{address:02X} value: 0x{value:02X} ({value})")


def write_adc_register(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    try:
        address = int(input("Enter register address (hex or int): ").strip(), 0)
        value = int(input("Enter value to write (hex or int): ").strip(), 0)
        if not 0 <= address <= 0x1F:
            raise ValueError("address must be between 0 and 0x1F")
        if not 0 <= value <= 0xFF:
            raise ValueError("value must be between 0 and 0xFF")
    except ValueError as exc:
        print(f"Invalid register write: {exc}")
        return
    if pdro.write_adc_register(readout, address, value):
        print(f"Successfully wrote 0x{value:02X} to register 0x{address:02X}.")
    else:
        print("Failed to write register.")


def reset_adc(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    if pdro.reset_adc(readout):
        print(f"Successfully reset {readout_name(readout)} ADC.")
    else:
        print(f"Failed to reset {readout_name(readout)} ADC.")


def read_all_adc_registers(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    print(f"\n--- ADC registers for {readout_name(readout)} ---")
    for address in range(0x12):
        value = pdro.read_adc_register(readout, address)
        if value is None:
            print(f"Reg 0x{address:02X}: Failed")
        else:
            print(f"Reg 0x{address:02X}: 0x{value:02X} (0b{value:08b})")


def interactive_menu(pdro: UVICPDRO):
    actions = {
        "1": run_all_checks,
        "2": manually_actuate_relays,
        "3": stream_input,
        "4": set_bias_voltage,
        "5": integrator_read,
        "6": read_adc_register,
        "7": write_adc_register,
        "8": reset_adc,
        "9": read_all_adc_registers,
    }
    while True:
        print("\n--- UVIC PDRO Interactive Test Menu ---")
        print("1. Run all checks")
        print("2. Manually actuate relays")
        print("3. Stream data from an input")
        print("4. Set bias voltage")
        print("5. Integrator read command")
        print("6. Read ADC register")
        print("7. Write ADC register")
        print("8. Send ADC reset")
        print("9. Read all ADC registers")
        print("q. Quit")
        choice = input("Enter choice: ").strip().lower()
        if choice == "q":
            break
        action = actions.get(choice)
        if action is None:
            print("Unknown choice. Please select a valid option.")
        else:
            action(pdro)


def main() -> int:
    parser = argparse.ArgumentParser(description="UVIC PDRO board test script")
    parser.add_argument(
        "-b",
        "--board",
        type=int,
        choices=[1, 2, 3],
        default=3,
        help="Select readout: 1 for Sergeant, 2 for Soldier, 3 for both (default)",
    )
    parser.add_argument(
        "--prelaunch",
        action="store_true",
        help="Run the automated photodiode/sphere-LED launch test and exit",
    )
    parser.add_argument(
        "--manual-illumination",
        action="store_true",
        help="During --prelaunch, monitor low-gain outputs for 10 seconds while "
        "the user shines a light into the sphere instead of driving its LEDs",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=DEFAULT_SAMPLES,
        help="ADC samples averaged per prelaunch measurement",
    )
    parser.add_argument(
        "--bias-voltage",
        type=float,
        default=DEFAULT_BIAS_V,
        help="Photodiode bias used by --prelaunch",
    )
    parser.add_argument(
        "--led-code",
        type=int,
        default=DEFAULT_LED_CODE,
        help=f"Sphere LED DAC code used by --prelaunch (1-{MAX_SAFE_CODE})",
    )
    parser.add_argument(
        "--min-bias-drop",
        type=float,
        default=DEFAULT_MIN_BIAS_DROP_V,
        help="Minimum high-gain drop below VGND, volts",
    )
    parser.add_argument(
        "--min-led-drop",
        type=float,
        default=DEFAULT_MIN_LED_DROP_V,
        help="Minimum LED-on low-gain drop, volts",
    )
    parser.add_argument(
        "--rail-margin",
        type=float,
        default=DEFAULT_ADC_RAIL_MARGIN_V,
        help="Distance from either ADC rail required to count as unsaturated, volts",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=DEFAULT_SETTLE_S,
        help="Bias and LED settling time, seconds",
    )
    parser.add_argument(
        "--bus", type=int, default=1, help="I2C bus for the sphere LED board"
    )
    parser.add_argument(
        "--i2c-dev", default="/dev/i2c-1", help="I2C device for the sphere LED DAC"
    )
    parser.add_argument(
        "--no-ldac",
        action="store_true",
        help="Do not drive LDAC (only if it is hardwired low)",
    )
    args = parser.parse_args()
    selected = {
        1: (Readout.SERGEANT,),
        2: (Readout.SOLDIER,),
        3: tuple(Readout),
    }[args.board]

    if args.prelaunch and args.board != 3:
        print("Prelaunch testing requires --board 3 (both readouts).", file=sys.stderr)
        return 2
    if args.manual_illumination and not args.prelaunch:
        print("--manual-illumination requires --prelaunch.", file=sys.stderr)
        return 2

    try:
        with UVICPDRO(readouts=selected) as pdro:
            if not args.prelaunch:
                interactive_menu(pdro)
                return 0

            sequence_args = {
                "bias_v": args.bias_voltage,
                "led_code": args.led_code,
                "samples": args.samples,
                "min_bias_drop_v": args.min_bias_drop,
                "min_led_drop_v": args.min_led_drop,
                "adc_rail_margin_v": args.rail_margin,
                "settle_s": args.settle,
            }
            if args.manual_illumination:
                passed = run_prelaunch_sequence(
                    pdro,
                    manual_illumination=True,
                    **sequence_args,
                )
                return 0 if passed else 1

            try:
                import smbus2
            except ImportError as exc:
                raise RuntimeError("smbus2 is required for the sphere LED test") from exc

            bus = smbus2.SMBus(args.bus)
            try:
                with SphereLedSource(
                    bus=bus,
                    i2c_dev=args.i2c_dev,
                    use_ldac=not args.no_ldac,
                ) as led:
                    passed = run_prelaunch_sequence(
                        pdro,
                        led,
                        **sequence_args,
                    )
            finally:
                bus.close()
            return 0 if passed else 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Failed to initialize or operate UVIC PDRO: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
