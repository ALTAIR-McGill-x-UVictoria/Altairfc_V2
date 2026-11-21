from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.ads1115 import SENSE_RESISTOR_OHM
from drivers.beacon_led import BEACON_MAX_SAFE_CODE, BEACON_SENSE_RESISTOR_OHM, BeaconChannel
from drivers.led_board import LedBoard
from drivers.sphere_led import LedState, SphereLedSource

logger = logging.getLogger(__name__)

_SECONDS_PER_DAY = 86400.0
_PULSED_TELEMETRY_PERIOD_S = 0.25


@dataclass
class ImagingWindow:
    start_s: float   # seconds since UTC midnight (or since the start of `period_s`)
    duration_s: float
    label: str = ""
    period_s: float = _SECONDS_PER_DAY   # recurrence period; e.g. 60.0 for "every minute"


def parse_windows(raw: list[dict[str, Any]]) -> list[ImagingWindow]:
    """Parse [[tasks.lighting.beacon_windows]] entries: {start_utc: "HH:MM:SS", duration_s: float, period_s?: float}."""
    windows: list[ImagingWindow] = []
    for i, w in enumerate(raw):
        h, m, s = str(w["start_utc"]).split(":")
        start_s = int(h) * 3600 + int(m) * 60 + float(s)
        windows.append(ImagingWindow(
            start_s=start_s,
            duration_s=float(w["duration_s"]),
            label=str(w.get("label", f"window_{i}")),
            period_s=float(w.get("period_s", _SECONDS_PER_DAY)),
        ))
    return windows


def seconds_of_day_utc(dt: datetime) -> float:
    return dt.hour * 3600 + dt.minute * 60 + dt.second + dt.microsecond / 1e6


def window_active(now_s: float, w: ImagingWindow) -> bool:
    """True if now_s falls in [w.start_s, w.start_s + w.duration_s) modulo w.period_s.

    The modulo handles both UTC-midnight wraparound (period_s == one day, a
    window starting before 00:00:00 and ending after) and sub-day recurrence
    (e.g. period_s == 60.0 for a window that repeats every minute) with no
    special-casing needed.
    """
    delta = (now_s - w.start_s) % w.period_s
    return delta < w.duration_s


def seconds_to_window(now_s: float, w: ImagingWindow) -> float:
    """Seconds until w next starts; 0.0 if it's already active."""
    if window_active(now_s, w):
        return 0.0
    return (w.start_s - now_s) % w.period_s


def flash_state(now_s: float, flash_hz: float) -> bool:
    """True during the "on" half of a 50% duty-cycle square wave at flash_hz.

    Phase-locked to now_s (seconds since UTC midnight), not wall-clock-since-task-start,
    so the on/off phase doesn't depend on exactly when the task happened to start.
    """
    period_s = 1.0 / flash_hz
    return (now_s % period_s) < (period_s / 2.0)


@dataclass(frozen=True)
class PulseSchedule:
    """Open-loop pulsed sphere: pulse k of every UTC minute starts at first_pulse_s + k * period_s and lasts on_s;
    nothing is lit from window_end_s (the start of the naked-eye beacon window) to the end of the minute."""

    on_s: float
    period_s: float
    first_pulse_s: float
    dac_code: int
    window_end_s: float
    minute_period_s: float = 60.0


def sphere_pulse_on(now_s: float, p: PulseSchedule) -> bool:
    """True while the pulsed sphere should be lit. now_s is seconds since UTC midnight; the phase is locked to the UTC minute."""
    t = now_s % p.minute_period_s
    if t >= p.window_end_s:
        return False
    return ((t - p.first_pulse_s) % p.period_s) < p.on_s


def validate_schedule(schedule: dict[str, Any], pulse: dict[str, Any], beacon_windows: list[ImagingWindow]) -> list[str]:
    """Problems with the shared UTC schedule ([tasks.lighting.schedule] + [tasks.lighting.sphere_pulse]); empty when it is consistent.

    LightingTask raises on a non-empty list at construction, so a bad settings.toml fails on the bench, not in flight. The ground station
    reads the same tables and applies the same rules before it commands an exposure."""
    minute = float(schedule.get("minute_period_s", 60.0))
    beacon_start = float(schedule["beacon_start_s"])
    beacon_dur = float(schedule["beacon_duration_s"])
    exp_start = float(schedule["exposure_start_s"])
    exp_s = float(schedule["exposure_s"])
    delay_max = float(schedule["exposure_delay_max_s"])
    guard = float(schedule["guard_s"])
    on_s = float(pulse["on_s"])
    period = float(pulse["period_s"])
    first = float(pulse.get("first_pulse_s", 0.0))
    problems: list[str] = []
    if beacon_start + beacon_dur > minute + 1e-9:
        problems.append("beacon window runs past the end of the minute")
    if exp_start < guard:
        problems.append("exposure may start inside the guard after the previous minute's beacon window")
    if exp_start + exp_s + delay_max + guard > beacon_start + 1e-9:
        problems.append("an exposure can still be open when the beacon flash starts")
    if on_s <= 0 or period <= 0 or on_s >= period:
        problems.append("pulse on_s must be positive and shorter than period_s")
    else:
        if abs(minute / period - round(minute / period)) > 1e-9:
            problems.append("pulse period does not divide the minute: the pulse phase would drift between minutes")
        if abs(exp_s / period - round(exp_s / period)) > 1e-9:
            problems.append("exposure is not a whole number of pulse periods: ON time per frame would depend on the phase")
        k_last = int((beacon_start - first - on_s) // period)
        if k_last < 0 or first + k_last * period + on_s > beacon_start + 1e-9:
            problems.append("no pulse fits before the beacon window")
    if not any(abs((w.start_s % minute) - beacon_start) < 1e-6 and abs(w.period_s - minute) < 1e-6 and w.duration_s >= beacon_dur - 1e-9 for w in beacon_windows):
        problems.append("beacon_windows has no recurring window matching schedule.beacon_start_s / beacon_duration_s")
    return problems


class LightingTask(BaseTask):
    """
    Two independent behaviors sharing one LED board, run fully independently
    of each other:

      * Sphere source: engages SphereLedSource's current-hold PI loop as soon
        as this task starts (first execute() after a successful setup()) —
        NOT gated on event.ascent_active or any other flight-stage event —
        and never turns it off internally — "do not turn off once started"
        is implemented by NOT having an off-path in execute() at all. The
        sphere (and the beacon) are only ever turned off by this task being
        stopped: FlightStageTask calls scheduler.get_task("lighting").stop()
        at apogee (burst/termination detected, mirroring how it already
        stops "pointing"), which ends this task's run loop and invokes
        teardown(), which zeroes both channels.

      * Spotter beacon: enabled on a fixed GPS-UTC schedule (beacon_windows,
        e.g. active for 5s at :25 and :55 of every minute), regardless of
        whether the sphere is on. Within an active window it strobes on/off
        as a 50% duty-cycle square wave at beacon_flash_hz (see flash_state())
        rather than staying continuously lit. Open-loop: brightness (MCP4728
        channel 1) is set once to a fixed DAC code (beacon_dac_code); each
        on/off toggle drives the relay (channel 3) — see drivers/beacon_led.py.
        beacon_windows are defined as the times ground-based imaging is NOT
        happening — i.e. the beacon flashing during those windows is exactly
        why they must stay clear of actual calibration exposures. Outside
        beacon_windows the beacon is off, so it never contaminates an
        exposure.

    The sphere and the beacon's relay CAN be energized together — there's no
    electrical constraint against it (drivers.led_board.LedBoard does not
    interlock them); keeping them apart is purely about not flashing the
    beacon during a real imaging exposure, which is what beacon_windows
    already guarantees by construction.

    Imaging is done by ground-based cameras/telescopes, not an onboard camera:
    since the downlink is unidirectional, ground observers independently know
    the same UTC window schedule (beacon_windows) and are expected to image
    outside of it, so no uplink command is needed to coordinate exposure
    timing with the beacon.

    UTC source: the Pi's system clock (datetime.now(timezone.utc)), which is
    chrony/GPS-PPS-disciplined pre-launch. Per project decision this is used
    unconditionally, whether or not chrony is currently PPS-disciplined in
    flight (see system.pps_synced for that diagnostic) — there is no separate
    fallback branch.

    PULSED SPHERE (branch pulsed-sphere). When [tasks.lighting.sphere_pulse] enabled = true the sphere is NOT held by the current loop: it is
    driven open loop between DAC code 0 and sphere_pulse.dac_code on a UTC-minute-locked schedule (sphere_pulse_on), dark during the
    naked-eye beacon window. The PI loop, the target current and the ground-commanded target-current update are not used in this mode. The
    schedule (shared with the ground station, which commands the exposures) is validated at construction (validate_schedule).
    Each edge is logged (INFO, Pi UTC) and written to lighting.pulse_edge_utc / pulse_edge_on / pulse_count.

    DataStore keys read:
        event.ascent_active          — telemetry only (mirrored to lighting.observation_active
                                        below); no longer gates the sphere, which latches on at
                                        task start regardless of flight stage

    DataStore keys written:
        lighting.sphere_on              (int 0/1)
        lighting.beacon_on              (int 0/1)
        lighting.observation_active     (int 0/1, mirrors event.ascent_active)
        lighting.next_beacon_flash_in_s (float, seconds until next beacon window; 0 if active)
        lighting.sphere_dac_code        (int, live MCP4728 code, SphereLedSource.code)
        lighting.sphere_current_a       (float, ADS1115 current-sense reading)
        lighting.sphere_temperature_c   (float, thermistor bridge reading)
        lighting.beacon_dac_code        (int, live MCP4728 code, BeaconChannel.code)
        lighting.beacon_current_a       (float, ADS1115 current-sense reading)
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        beacon_windows: list[dict[str, Any]],
        i2c_dev: str = "/dev/i2c-1",
        sphere_target_current_a: float | None = None,
        sphere_kp: float | None = None,
        sphere_ki: float | None = None,
        beacon_dac_code: int = 1500,
        beacon_flash_hz: float = 2.0,
        sphere_sense_resistor_ohm: float = SENSE_RESISTOR_OHM,
        beacon_sense_resistor_ohm: float = BEACON_SENSE_RESISTOR_OHM,
        schedule: dict[str, Any] | None = None,
        sphere_pulse: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        if beacon_flash_hz <= 0:
            raise ValueError("beacon_flash_hz must be greater than zero")
        self._i2c_dev = i2c_dev
        self._beacon_windows = parse_windows(beacon_windows)
        self._sphere_target_current_a = sphere_target_current_a
        self._sphere_kp = sphere_kp
        self._sphere_ki = sphere_ki
        self._beacon_dac_code = max(0, min(BEACON_MAX_SAFE_CODE, int(beacon_dac_code)))
        self._beacon_flash_hz = beacon_flash_hz
        self._sphere_sense_resistor_ohm = sphere_sense_resistor_ohm
        self._beacon_sense_resistor_ohm = beacon_sense_resistor_ohm

        self._pulse: PulseSchedule | None = None
        if sphere_pulse and sphere_pulse.get("enabled", False):
            if not schedule:
                raise ValueError("sphere_pulse is enabled but [tasks.lighting.schedule] is missing")
            problems = validate_schedule(schedule, sphere_pulse, self._beacon_windows)
            if problems:
                raise ValueError("invalid pulsed-sphere schedule: " + "; ".join(problems))
            self._pulse = PulseSchedule(
                on_s=float(sphere_pulse["on_s"]),
                period_s=float(sphere_pulse["period_s"]),
                first_pulse_s=float(sphere_pulse.get("first_pulse_s", 0.0)),
                dac_code=max(0, min(4095, int(sphere_pulse["dac_code"]))),
                window_end_s=float(schedule["beacon_start_s"]),
                minute_period_s=float(schedule.get("minute_period_s", 60.0)),
            )
            logger.info(
                "LightingTask: pulsed sphere, %.2f s ON every %.2f s at DAC code %d, dark from :%02d of each minute",
                self._pulse.on_s, self._pulse.period_s, self._pulse.dac_code, int(self._pulse.window_end_s),
            )

        if not self._beacon_windows:
            logger.warning("LightingTask: no beacon windows configured — beacon will never flash")
        if self._sphere_target_current_a is None and self._pulse is None:
            logger.warning("LightingTask: sphere_target_current_a not configured — sphere source will never fire")

        self._bus = None
        self._board: LedBoard | None = None
        self._sphere: SphereLedSource | None = None
        self._beacon: BeaconChannel | None = None

        self._sphere_on = False
        self._beacon_on = False
        self._pulse_count = 0
        self._telemetry_t = 0.0

        # Sphere current-loop instrumentation: logs the achieved update() rate plus the
        # current spread seen each second, so both "is the loop running fast enough" and
        # "is it actually settling" are visible in flight.log rather than assumed. The
        # beacon is open-loop (no update() to instrument) -- see drivers/beacon_led.py.
        self._loop_stats = {"sphere": self._new_loop_stats()}

    @staticmethod
    def _new_loop_stats() -> dict[str, float]:
        return {
            "iters": 0,
            "rate_log_t": 0.0,
            "current_min": float("inf"),
            "current_max": float("-inf"),
            "current_sum": 0.0,
            "settled_count": 0,
        }

    def setup(self) -> None:
        # _sphere_on/_beacon_on must be reset here, not just in __init__: if execute() ever
        # raises (e.g. a transient I2C fault), core/task_base.py restarts this task, which
        # calls setup() again and builds a brand-new SphereLedSource/BeaconChannel — but
        # without this reset, the one-shot latch in _apply() would see _sphere_on already
        # True from before the restart and never call hold_current() on the new object,
        # leaving it silently spinning at code 0 with no target forever (looked, in the log,
        # like a healthy loop running at speed with target=nan and code=0).
        self._sphere_on = False
        self._beacon_on = False
        self._pulse_count = 0
        self._telemetry_t = 0.0
        self._loop_stats = {"sphere": self._new_loop_stats()}
        try:
            import smbus2
            self._bus = smbus2.SMBus(int(self._i2c_dev.replace("/dev/i2c-", "")))
            self._board = LedBoard(bus=self._bus, i2c_dev=self._i2c_dev)
            self._sphere = SphereLedSource(
                board=self._board, sense_resistor_ohm=self._sphere_sense_resistor_ohm
            )
            self._beacon = BeaconChannel(
                board=self._board, sense_resistor_ohm=self._beacon_sense_resistor_ohm
            )
            self._beacon.set_brightness(self._beacon_dac_code)
        except Exception:
            logger.exception(
                "LightingTask: failed to open LED board on %s — lighting control disabled",
                self._i2c_dev,
            )
            self._board = None
            self._sphere = None
            self._beacon = None

    def execute(self) -> None:
        now = datetime.now(timezone.utc)
        now_s = seconds_of_day_utc(now)

        observation_active = bool(int(self.datastore.read("event.ascent_active", default=0)))

        active_beacon_window = next(
            (w for w in self._beacon_windows if window_active(now_s, w)), None
        )
        if active_beacon_window is not None:
            next_in_s = 0.0
        elif self._beacon_windows:
            next_in_s = min(seconds_to_window(now_s, w) for w in self._beacon_windows)
        else:
            next_in_s = float("inf")

        sphere_state = None
        if self._board is not None:
            sphere_state = self._apply(observation_active, active_beacon_window, now_s)

        self.datastore.write("lighting.sphere_on", int(self._sphere_on))
        self.datastore.write("lighting.beacon_on", int(self._beacon_on))
        self.datastore.write("lighting.observation_active", int(observation_active))
        self.datastore.write("lighting.next_beacon_flash_in_s", next_in_s)
        # Pulsed mode: the ADC reads behind this call are two I2C transactions, which would delay the next edge check at the task's 50 Hz rate -- 4 Hz is plenty.
        if self._pulse is None or time.monotonic() - self._telemetry_t >= _PULSED_TELEMETRY_PERIOD_S:
            self._telemetry_t = time.monotonic()
            self._write_channel_telemetry(sphere_state)

    def _write_channel_telemetry(self, sphere_state: LedState | None) -> None:
        """DAC code + measured current for both channels, plus sphere temperature.

        sphere_state, when given, is the LedState _apply() already got out of
        SphereLedSource.update() this tick -- reused here instead of taking a
        second ADC/thermistor reading. Falls back to a fresh read when the
        sphere hasn't been engaged yet (update() not called), so the fields
        are still populated at their idle values pre-launch. The beacon is
        open-loop with no per-tick update(), so it's always read fresh here.
        """
        if self._sphere is not None:
            if sphere_state is not None:
                self.datastore.write("lighting.sphere_dac_code", sphere_state.code)
                self.datastore.write("lighting.sphere_current_a", sphere_state.current_a)
                self.datastore.write("lighting.sphere_temperature_c", sphere_state.temperature_c)
            else:
                self.datastore.write("lighting.sphere_dac_code", self._sphere.code)
                self.datastore.write("lighting.sphere_current_a", self._sphere.read_current_a())
                self.datastore.write("lighting.sphere_temperature_c", self._sphere.read_bridge_temperature_c())
        if self._beacon is not None:
            self.datastore.write("lighting.beacon_dac_code", self._beacon.code)
            self.datastore.write("lighting.beacon_current_a", self._beacon.read_current_a())

    def _apply(
        self, observation_active: bool, active_beacon_window: ImagingWindow | None, now_s: float
    ) -> LedState | None:
        # One-shot latch: engages the moment this task starts (no longer
        # gated on event.ascent_active) and there is no internal path back
        # off. It only ever stops via this task being stopped (see the
        # class docstring) — teardown() zeroes it unconditionally.
        if self._pulse is not None:
            self._apply_pulse(now_s)
        elif not self._sphere_on and self._sphere_target_current_a is not None:
            self._sphere.hold_current(
                self._sphere_target_current_a, kp=self._sphere_kp, ki=self._sphere_ki
            )
            self._sphere_on = True
            logger.info(
                "LightingTask: sphere on at task start (target %.4f A) — will not turn off "
                "internally; stopped externally at apogee",
                self._sphere_target_current_a,
            )
        elif self._sphere_on:
            self._poll_target_current_update()

        sphere_state = None
        if self._sphere_on and self._pulse is None:
            sphere_state = self._sphere.update()
            self._log_loop_stats(
                "sphere", sphere_state.target_current_a, sphere_state.current_a,
                sphere_state.settled, sphere_state.code,
            )

        # Independent of the sphere: strobe on/off at beacon_flash_hz while inside a
        # beacon_windows entry (which are defined as the times ground imaging is NOT
        # happening, so the sphere being on at the same time never contaminates a real
        # exposure), off otherwise. Open-loop -- no per-tick update() to drive, just the
        # relay toggle (see drivers/beacon_led.py).
        beacon_should_flash_on = (
            active_beacon_window is not None and flash_state(now_s, self._beacon_flash_hz)
        )
        self._set_beacon(beacon_should_flash_on)

        return sphere_state

    def _apply_pulse(self, now_s: float) -> None:
        """Open-loop pulsed sphere: one DAC code step per edge, no PI loop, no ADC reads on this path."""
        want = sphere_pulse_on(now_s, self._pulse)
        if want == self._sphere_on:
            return
        code = self._pulse.dac_code if want else 0
        self._sphere.set_code(code)
        t_edge = time.time()          # Pi UTC system clock, immediately after the DAC write returned
        self._sphere_on = want
        if want:
            self._pulse_count += 1
        self.datastore.write("lighting.pulse_edge_utc", t_edge)
        self.datastore.write("lighting.pulse_edge_on", int(want))
        self.datastore.write("lighting.pulse_count", self._pulse_count)
        logger.info("LightingTask: sphere pulse %d %s at %.3f UTC (DAC code %d)", self._pulse_count, "ON" if want else "OFF", t_edge, code)

    def _poll_target_current_update(self) -> None:
        """
        Re-engage hold_current() if settings.sphere_target_current_a has been
        changed at runtime — e.g. via UpdateSettingCommandPacket field_id 18
        from the GS calibration tab (see telemetry/commands/update_setting.py
        SETTING_KEYS). CommandReceiverTask writes straight to this DataStore
        key; nothing else in the flight code path does, so a plain read here
        is enough to notice a ground-commanded brightness change without any
        extra plumbing between the two tasks.

        Reads the same settings.sphere_target_current_a key the constructor's
        sphere_target_current_a argument seeded at startup, so a GS command
        transparently overrides the settings.toml default for the rest of
        this task's run (until the next command or task restart).

        A target of exactly 0 A drives the DAC to code 0 directly
        (SphereLedSource.all_off()) instead of engaging the PI loop toward
        zero — hold_current(0.0) would otherwise spend several update()
        cycles servoing the code down from wherever it currently sits, which
        is pointless when the intent is simply "off now". all_off() also
        disengages the loop, so subsequent update() calls just report sensor
        readback without fighting to re-converge on 0.
        """
        new_target = self.datastore.read(
            "settings.sphere_target_current_a", default=self._sphere_target_current_a
        )
        if new_target is None or new_target == self._sphere_target_current_a:
            return
        self._sphere_target_current_a = float(new_target)
        if self._sphere_target_current_a == 0.0:
            self._sphere.all_off()
            logger.info("LightingTask: sphere target current set to 0 A — DAC driven to 0 directly")
        else:
            self._sphere.hold_current(
                self._sphere_target_current_a, kp=self._sphere_kp, ki=self._sphere_ki
            )
            logger.info(
                "LightingTask: sphere target current updated to %.4f A via ground command",
                self._sphere_target_current_a,
            )

    def _log_loop_stats(
        self, channel: str, target_current_a: float | None, current_a: float, settled: bool, code: int
    ) -> None:
        """Once/sec, log the sphere loop's achieved update() rate plus the current spread seen
        within that window (min/max/mean vs. target) — the rate alone (previously all this
        logged) says nothing about whether the loop is actually settling or oscillating around
        target."""
        stats = self._loop_stats[channel]
        stats["iters"] += 1
        stats["current_min"] = min(stats["current_min"], current_a)
        stats["current_max"] = max(stats["current_max"], current_a)
        stats["current_sum"] += current_a
        if settled:
            stats["settled_count"] += 1

        now = time.monotonic()
        if stats["rate_log_t"] == 0.0:
            stats["rate_log_t"] = now
            return
        elapsed = now - stats["rate_log_t"]
        if elapsed >= 1.0:
            mean_a = stats["current_sum"] / stats["iters"]
            logger.info(
                "LightingTask: %s loop %.1f Hz (%d updates/%.2fs) — target=%.4f A "
                "mean=%.4f A min=%.4f A max=%.4f A spread=%.4f A settled=%d/%d code=%d",
                channel, stats["iters"] / elapsed, stats["iters"], elapsed,
                target_current_a if target_current_a is not None else float("nan"),
                mean_a, stats["current_min"], stats["current_max"],
                stats["current_max"] - stats["current_min"],
                stats["settled_count"], stats["iters"], code,
            )
            self._loop_stats[channel] = self._new_loop_stats()
            self._loop_stats[channel]["rate_log_t"] = now

    def _set_beacon(self, on: bool) -> None:
        if on == self._beacon_on:
            return
        if on:
            self._beacon.on()
        else:
            self._beacon.off()
        self._beacon_on = on

    def teardown(self) -> None:
        if self._sphere is not None:
            self._sphere.all_off()
        if self._beacon is not None:
            self._beacon.all_off()  # zeros both brightness (ch1) and relay (ch3)
        if self._board is not None:
            self._board.close()
        if self._bus is not None:
            self._bus.close()
