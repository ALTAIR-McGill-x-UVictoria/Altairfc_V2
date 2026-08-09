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
        rather than staying continuously lit. Brightness (MCP4728 channel 1)
        is set once; each on/off toggle drives the relay (channel 3) — see
        drivers/beacon_led.py. beacon_windows are defined as the times
        ground-based imaging is NOT happening — i.e. the beacon flashing
        during those windows is exactly why they must stay clear of actual
        calibration exposures. Outside beacon_windows the beacon is off, so
        it never contaminates an exposure.

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

    DataStore keys read:
        event.ascent_active          — telemetry only (mirrored to lighting.observation_active
                                        below); no longer gates the sphere, which latches on at
                                        task start regardless of flight stage

    DataStore keys written:
        lighting.sphere_on              (int 0/1)
        lighting.beacon_on              (int 0/1)
        lighting.observation_active     (int 0/1, mirrors event.ascent_active)
        lighting.next_beacon_flash_in_s (float, seconds until next beacon window; 0 if active)
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

        if not self._beacon_windows:
            logger.warning("LightingTask: no beacon windows configured — beacon will never flash")
        if self._sphere_target_current_a is None:
            logger.warning("LightingTask: sphere_target_current_a not configured — sphere source will never fire")

        self._bus = None
        self._board: LedBoard | None = None
        self._sphere: SphereLedSource | None = None
        self._beacon: BeaconChannel | None = None

        self._sphere_on = False
        self._beacon_on = False

        # Sphere current-loop instrumentation: logs the achieved sphere.update()
        # rate plus the current spread seen each second, so both "is the loop
        # running fast enough" and "is it actually settling" are visible in
        # flight.log rather than assumed.
        self._loop_iters = 0
        self._loop_rate_log_t = 0.0
        self._loop_current_min = float("inf")
        self._loop_current_max = float("-inf")
        self._loop_current_sum = 0.0
        self._loop_settled_count = 0

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
        self._loop_iters = 0
        self._loop_rate_log_t = 0.0
        self._loop_current_min = float("inf")
        self._loop_current_max = float("-inf")
        self._loop_current_sum = 0.0
        self._loop_settled_count = 0
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

        if self._board is not None:
            self._apply(observation_active, active_beacon_window, now_s)

        self.datastore.write("lighting.sphere_on", int(self._sphere_on))
        self.datastore.write("lighting.beacon_on", int(self._beacon_on))
        self.datastore.write("lighting.observation_active", int(observation_active))
        self.datastore.write("lighting.next_beacon_flash_in_s", next_in_s)

    def _apply(
        self, observation_active: bool, active_beacon_window: ImagingWindow | None, now_s: float
    ) -> None:
        # One-shot latch: engages the moment this task starts (no longer
        # gated on event.ascent_active) and there is no internal path back
        # off. It only ever stops via this task being stopped (see the
        # class docstring) — teardown() zeroes it unconditionally.
        if not self._sphere_on and self._sphere_target_current_a is not None:
            self._sphere.hold_current(
                self._sphere_target_current_a, kp=self._sphere_kp, ki=self._sphere_ki
            )
            self._sphere_on = True
            logger.info(
                "LightingTask: sphere on at task start (target %.4f A) — will not turn off "
                "internally; stopped externally at apogee",
                self._sphere_target_current_a,
            )

        if self._sphere_on:
            state = self._sphere.update()
            self._log_loop_stats(state)

        # Independent of the sphere: strobe on/off at beacon_flash_hz while inside a
        # beacon_windows entry (which are defined as the times ground imaging is NOT
        # happening, so the sphere being on at the same time never contaminates a real
        # exposure), off otherwise.
        beacon_should_flash_on = (
            active_beacon_window is not None and flash_state(now_s, self._beacon_flash_hz)
        )
        self._set_beacon(beacon_should_flash_on)

    def _log_loop_stats(self, state: LedState) -> None:
        """Once/sec, log the achieved update() rate plus the current spread seen within that
        window (min/max/mean vs. target) — the rate alone (previously all this logged) says
        nothing about whether the loop is actually settling or oscillating around target."""
        self._loop_iters += 1
        self._loop_current_min = min(self._loop_current_min, state.current_a)
        self._loop_current_max = max(self._loop_current_max, state.current_a)
        self._loop_current_sum += state.current_a
        if state.settled:
            self._loop_settled_count += 1

        now = time.monotonic()
        if self._loop_rate_log_t == 0.0:
            self._loop_rate_log_t = now
            return
        elapsed = now - self._loop_rate_log_t
        if elapsed >= 1.0:
            mean_a = self._loop_current_sum / self._loop_iters
            logger.info(
                "LightingTask: sphere loop %.1f Hz (%d updates/%.2fs) — target=%.4f A "
                "mean=%.4f A min=%.4f A max=%.4f A spread=%.4f A settled=%d/%d code=%d",
                self._loop_iters / elapsed, self._loop_iters, elapsed,
                state.target_current_a if state.target_current_a is not None else float("nan"),
                mean_a, self._loop_current_min, self._loop_current_max,
                self._loop_current_max - self._loop_current_min,
                self._loop_settled_count, self._loop_iters, state.code,
            )
            self._loop_iters = 0
            self._loop_current_min = float("inf")
            self._loop_current_max = float("-inf")
            self._loop_current_sum = 0.0
            self._loop_settled_count = 0
            self._loop_rate_log_t = now

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
