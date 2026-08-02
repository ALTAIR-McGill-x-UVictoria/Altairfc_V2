from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.beacon_led import BEACON_MAX_SAFE_CODE, BeaconChannel
from drivers.led_board import InterlockViolation, LedBoard
from drivers.sphere_led import SphereLedSource

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


class LightingTask(BaseTask):
    """
    Two independent behaviors sharing one interlocked LED board:

      * Sphere source: once the observation phase begins (event.ascent_active
        becomes 1), engages SphereLedSource's current-hold PI loop and never
        turns it off internally — "do not turn off during the entire
        observation window" is implemented by NOT having an off-path in
        execute() at all. The sphere (and the beacon) are only ever turned
        off by this task being stopped: FlightStageTask calls scheduler
        .get_task("lighting").stop() at apogee (burst/termination detected,
        mirroring how it already stops "pointing"), which ends this task's
        run loop and invokes teardown(), which zeroes both channels.

      * Spotter beacon: strobes on a fixed GPS-UTC schedule (beacon_windows,
        e.g. 5s flashes at :25 and :55 of every minute) whenever the sphere
        isn't on. Brightness (MCP4728 channel 1) is set once; each flash
        toggles the relay (channel 3) on/off — see drivers/beacon_led.py.

    The sphere and the beacon's relay are never energized together —
    enforced by drivers.led_board.LedBoard, which both SphereLedSource and
    BeaconChannel share.

    Imaging is done by ground-based cameras/telescopes, not an onboard camera:
    since the downlink is unidirectional, ground observers independently know
    the same UTC window schedule, so no uplink command is needed to
    coordinate exposure with the sphere source turning on.

    UTC source: the Pi's system clock (datetime.now(timezone.utc)), which is
    chrony/GPS-PPS-disciplined pre-launch. Per project decision this is used
    unconditionally, whether or not chrony is currently PPS-disciplined in
    flight (see system.pps_synced for that diagnostic) — there is no separate
    fallback branch.

    DataStore keys read:
        event.ascent_active          — 1 once ascent is detected; marks the start
                                        of the sphere's observation window

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
        beacon_dac_code: int = 1500,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._i2c_dev = i2c_dev
        self._beacon_windows = parse_windows(beacon_windows)
        self._sphere_target_current_a = sphere_target_current_a
        self._beacon_dac_code = max(0, min(BEACON_MAX_SAFE_CODE, int(beacon_dac_code)))

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

    def setup(self) -> None:
        try:
            import smbus2
            self._bus = smbus2.SMBus(int(self._i2c_dev.replace("/dev/i2c-", "")))
            self._board = LedBoard(bus=self._bus, i2c_dev=self._i2c_dev)
            self._sphere = SphereLedSource(board=self._board)
            self._beacon = BeaconChannel(board=self._board)
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
            self._apply(observation_active, active_beacon_window)

        self.datastore.write("lighting.sphere_on", int(self._sphere_on))
        self.datastore.write("lighting.beacon_on", int(self._beacon_on))
        self.datastore.write("lighting.observation_active", int(observation_active))
        self.datastore.write("lighting.next_beacon_flash_in_s", next_in_s)

    def _apply(self, observation_active: bool, active_beacon_window: ImagingWindow | None) -> None:
        # One-shot latch: once the sphere turns on there is no internal path
        # back off. It only ever stops via this task being stopped (see the
        # class docstring) — teardown() zeroes it unconditionally.
        if observation_active and not self._sphere_on and self._sphere_target_current_a is not None:
            self._set_beacon(False)
            self._sphere.hold_current(self._sphere_target_current_a)
            self._sphere_on = True
            logger.info(
                "LightingTask: sphere on (target %.4f A) — will not turn off internally; "
                "stopped externally at apogee",
                self._sphere_target_current_a,
            )

        if self._sphere_on:
            self._set_beacon(False)
            try:
                self._sphere.update()
            except InterlockViolation:
                logger.error("LightingTask: sphere current-hold step blocked by interlock — beacon did not turn off")
        else:
            self._set_beacon(active_beacon_window is not None)

    def _set_beacon(self, on: bool) -> None:
        if on == self._beacon_on:
            return
        if on:
            try:
                self._beacon.on()
                self._beacon_on = True
            except InterlockViolation:
                logger.error("LightingTask: beacon blocked by interlock — sphere is on")
                self._beacon_on = False
        else:
            self._beacon.off()
            self._beacon_on = False

    def teardown(self) -> None:
        if self._sphere is not None:
            self._sphere.all_off()
        if self._beacon is not None:
            self._beacon.all_off()  # zeros both brightness (ch1) and relay (ch3)
        if self._board is not None:
            self._board.close()
        if self._bus is not None:
            self._bus.close()
