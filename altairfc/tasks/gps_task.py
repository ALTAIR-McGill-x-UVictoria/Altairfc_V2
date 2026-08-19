from __future__ import annotations

import calendar
import logging
import time
from datetime import datetime, timezone
from typing import Callable

from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.gps_driver import GpsDriver
from drivers.mcp23017 import MCP23017, HIGH, LOW



logger = logging.getLogger(__name__)

_PING_INTERVAL_S = 10.0

# UBX-NAV-PVT validFlags bits (fix.time_valid)
_VALID_DATE = 0x01
_VALID_TIME = 0x02
_VALID_DATETIME = _VALID_DATE | _VALID_TIME

# Only step the system clock (and fire on_time_sync) once GPS and system time
# disagree by more than this. The Pi has no RTC and no network in flight, so
# the clock is wrong until the first GPS fix; this also becomes irrelevant
# once chrony/PPS is disciplining the clock (small drift, no correction needed).
_CLOCK_CORRECTION_THRESHOLD_S = 1.0


class GpsTask(BaseTask):
    """
    Reads the u-blox MAX-M10M GPS module over I2C via the C gps_driver shared library.

    Polls UBX-NAV-PVT at the configured period (1 Hz default) and writes results
    to the DataStore under the "gps.*" namespace.

    gps.active is updated every _PING_INTERVAL_S seconds by probing the DDC
    byte-count register independently of the PVT poll, so it reflects true I2C
    presence rather than fix availability.

    DataStore keys written:
        gps.active       (int, 1 if module responding to I2C)
        gps.lat          (float, deg)
        gps.lon          (float, deg)
        gps.alt_msl      (float, m)
        gps.speed_ms     (float, m/s)
        gps.heading_deg  (float, deg)
        gps.hdop         (float)
        gps.fix_type     (int, 0=no fix / 2=2D / 3=3D / 4=GNSS+DR)
        gps.num_sv       (int)
        gps.valid        (int, 1 if gnssFixOK)
        gps.time_valid   (int, UBX validFlags bitmask)
        gps.utc_hour     (int)
        gps.utc_min      (int)
        gps.utc_sec      (int)
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        i2c_dev: str = "/dev/i2c-1",
        on_time_sync: Callable[[datetime], None] | None = None,
    ) -> None:
        super().__init__(name, period_s, datastore)
        self._i2c_dev = i2c_dev
        self._driver: GpsDriver | None = None
        self._last_ping: float = 0.0
        self._on_time_sync = on_time_sync
        self._time_synced_once = False

    def setup(self) -> None:
        self._driver = GpsDriver(i2c_dev=self._i2c_dev)
        self.datastore.write("gps.active", 1)
        self._last_ping = time.monotonic()
        logger.info("GpsTask: driver ready on %s", self._i2c_dev)
        self.io = MCP23017()
        self._timepulse_led = 0
        self.io.set_output(self._timepulse_led)

        

        

    def execute(self) -> None:
        if self._driver is None:
            self.datastore.write("gps.active", 0)
            return

        now = time.monotonic()
        if now - self._last_ping >= _PING_INTERVAL_S:
            active = 1 if self._driver.ping() else 0
            self.datastore.write("gps.active", active)
            self._last_ping = now
            if not active:
                logger.warning("GpsTask: module not responding to I2C ping")

        fix = self._driver.read()
        if fix is None:
            return

        self.datastore.write("gps.lat",         fix.lat)
        self.datastore.write("gps.lon",         fix.lon)
        self.datastore.write("gps.alt_msl",     fix.alt_msl)
        self.datastore.write("gps.speed_ms",    float(fix.speed_ms))
        self.datastore.write("gps.heading_deg", float(fix.heading_deg))
        self.datastore.write("gps.hdop",        float(fix.hdop))
        self.datastore.write("gps.fix_type",    int(fix.fix_type))
        self.datastore.write("gps.num_sv",      int(fix.num_sv))
        self.datastore.write("gps.valid",       int(fix.valid))
        self.datastore.write("gps.time_valid",  int(fix.time_valid))
        self.datastore.write("gps.utc_hour",    int(fix.hour))
        self.datastore.write("gps.utc_min",     int(fix.min))
        self.datastore.write("gps.utc_sec",     int(fix.sec))

        if fix.time_valid & _VALID_DATETIME == _VALID_DATETIME:
            self._maybe_sync_clock(fix)

        self.io.set(self._timepulse_led, HIGH if fix.valid else LOW)
        if fix.valid:
            logger.debug(
                "GpsTask: fix=3D sv=%d lat=%.6f lon=%.6f alt=%.1fm spd=%.1fm/s",
                fix.num_sv, fix.lat, fix.lon, fix.alt_msl, fix.speed_ms,
            )

    def _maybe_sync_clock(self, fix) -> None:
        """
        Step the system clock to the GPS-reported UTC time if they disagree
        by more than _CLOCK_CORRECTION_THRESHOLD_S. On the first correction,
        fire on_time_sync so the caller can fix up anything (e.g. the
        already-created, wrongly-dated log session directory) that was named
        from the system clock before GPS time became available.
        """
        try:
            gps_dt = datetime(
                fix.year, fix.month, fix.day, fix.hour, fix.min, fix.sec,
                tzinfo=timezone.utc,
            )
        except ValueError:
            return  # not-yet-converged receiver can report a garbage date

        gps_epoch = calendar.timegm(gps_dt.timetuple())
        drift_s = gps_epoch - time.time()
        if abs(drift_s) < _CLOCK_CORRECTION_THRESHOLD_S:
            return

        try:
            time.clock_settime(time.CLOCK_REALTIME, gps_epoch)
        except (PermissionError, OSError) as exc:
            logger.warning("GpsTask: could not set system clock from GPS fix: %s", exc)
            return

        logger.info(
            "GpsTask: system clock corrected from GPS fix (%+.1fs) -> %s",
            drift_s, gps_dt.isoformat(),
        )

        if not self._time_synced_once:
            self._time_synced_once = True
            if self._on_time_sync is not None:
                try:
                    self._on_time_sync(gps_dt)
                except Exception:
                    logger.exception("GpsTask: on_time_sync callback failed")

    def teardown(self) -> None:
        self.io.set(self._timepulse_led, LOW)
        self.io.close()
        if self._driver is not None:
            self._driver.close()
            self._driver = None
            logger.info("GpsTask: driver closed")
