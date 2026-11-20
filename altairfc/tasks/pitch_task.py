from __future__ import annotations

import logging
import math
import time
from config.settings import GroundStationConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.servo import ServoPointer, nadir_angle_deg
from controls.error_computation import compute_error
from controls.ground_station import ground_station_position

logger = logging.getLogger(__name__)

# GPS counts as lost when no receiver has a 3D fix, or the fix / EKF position is older than this.
_GPS_STALE_S = 2.0
# pitch error of +90° = GS straight below = servo at nadir (0° deflection)
_NADIR_PITCH_ERR_RAD = math.pi / 2


class PitchTask(BaseTask):

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        ground_station: GroundStationConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._default_gs_pos = [
            ground_station.latitude,
            ground_station.longitude,
            ground_station.altitude,
        ]
        self._gps_lost: bool | None = None  # None until the first execute()

    def setup(self) -> None:
        self._servo = ServoPointer()
        self._servo.connect()

        while not self._stop_event.is_set():
            if int(self.datastore.read("event.pointing_active", default=0.0)) == 1:
                break
            self._stop_event.wait(timeout=0.5)

        if self._stop_event.is_set():
            return

    def execute(self) -> None:
        pointing_active = self.datastore.read("event.pointing_active", default=None)

        if pointing_active is None:
            logger.warning("pointing_active is missing")
            return

        if int(pointing_active) != 1:
            logger.info("PitchTask: run duration elapsed — stopping motor")
            self._stop_event.set()
            return

        gps_lost = not self._gps_ok()
        if gps_lost != self._gps_lost:
            if gps_lost:
                logger.warning("PitchTask: no GPS fix — pointing the sphere straight down")
            elif self._gps_lost is not None:
                logger.info("PitchTask: GPS fix back — pointing at the ground station")
            self._gps_lost = gps_lost
        if gps_lost:
            self._servo.set_pitch_error(_NADIR_PITCH_ERR_RAD)
            self.datastore.write("pointing.source_angle_deg", 0.0)
            return

        quat, pos, gs_pos, yaw = self._read()
        az_err, pitch_err = compute_error(quat, pos, gs_coords=gs_pos)
        self._write_pointing(yaw, az_err, pitch_err)
        self._servo.set_pitch_error(pitch_err)

    def teardown(self) -> None:
        self._servo.disconnect()


    def _gps_ok(self) -> bool:
        """True when a receiver has a fresh 3D fix and the EKF position is fresh."""
        now = time.monotonic()
        fix = self.datastore.read_with_timestamp("mavlink.gps.fix_type")
        pos = self.datastore.read_with_timestamp("mavlink.gps.lat")
        return (
            fix is not None and int(fix[0]) >= 3 and now - fix[1] < _GPS_STALE_S
            and pos is not None and now - pos[1] < _GPS_STALE_S
        )

    def _read(self):
        quat = [
            float(self.datastore.read("mavlink.quaternion.x", default=0.0)),
            float(self.datastore.read("mavlink.quaternion.y", default=0.0)),
            float(self.datastore.read("mavlink.quaternion.z", default=0.0)),
            float(self.datastore.read("mavlink.quaternion.w", default=1.0)),
        ]
        pos = [
            float(self.datastore.read("mavlink.gps.lat", default=0.0)),
            float(self.datastore.read("mavlink.gps.lon", default=0.0)),
            float(self.datastore.read("mavlink.gps.alt", default=0.0)),
        ]
        gs_pos = ground_station_position(self.datastore, self._default_gs_pos)
        yaw = float(self.datastore.read("mavlink.attitude.yaw", default=0.0))
        return quat, pos, gs_pos, yaw

    def _write_pointing(self, yaw: float, az_err: float, pitch_err_rad: float) -> None:
        target_heading = (yaw + az_err + math.pi) % (2.0 * math.pi) - math.pi
        desired_deflection_deg = nadir_angle_deg(pitch_err_rad)
        achieved_deflection_deg = self._servo.achieved_deflection_deg(pitch_err_rad)
        source_angle_error_deg = desired_deflection_deg - achieved_deflection_deg
        self.datastore.write("pointing.target_heading_rad",     target_heading)
        self.datastore.write("pointing.heading_error_rad",      az_err)
        self.datastore.write("pointing.source_angle_deg",       achieved_deflection_deg)
        self.datastore.write("pointing.source_angle_error_deg", source_angle_error_deg)
