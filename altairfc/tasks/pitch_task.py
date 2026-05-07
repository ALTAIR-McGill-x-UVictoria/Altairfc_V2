from __future__ import annotations

import logging
import math
from config.settings import GroundStationConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.servo import ServoPointer
from controls.error_computation import compute_error

logger = logging.getLogger(__name__)


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
        

    def setup(self) -> None:
        self._servo = ServoPointer()
        self._servo.connect()

    def execute(self) -> None:
        quat, pos, gs_pos, yaw = self._read()
        az_err, pitch_err = compute_error(quat, pos, gs_coords=gs_pos)


        self._write_pointing(yaw, az_err, pitch_err)
        self._servo.set_pitch_error(pitch_err)

    def teardown(self) -> None:
        self._servo.disconnect()


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
        gs_lat = self.datastore.read("command.gs_lat", default=None)
        gs_lon = self.datastore.read("command.gs_lon", default=None)
        gs_alt = self.datastore.read("command.gs_alt", default=None)
        gs_pos = (
            [float(gs_lat), float(gs_lon), float(gs_alt)]
            if all(v is not None for v in (gs_lat, gs_lon, gs_alt)) else self._default_gs_pos
        )
        yaw = float(self.datastore.read("mavlink.attitude.yaw", default=0.0))
        return quat, pos, gs_pos, yaw

    def _write_pointing(self, yaw: float, az_err: float, pitch_err_rad: float) -> None:
        target_heading = yaw + az_err
        desired_deflection_deg = -math.degrees(pitch_err_rad)
        achieved_deflection_deg = self._servo.achieved_deflection_deg(pitch_err_rad)
        source_angle_error_deg = desired_deflection_deg - achieved_deflection_deg
        self.datastore.write("pointing.target_heading_rad",     target_heading)
        self.datastore.write("pointing.heading_error_rad",      az_err)
        self.datastore.write("pointing.source_angle_deg",       achieved_deflection_deg)
        self.datastore.write("pointing.source_angle_error_deg", source_angle_error_deg)
