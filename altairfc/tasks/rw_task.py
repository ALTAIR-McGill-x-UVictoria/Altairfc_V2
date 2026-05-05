from __future__ import annotations

import logging
import math
import time
from config.settings import GroundStationConfig, SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.vesc_interface import VESCObject
from drivers.servo import ServoPointer
from controls.error_computation import compute_error
from controls.controller import Controller

logger = logging.getLogger(__name__)


class RWTask(BaseTask):

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        vesc_port: SerialPortConfig,
        controller_config: list,
        ground_station: GroundStationConfig,
        pointing_enabled: bool = True,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._vesc_port = vesc_port.port
        self._default_gs_pos = [
            ground_station.latitude,
            ground_station.longitude,
            ground_station.altitude,
        ]
        self._pointing_enabled = pointing_enabled
        self.controller = Controller(controller_config, period_s)
        

    def setup(self) -> None:
        self.motor = None
        self._servo = ServoPointer()
        self._servo.connect()

        try:
            self.motor = VESCObject(self._vesc_port)
            logger.info("Initialized VESC motor interface on port %s", self._vesc_port)
        except Exception as e:
            logger.error("Failed to initialize VESC motor interface on port %s: %s", self._vesc_port, e)
            return

        logger.info("Bringing reaction wheel up to speed")
        self._hold(self.motor.set_rpm, 1700, duration=5.0)
        while not self._stop_event.is_set():
            logger.info("Stabilizing Payload")
            self._store()
            self.motor.set_rpm(1700)
            yaw_rate = abs(float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0)))
            if yaw_rate < 0.1:
                return
            time.sleep(0.05)

    def execute(self) -> None:
        quat, pos, gs_pos, yaw_rate, yaw = self._read()
        az_err, pitch_err = compute_error(quat, pos, gs_coords=gs_pos)

        logger.debug("gs_pos: lat=%f lon=%f alt=%f", gs_pos[0], gs_pos[1], gs_pos[2])

        if self._pointing_enabled:
            self._write_pointing(yaw, az_err, pitch_err)
            self._servo.set_pitch_error(pitch_err)

        if self.motor is None:
            return
        self._store()
        control_signal = self.controller.output(az_err, yaw_rate) + 1700.0
        logger.info("yaw_error:%f, control signal: %f", az_err, control_signal)
        self.motor.set_rpm(int(control_signal))

    def teardown(self) -> None:
        if self.motor is not None:
            self.motor.set_rpm(0)
        self._servo.disconnect()

    def _store(self):
        data = self.motor.get_data(timeout=0.3)
        if data:
            # Write all GetValues fields into the datastore under the 'rw.' namespace
            fields = [
                'can_id', 'temp_fet', 'temp_motor', 'avg_motor_current', 'avg_input_current', 'avg_id',
                'avg_iq', 'duty_cycle_now', 'rpm', 'v_in', 'amp_hours', 'amp_hours_charged',
                'watt_hours', 'watt_hours_charged','tachometer', 'tachometer_abs', 'mc_fault_code',
                'pid_pos_now', 'app_controller_id', 'time_ms',
            ]
            for f in fields:
                try:
                    val = getattr(data, f, None)
                except Exception:
                    val = None
                self.datastore.write(f"rw.{f}", val)

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
        yaw_rate = float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0))
        yaw = float(self.datastore.read("mavlink.attitude.yaw", default=0.0))
        return quat, pos, gs_pos, yaw_rate, yaw

    def _write_pointing(self, yaw: float, az_err: float, pitch_err_rad: float) -> None:
        target_heading = yaw + az_err
        desired_deflection_deg = -math.degrees(pitch_err_rad)
        achieved_deflection_deg = self._servo.achieved_deflection_deg(pitch_err_rad)
        source_angle_error_deg = desired_deflection_deg - achieved_deflection_deg
        self.datastore.write("pointing.target_heading_rad",     target_heading)
        self.datastore.write("pointing.heading_error_rad",      az_err)
        self.datastore.write("pointing.source_angle_deg",       achieved_deflection_deg)
        self.datastore.write("pointing.source_angle_error_deg", source_angle_error_deg)

    def _hold(self, fn, value, duration, dt = 0.05):
        start_time = time.time()
        while time.time() - start_time < duration:
            fn(value)
            time.sleep(dt)
