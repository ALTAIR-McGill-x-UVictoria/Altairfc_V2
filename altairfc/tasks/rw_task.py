from __future__ import annotations

import logging
import time
import numpy as np
from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.vesc_interface import VESCObject
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
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._vesc_port = vesc_port.port
        self.controller = Controller(controller_config, period_s)
        

    def setup(self) -> None:
        self.motor = None
        try:
            self.motor = VESCObject(self._vesc_port)
            logger.info("Initialized VESC motor interface on port %s", self._vesc_port)
        except Exception as e:
            logger.error("Failed to initialize VESC motor interface on port %s: %s", self._vesc_port, e)
            return

        logger.info("Bringing reaction wheel up to speed")
        self._hold(self.motor.set_rpm, 1700, duration=5.0)
        while not self._stop_event.is_set():
            self._store()
            self.motor.set_rpm(1700)
            yaw_rate = abs(float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0
                                                     )))
            if yaw_rate < 0.1:
                break
            time.sleep(0.05)

    def execute(self) -> None:
        if self.motor is None:
            return
        self.controller.Kp        = float(self.datastore.read("settings.rw_kp",      default=self.controller.Kp))
        self.controller.Kd        = float(self.datastore.read("settings.rw_kd",      default=self.controller.Kd))
        self.controller.max_value = float(self.datastore.read("settings.rw_max_rpm", default=self.controller.max_value))
        self._store()
        quat, pos = self._read()
        az_err, _ = compute_error(quat, pos)
        control_signal = self.controller.output(az_err) + 1700
        self.motor.set_rpm(control_signal)

    def teardown(self) -> None:
        if self.motor is not None:
            self.motor.set_rpm(0)

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
        return quat, pos

    def _hold(self, fn, value, duration, dt = 0.05):
        start_time = time.time()
        while time.time() - start_time < duration:
            fn(value)
            time.sleep(dt)