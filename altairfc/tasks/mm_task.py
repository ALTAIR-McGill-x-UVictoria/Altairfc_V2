from __future__ import annotations

import logging
import time
import numpy as np
from config.settings import SerialPortConfig, ControllerConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.vesc_interface import VESCObject
from controls.error_computation import compute_error
from controls.controller import Controller

logger = logging.getLogger(__name__)


class MMTask(BaseTask):

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        vesc_port: SerialPortConfig,
        controller_config: ControllerConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._vesc_port = vesc_port.port
        self.controller = Controller(controller_config, period_s)
        

    def setup(self) -> None:
        from tasks.flight_stage_task import STAGE_LAUNCH
        logger.info("MMTask: waiting for LAUNCH command before starting")
        while not self._stop_event.is_set():
            stage = int(self.datastore.read("event.flight_stage", default=0))
            if stage >= STAGE_LAUNCH:
                break
            self._stop_event.wait(timeout=0.5)

        if self._stop_event.is_set():
            return

        self.motor = None
        try:
            self.motor = VESCObject(self._vesc_port)
            logger.info("Initialized VESC motor interface on port %s", self._vesc_port)
        except Exception as e:
            logger.error("Failed to initialize VESC motor interface on port %s: %s", self._vesc_port, e)
            return

        logger.info("Braking payload")
        while not self._stop_event.is_set():
            yaw_rate = float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0))
            motor_rpm = float(self.datastore.read("rw.rpm", default=0.0))
            self.motor.set_brake_current(1650)
            time.sleep(0.05)
            if yaw_rate < 0.1 and motor_rpm >= 1700:
                break

    def execute(self) -> None:
        if self.motor is None:
            return
        self._store()
        self.controller.Kp        = float(self.datastore.read("settings.mm_kp",          default=self.controller.Kp))
        self.controller.Kd        = float(self.datastore.read("settings.mm_kd",          default=self.controller.Kd))
        self.controller.max_value = float(self.datastore.read("settings.mm_max_current", default=self.controller.max_value))
        motor_speed_err = float(self.datastore.read("rw.rpm", default=0.0)) - 1700
        control_signal = self.controller.output(motor_speed_err)
        self.motor.set_current(10)
        time.sleep(0.1)
        self.motor.set_current(0)

    def teardown(self) -> None:
        if self.motor is not None:
            self.motor.set_current(0)

    def _store(self) -> None:
        try:
            data = self.motor.get_data(timeout=0.3)
        except Exception as e:
            logger.error("MM VESC disconnected during data read: %s", e)
            self.motor = None
            return
        if data:
            for f in ('rpm', 'duty_now', 'current_motor', 'current_in',
                      'v_in', 'temp_pcb', 'amp_hours', 'tachometer',
                      'tachometer_abs'):
                self.datastore.write(f"mm.{f}", getattr(data, f, 0.0))
            fault = getattr(data, 'mc_fault_code', b'\x00')
            self.datastore.write("mm.mc_fault_code", fault[0] if isinstance(fault, (bytes, bytearray)) else int(fault))

    def _hold(self, fn, value, duration, dt = 0.05):
        start_time = time.time()
        while time.time() - start_time < duration:
            fn(value)
            time.sleep(dt)
