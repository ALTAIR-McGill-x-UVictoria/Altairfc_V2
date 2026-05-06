from __future__ import annotations

import logging
import time
import numpy as np
from config.settings import ControllerConfig, MotorControlConfig, SerialPortConfig
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
        motor_control: MotorControlConfig | None = None,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._vesc_port = vesc_port.port
        self._activate_altitude_m = motor_control.activate_altitude_m if motor_control else 0.0
        self._run_duration_s      = motor_control.run_duration_min * 60.0 if motor_control else float("inf")
        self._activate_time       = float("inf")
        self.controller = Controller(controller_config, period_s)
        

    def setup(self) -> None:
        from tasks.flight_stage_task import STAGE_LAUNCH
        self.motor = None
        try:
            self.motor = VESCObject(self._vesc_port)
            logger.info("MMTask: VESC connected on %s", self._vesc_port)
        except Exception as e:
            logger.error("MMTask: failed to connect VESC on %s: %s", self._vesc_port, e)
            return

        # Poll telemetry during preflight so mm.* keys are available for preflight checks
        logger.info("MMTask: polling VESC telemetry, waiting for LAUNCH and altitude >= %.0f m", self._activate_altitude_m)
        while not self._stop_event.is_set():
            self._store()
            stage = int(self.datastore.read("event.flight_stage", default=0))
            alt   = float(self.datastore.read("gps.alt_msl", default=0.0))
            if stage >= STAGE_LAUNCH and alt >= self._activate_altitude_m:
                break
            self._stop_event.wait(timeout=0.5)

        if self._stop_event.is_set():
            return

        self._activate_time = time.monotonic()
        logger.info("MMTask: LAUNCH + altitude reached — braking payload")
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
        if time.monotonic() - self._activate_time > self._run_duration_s:
            logger.info("MMTask: run duration elapsed — stopping motor")
            self.motor.set_current(0)
            self._stop_event.set()
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
        if self.motor is None:
            return
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
