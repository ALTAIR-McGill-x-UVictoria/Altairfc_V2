from __future__ import annotations

import logging
import time
from config.settings import ControllerConfig, SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.vesc_interface import VESCObject
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
        ## Checking VESC Connection
        self.motor = None
        try:
            self.motor = VESCObject(self._vesc_port)
            logger.info("MMTask: VESC connected on %s", self._vesc_port)
        except Exception as e:
            logger.error("MMTask: failed to connect VESC on %s: %s", self._vesc_port, e)
            return

        ## Polling Telemetry During Preflight
        while not self._stop_event.is_set():
            self._store()
            if int(self.datastore.read("event.pointing_active", default=0.0)) == 1:
                break
            self._stop_event.wait(timeout=0.5)

        if self._stop_event.is_set():
            return
        
        ## Stabilizing and Braking Payload
        logger.info("MMTask: LAUNCH + altitude reached — braking payload")
        while not self._stop_event.is_set():
            self._store()
            yaw_rate = self.datastore.read("mavlink.attitude.yawspeed", default=None)
            if yaw_rate is None:
                logger.warning("mavlink.attitude.yawspeed is missing")
                continue
            motor_rpm = float(self.datastore.read("rw.rpm", default=0.0))
            self.motor.set_brake_current(1650)
            time.sleep(0.05)
            if abs(float(yaw_rate)) < 0.1 and motor_rpm >= 1700:
                break

    def execute(self) -> None:
        if self.motor is None:
            return
        
        pointing_active = self.datastore.read("event.pointing_active", default=None)

        if pointing_active is None:
            logger.warning("pointing_active is missing")
            return

        if int(pointing_active) != 1:
            logger.info("MMTask: run duration elapsed — stopping motor")
            self._stop_event.set()
            return
        
        self._store()
        motor_speed_err = float(self.datastore.read("rw.rpm", default=0.0)) - 1700
        control_signal = self.controller.output(motor_speed_err)
        self.motor.set_current(int(control_signal))

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
            self._store()
            fn(value)
            time.sleep(dt)
