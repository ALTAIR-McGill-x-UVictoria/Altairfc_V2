from __future__ import annotations

import logging
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
        try:
            self.motor = VESCObject(self._vesc_port)
            logger.info("Initialized VESC motor interface on port %s", self._vesc_port)

        except Exception as e:
            logger.error("Failed to initialize VESC motor interface on port %s: %s", self._vesc_port, e)

        # Start momentum management by braking payload and bringing rotation rate to stable threshold before starting control sequence
        logger.info("Braking payload")
        yaw_rate = float(self.datastore.read("mavlink.attitude.yaw_speed", default=0.0))
        while yaw_rate > 0.1:
            self.motor.set_brake_current(1650) # 3.3A estimated for 0.4Nm braking torque based on flight data
        return



    def execute(self) -> None:
        motor_speed_err = float(self.datastore.read("rw.motor_speed")) - 1700
        control_signal = self.controller.output(motor_speed_err)
        self.motor.set_current(control_signal)



    def teardown(self) -> None:
        if self.motor is not None:
            self.motor.set_current(0)
