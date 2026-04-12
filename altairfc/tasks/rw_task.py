from __future__ import annotations

import logging
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
        try:
            self.motor = VESCObject(self._vesc_port)
            logger.info("Initialized VESC motor interface on port %s", self._vesc_port)

        except Exception as e:
            logger.error("Failed to initialize VESC motor interface on port %s: %s", self._vesc_port, e)

        logger.info("Bringing reaction wheel up to speed")
        yaw_rate = float(self.datastore.read("mavlink.attitude.yaw_speed"))
        self.motor.set_rpm(1700) 
        if yaw_rate < 0.1:
            return


    def execute(self) -> None:
        quat = [
            float(self.datastore.read("mavlink.quaternion.x")),
            float(self.datastore.read("mavlink.quaternion.y")),
            float(self.datastore.read("mavlink.quaternion.z")),
            float(self.datastore.read("mavlink.quaternion.w"))
        ]
        pos = [
            float(self.datastore.read("mavlink.gps.lat")),
            float(self.datastore.read("mavlink.gps.lon")),
            float(self.datastore.read("mavlink.gps.alt"))
        ]
        az_err, _ = compute_error(quat, pos)
        control_signal = self.controller.output(az_err) + 1700
        self.motor.set_rpm(control_signal)


    def teardown(self) -> None:
        if self.motor is not None:
            self.motor.set_rpm(0)
