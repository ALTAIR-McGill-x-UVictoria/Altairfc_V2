from __future__ import annotations

import logging

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class ControlTask(BaseTask):

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        rw_vesc_port: SerialPortConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._rw_vesc_port = rw_vesc_port.port
        self.rw_motor = None

    def setup(self) -> None:
        from drivers.vesc_interface import VESCObject

        self.rw_motor = VESCObject(self._rw_vesc_port)

    def execute(self) -> None:
        self.rw_motor.set_current(2000)

    def teardown(self) -> None:
        if self.rw_motor is not None:
            self.rw_motor.set_current(0)
