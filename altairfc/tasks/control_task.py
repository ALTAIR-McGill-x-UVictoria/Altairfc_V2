from __future__ import annotations

import logging

from drivers.vesc_interface import VESCObject
from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger()


class ControlTask(BaseTask):

    def __init__(
        self,
        name: str,
        datastore: DataStore,
        rw_vesc_port: SerialPortConfig,
    ) -> None:
        self._rw_vesc_port = rw_vesc_port
        self.datastore = datastore

    def setup(self) -> None:
        self.rw_motor = VESCObject(self._rw_vesc_port.port)

    def execute(self) -> None:
        self.rw_motor.set_current(1000)

    def teardown(self) -> None:
        self.rw_motor.set_current(0)
