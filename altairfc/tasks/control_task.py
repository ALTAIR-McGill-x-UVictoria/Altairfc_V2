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
        datastore: DataStore,
        rw_vesc_port: SerialPortConfig,
        mm_vesc_port: SerialPortConfig,
    ) -> None:
        self._rw_vesc_port = rw_vesc_port
        self._mm_vesc_port = mm_vesc_port
        self.datastore = datastore
    def setup(self) -> None:
        rw_motor = VESCObject(self._rw_vesc_port.port)
        mm_motor = VESCObject(self._mm_vesc_port.port)