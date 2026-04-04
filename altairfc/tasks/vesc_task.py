from __future__ import annotations

import logging

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class VescTask(BaseTask):
    """
    VESC motor controller task (stub).

    DataStore keys to write:
        vesc.rpm
        vesc.duty_cycle
        vesc.motor_current
        vesc.input_voltage
        vesc.temperature_mos
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        port_config: SerialPortConfig,
    ) -> None:
        super().__init__(name, period_s, datastore)
        self._port_config = port_config

    def setup(self) -> None:
        logger.info("VescTask: setup not yet implemented")

    def execute(self) -> None:
        pass

    def teardown(self) -> None:
        logger.info("VescTask: teardown not yet implemented")
