from __future__ import annotations

import logging

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class VescTask(BaseTask):

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
        self.datastore.write("system.vesc_connected", 0.0)

    def execute(self) -> None:
        pass

    def teardown(self) -> None:
        self.datastore.write("system.vesc_connected", 0.0)
        logger.info("VescTask: teardown not yet implemented")
