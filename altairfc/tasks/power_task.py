from __future__ import annotations

import logging

from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class PowerTask(BaseTask):
    """
    Power distribution HAT task (stub).

    DataStore keys to write:
        power.voltage_bus
        power.current_total
        power.temperature
    """

    def setup(self) -> None:
        logger.info("PowerTask: setup not yet implemented")

    def execute(self) -> None:
        pass

    def teardown(self) -> None:
        logger.info("PowerTask: teardown not yet implemented")
