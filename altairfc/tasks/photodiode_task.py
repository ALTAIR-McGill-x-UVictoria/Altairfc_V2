from __future__ import annotations

import logging

from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class PhotodiodeTask(BaseTask):
    """
    Photodiode interface HAT task (stub).

    DataStore keys to write:
        photodiode.channel_0
        photodiode.channel_1
        photodiode.channel_2
        photodiode.channel_3
    """

    def setup(self) -> None:
        logger.info("PhotodiodeTask: setup not yet implemented")
        self.datastore.write("system.photodiode_connected", 0.0)

    def execute(self) -> None:
        pass

    def teardown(self) -> None:
        self.datastore.write("system.photodiode_connected", 0.0)
        logger.info("PhotodiodeTask: teardown not yet implemented")
