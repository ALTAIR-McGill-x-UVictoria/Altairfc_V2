from __future__ import annotations

import logging

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class ControlTask(BaseTask):

    def __init__(
        self,
        datastore: DataStore,
    ) -> None:
        