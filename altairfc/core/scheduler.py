from __future__ import annotations

import logging
import threading

from config.settings import SystemConfig
from core.datastore import DataStore
from core.task_base import BaseTask, TaskState

logger = logging.getLogger(__name__)


class TaskScheduler:
    """
    Central orchestrator for all subtasks.

    Responsibilities:
    - Register tasks before startup
    - Start and stop all tasks with a clean lifecycle
    - Monitor task health in a background thread
    - Set a shutdown_event if a critical task fails (consumed by main.py)
    """

    def __init__(self, datastore: DataStore, config: SystemConfig) -> None:
        self.datastore = datastore
        self.config = config
        self._tasks: dict[str, BaseTask] = {}
        self._monitor_thread: threading.Thread | None = None
        self._shutdown_event = threading.Event()

    @property
    def shutdown_event(self) -> threading.Event:
        return self._shutdown_event

    def register(self, task: BaseTask) -> None:
        if task.name in self._tasks:
            raise ValueError(f"Task '{task.name}' is already registered")
        task_cfg = self.config.get_task(task.name)
        if task_cfg is not None and not task_cfg.enabled:
            logger.info("Task %s is disabled in config, skipping registration", task.name)
            return
        self._tasks[task.name] = task
        logger.debug("Registered task: %s", task.name)

    def start_all(self) -> None:
        logger.info("Starting %d task(s)", len(self._tasks))
        for task in self._tasks.values():
            task.start()
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="scheduler-monitor",
            daemon=True,
        )
        self._monitor_thread.start()

    def stop_all(self, timeout_s: float = 5.0) -> None:
        logger.info("Stopping all tasks")
        for task in reversed(list(self._tasks.values())):
            task.stop(timeout_s=timeout_s)

    def get_task(self, name: str) -> BaseTask | None:
        return self._tasks.get(name)

    def _monitor_loop(self) -> None:
        interval = self.config.monitor_interval_s
        while not self._shutdown_event.is_set():
            self._shutdown_event.wait(timeout=interval)
            if self._shutdown_event.is_set():
                break
            for task in list(self._tasks.values()):
                if task.state == TaskState.FAILED:
                    if task.critical:
                        logger.error(
                            "Critical task %s has failed — triggering system shutdown", task.name
                        )
                        self._shutdown_event.set()
                        return
                    else:
                        logger.warning(
                            "Task %s has failed (non-critical, system continues)", task.name
                        )
                if not task.is_alive and task.state == TaskState.RUNNING:
                    logger.warning("Task %s thread died unexpectedly", task.name)
