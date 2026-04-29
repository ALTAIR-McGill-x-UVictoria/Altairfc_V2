from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from enum import Enum

from core.datastore import DataStore

logger = logging.getLogger(__name__)

_RESTART_BACKOFF_BASE = 1.0   # seconds before first retry
_RESTART_BACKOFF_MAX  = 60.0  # cap on backoff


class TaskState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    STOPPING = "stopping"
    FAILED = "failed"
    RECOVERING = "recovering"


class BaseTask(ABC):
    """
    Abstract base class for all Python subtasks.

    Each task runs in its own daemon thread. The scheduler calls start() and stop().
    Subclasses implement setup(), execute(), and optionally teardown().

    The task loop calls execute() every period_s seconds, accounting for the time
    execute() itself takes (deadline scheduling, not fixed sleep).

    Non-critical tasks automatically restart after failures using exponential
    backoff (1 s, 2 s, 4 s … capped at 60 s). Critical tasks still trigger
    system shutdown on failure.
    """

    def __init__(self, name: str, period_s: float, datastore: DataStore, critical: bool = False) -> None:
        self.name = name
        self.period_s = period_s
        self.datastore = datastore
        self.critical = critical  # if True, failure triggers system shutdown
        self.state = TaskState.IDLE
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @abstractmethod
    def setup(self) -> None:
        """Called once before the loop starts. Open hardware connections here."""

    @abstractmethod
    def execute(self) -> None:
        """Called every period_s. Read hardware, write results to datastore."""

    def teardown(self) -> None:
        """Called after the loop exits. Close connections, release resources."""

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            logger.warning("Task %s is already running", self.name)
            return
        self._stop_event.clear()
        self.state = TaskState.IDLE
        self._thread = threading.Thread(
            target=self._run_loop,
            name=f"task-{self.name}",
            daemon=True,
        )
        self._thread.start()
        logger.info("Task %s started", self.name)

    def stop(self, timeout_s: float = 5.0) -> None:
        self.state = TaskState.STOPPING
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            if self._thread.is_alive():
                logger.warning("Task %s did not stop within %.1fs", self.name, timeout_s)
        logger.info("Task %s stopped", self.name)

    @property
    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run_loop(self) -> None:
        backoff = _RESTART_BACKOFF_BASE

        while not self._stop_event.is_set():
            # --- setup phase ---
            try:
                self.setup()
            except Exception:
                logger.exception("Task %s failed in setup()", self.name)
                if self.critical:
                    self.state = TaskState.FAILED
                    return
                self.state = TaskState.RECOVERING
                logger.warning("Task %s recovering in %.1fs", self.name, backoff)
                if self._stop_event.wait(timeout=backoff):
                    break
                backoff = min(backoff * 2, _RESTART_BACKOFF_MAX)
                continue

            backoff = _RESTART_BACKOFF_BASE  # reset after a clean setup
            self.state = TaskState.RUNNING

            # --- execute loop ---
            failed = False
            while not self._stop_event.is_set():
                deadline = time.monotonic() + self.period_s
                try:
                    self.execute()
                except Exception:
                    logger.exception("Task %s raised in execute()", self.name)
                    failed = True
                    break

                remaining = deadline - time.monotonic()
                if remaining > 0:
                    self._stop_event.wait(timeout=remaining)

            # --- teardown ---
            try:
                self.teardown()
            except Exception:
                logger.exception("Task %s failed in teardown()", self.name)

            if not failed or self._stop_event.is_set():
                break

            # execute() failed — decide whether to restart
            if self.critical:
                self.state = TaskState.FAILED
                return

            self.state = TaskState.RECOVERING
            logger.warning("Task %s recovering in %.1fs (attempt backoff)", self.name, backoff)
            if self._stop_event.wait(timeout=backoff):
                break
            backoff = min(backoff * 2, _RESTART_BACKOFF_MAX)

        self.state = TaskState.IDLE
