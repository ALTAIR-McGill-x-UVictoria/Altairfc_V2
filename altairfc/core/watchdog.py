from __future__ import annotations

import logging
import os
import socket
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from core.scheduler import TaskScheduler

logger = logging.getLogger(__name__)


def _sd_notify(msg: str) -> None:
    """Send a message to systemd via $NOTIFY_SOCKET (no-op if not running under systemd)."""
    path = os.environ.get("NOTIFY_SOCKET")
    if not path:
        return
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.sendto(msg.encode(), path)
    except OSError:
        logger.debug("sd_notify failed (socket=%s)", path)


class WatchdogThread:
    """
    Pets the systemd watchdog at half the configured WatchdogSec interval.

    The ping is withheld if any critical task has entered the FAILED state,
    which lets systemd's timeout expire and triggers a process restart.

    Usage:
        wd = WatchdogThread(scheduler, watchdog_sec=30.0)
        wd.start()
        ...
        wd.stop()
    """

    def __init__(self, scheduler: "TaskScheduler", watchdog_sec: float) -> None:
        self._scheduler = scheduler
        self._interval = watchdog_sec / 2.0
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._loop,
            name="watchdog",
            daemon=True,
        )

    def start(self) -> None:
        _sd_notify("READY=1")
        self._thread.start()
        logger.info("Watchdog started (ping interval %.1fs)", self._interval)

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=self._interval + 1)
        _sd_notify("STOPPING=1")
        logger.info("Watchdog stopped")

    def _loop(self) -> None:
        from core.task_base import TaskState

        while not self._stop_event.wait(timeout=self._interval):
            if self._scheduler.shutdown_event.is_set():
                # Graceful shutdown in progress — keep petting so systemd doesn't
                # kill us before stop_all() finishes.
                _sd_notify("WATCHDOG=1")
                continue

            critical_failed = any(
                t.critical and t.state == TaskState.FAILED
                for t in self._scheduler.tasks.values()
            )
            if critical_failed:
                logger.error("Watchdog witholding heartbeat — critical task failed")
                # Don't ping; let systemd's WatchdogSec timeout expire.
            else:
                _sd_notify("WATCHDOG=1")
