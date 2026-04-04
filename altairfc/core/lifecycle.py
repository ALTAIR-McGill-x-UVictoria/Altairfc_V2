from __future__ import annotations

import logging
import signal
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from core.scheduler import TaskScheduler

logger = logging.getLogger(__name__)

shutdown_event = threading.Event()


def install_signal_handlers(scheduler: "TaskScheduler") -> None:
    """
    Install SIGINT and SIGTERM handlers that gracefully stop the scheduler
    and set the global shutdown_event so main.py can unblock.
    """

    def _handler(signum: int, frame: object) -> None:
        sig_name = signal.Signals(signum).name
        logger.info("Received %s — initiating graceful shutdown", sig_name)
        shutdown_event.set()
        scheduler.shutdown_event.set()

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    logger.debug("Signal handlers installed (SIGINT, SIGTERM)")
