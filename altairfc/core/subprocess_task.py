from __future__ import annotations

import json
import logging
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Literal

from core.datastore import DataStore
from core.task_base import BaseTask, TaskState

logger = logging.getLogger(__name__)


class SubprocessTask(BaseTask):
    """
    Wraps a C/C++ executable as a managed subtask.

    The binary must write newline-delimited JSON records to stdout:
        {"key": "sensor.value_name", "value": 3.14, "ts": 1234567890.123}

    Each record is parsed and written to the DataStore immediately.
    stderr is captured and forwarded to the Python logger.

    The execute() loop is a health check: it verifies the process is alive
    and optionally restarts it if auto_restart=True.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        executable_path: Path,
        args: list[str] | None = None,
        protocol: Literal["json_lines"] = "json_lines",
        auto_restart: bool = True,
    ) -> None:
        super().__init__(name, period_s, datastore)
        self.executable_path = Path(executable_path)
        self.args = args or []
        self.protocol = protocol
        self.auto_restart = auto_restart
        self._proc: subprocess.Popen | None = None
        self._reader_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None

    def setup(self) -> None:
        self._launch_process()

    def execute(self) -> None:
        if self._proc is None or self._proc.poll() is not None:
            exit_code = self._proc.poll() if self._proc else "N/A"
            logger.warning("Task %s: subprocess exited (code=%s)", self.name, exit_code)
            if self.auto_restart:
                logger.info("Task %s: restarting subprocess", self.name)
                self._launch_process()

    def teardown(self) -> None:
        if self._proc is None:
            return
        try:
            self._proc.send_signal(signal.SIGTERM)
            self._proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            logger.warning("Task %s: subprocess did not terminate, sending SIGKILL", self.name)
            self._proc.kill()
            self._proc.wait()
        except Exception:
            logger.exception("Task %s: error during subprocess teardown", self.name)

    def _launch_process(self) -> None:
        cmd = [str(self.executable_path)] + self.args
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._reader_thread = threading.Thread(
            target=self._read_stdout,
            name=f"task-{self.name}-stdout",
            daemon=True,
        )
        self._reader_thread.start()
        self._stderr_thread = threading.Thread(
            target=self._read_stderr,
            name=f"task-{self.name}-stderr",
            daemon=True,
        )
        self._stderr_thread.start()
        logger.info("Task %s: launched subprocess PID=%d", self.name, self._proc.pid)

    def _read_stdout(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        for line in self._proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
                key = record["key"]
                value = record["value"]
                ts = record.get("ts", time.monotonic())
                self.datastore.write(key, value, timestamp=ts)
            except (json.JSONDecodeError, KeyError):
                logger.warning("Task %s: malformed stdout line: %r", self.name, line)

    def _read_stderr(self) -> None:
        assert self._proc is not None and self._proc.stderr is not None
        for line in self._proc.stderr:
            line = line.strip()
            if line:
                logger.warning("Task %s [stderr]: %s", self.name, line)
