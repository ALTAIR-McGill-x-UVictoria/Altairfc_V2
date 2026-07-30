from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from config.settings import ProfilingConfig

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class TaskCpuSample:
    name: str
    cpu_pct: float
    calls: int
    average_wall_ms: float
    maximum_wall_ms: float


@dataclass(frozen=True)
class ProfileReport:
    interval_s: float
    altairfc_cpu_pct: float
    task_cpu_pct: float
    other_threads_cpu_pct: float
    tasks: list[TaskCpuSample]


@dataclass
class _TaskTotals:
    cpu_s: float = 0.0
    wall_s: float = 0.0
    calls: int = 0
    maximum_wall_s: float = 0.0


class TaskCpuProfiler:
    """
    Attributes AltairFC CPU time to scheduler tasks.

    BaseTask measures each execute() call with time.thread_time(), which counts
    CPU used by only that task's thread. Overall process CPU comes from
    time.process_time(); the difference is reported as "other threads" and
    covers transport, telemetry-stat, buzzer, watchdog, and similar helpers.

    Percentages are relative to one CPU core, so the AltairFC process can
    exceed 100% when multiple threads are executing concurrently.
    """

    def __init__(self, config: ProfilingConfig) -> None:
        if config.interval_s <= 0:
            raise ValueError("profiling.interval_s must be greater than zero")
        if config.top_n <= 0:
            raise ValueError("profiling.top_n must be greater than zero")
        if config.minimum_cpu_pct < 0:
            raise ValueError("profiling.minimum_cpu_pct cannot be negative")

        self._config = config
        self._totals: dict[str, _TaskTotals] = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="task-cpu-profiler",
            daemon=True,
        )
        self._thread.start()
        logger.info(
            "Task CPU profiler started (interval=%.1fs, top_n=%d)",
            self._config.interval_s,
            self._config.top_n,
        )

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
        logger.info("Task CPU profiler stopped")

    def record_execution(self, task_name: str, cpu_s: float, wall_s: float) -> None:
        """Record one completed BaseTask.execute() call."""
        with self._lock:
            totals = self._totals.setdefault(task_name, _TaskTotals())
            totals.cpu_s += max(cpu_s, 0.0)
            totals.wall_s += max(wall_s, 0.0)
            totals.calls += 1
            totals.maximum_wall_s = max(totals.maximum_wall_s, wall_s)

    def _run(self) -> None:
        previous_wall_s = time.monotonic()
        previous_process_cpu_s = time.process_time()

        while not self._stop_event.wait(self._config.interval_s):
            with self._lock:
                now_wall_s = time.monotonic()
                now_process_cpu_s = time.process_time()
                totals = self._totals
                self._totals = {}

            report = self._build_report(
                interval_s=now_wall_s - previous_wall_s,
                process_cpu_s=now_process_cpu_s - previous_process_cpu_s,
                totals=totals,
            )
            previous_wall_s = now_wall_s
            previous_process_cpu_s = now_process_cpu_s
            self._log_report(report)

    def _build_report(
        self,
        interval_s: float,
        process_cpu_s: float,
        totals: dict[str, _TaskTotals],
    ) -> ProfileReport:
        if interval_s <= 0:
            return ProfileReport(0.0, 0.0, 0.0, 0.0, [])

        task_cpu_s = sum(item.cpu_s for item in totals.values())
        samples = [
            TaskCpuSample(
                name=name,
                cpu_pct=item.cpu_s / interval_s * 100.0,
                calls=item.calls,
                average_wall_ms=(
                    item.wall_s / item.calls * 1000.0 if item.calls else 0.0
                ),
                maximum_wall_ms=item.maximum_wall_s * 1000.0,
            )
            for name, item in totals.items()
        ]
        samples.sort(key=lambda item: item.cpu_pct, reverse=True)
        visible = [
            item
            for item in samples
            if item.cpu_pct >= self._config.minimum_cpu_pct
        ][: self._config.top_n]

        altairfc_cpu_pct = max(process_cpu_s, 0.0) / interval_s * 100.0
        task_cpu_pct = task_cpu_s / interval_s * 100.0
        return ProfileReport(
            interval_s=interval_s,
            altairfc_cpu_pct=altairfc_cpu_pct,
            task_cpu_pct=task_cpu_pct,
            other_threads_cpu_pct=max(altairfc_cpu_pct - task_cpu_pct, 0.0),
            tasks=visible,
        )

    @staticmethod
    def _log_report(report: ProfileReport) -> None:
        task_text = ", ".join(
            (
                f"{task.name}={task.cpu_pct:.1f}% "
                f"({task.calls} calls, avg={task.average_wall_ms:.2f}ms, "
                f"max={task.maximum_wall_ms:.2f}ms)"
            )
            for task in report.tasks
        )
        logger.info(
            "Task CPU profile: altairfc=%.1f%%, scheduled_tasks=%.1f%%, "
            "other_threads=%.1f%%; tasks: %s",
            report.altairfc_cpu_pct,
            report.task_cpu_pct,
            report.other_threads_cpu_pct,
            task_text or "none above threshold",
        )
