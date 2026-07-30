from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from config.settings import ProfilingConfig

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ProcessCpu:
    pid: int
    command: str
    cpu_pct: float


@dataclass(frozen=True)
class _Snapshot:
    monotonic_s: float
    total_ticks: int
    busy_ticks: int
    processes: dict[int, tuple[int, str]]


class CpuProfiler:
    """
    Periodically reports the processes consuming the most CPU.

    Linux CPU counters are sampled from /proc, so this adds no third-party
    dependency. Process percentages are expressed relative to one CPU core:
    a multi-threaded process can therefore exceed 100%.
    """

    def __init__(
        self,
        config: ProfilingConfig,
        proc_root: Path = Path("/proc"),
    ) -> None:
        if config.interval_s <= 0:
            raise ValueError("profiling.interval_s must be greater than zero")
        if config.top_n <= 0:
            raise ValueError("profiling.top_n must be greater than zero")
        if config.minimum_cpu_pct < 0:
            raise ValueError("profiling.minimum_cpu_pct cannot be negative")

        self._config = config
        self._proc_root = proc_root
        try:
            self._clock_ticks = os.sysconf("SC_CLK_TCK")
        except (AttributeError, ValueError):
            # Only used with Linux /proc; this keeps construction testable on
            # development hosts where sysconf is not exposed.
            self._clock_ticks = 100
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        if not (self._proc_root / "stat").is_file():
            logger.warning(
                "CPU profiler requires Linux /proc; profiling is unavailable"
            )
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="cpu-profiler",
            daemon=True,
        )
        self._thread.start()
        logger.info(
            "CPU profiler started (interval=%.1fs, top_n=%d)",
            self._config.interval_s,
            self._config.top_n,
        )

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
        logger.info("CPU profiler stopped")

    def _run(self) -> None:
        previous = self._snapshot()
        while not self._stop_event.wait(self._config.interval_s):
            current = self._snapshot()
            system_cpu, processes = self._calculate_usage(previous, current)
            previous = current

            visible = [
                process
                for process in processes
                if process.cpu_pct >= self._config.minimum_cpu_pct
            ][: self._config.top_n]
            process_text = ", ".join(
                f"{process.command}[{process.pid}]={process.cpu_pct:.1f}%"
                for process in visible
            )
            logger.info(
                "CPU profile: system=%.1f%%; top processes: %s",
                system_cpu,
                process_text or "none above threshold",
            )

    def _snapshot(self) -> _Snapshot:
        total_ticks, busy_ticks = self._read_system_ticks()
        processes: dict[int, tuple[int, str]] = {}

        try:
            entries = list(self._proc_root.iterdir())
        except OSError:
            entries = []

        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                ticks, command = self._read_process(entry)
            except (OSError, ValueError, IndexError):
                # Processes may exit or become inaccessible during a scan.
                continue
            processes[int(entry.name)] = (ticks, command)

        return _Snapshot(
            monotonic_s=time.monotonic(),
            total_ticks=total_ticks,
            busy_ticks=busy_ticks,
            processes=processes,
        )

    def _read_system_ticks(self) -> tuple[int, int]:
        fields = (self._proc_root / "stat").read_text().splitlines()[0].split()
        if not fields or fields[0] != "cpu":
            raise ValueError("invalid /proc/stat CPU row")
        # guest and guest_nice (fields 9 and 10) are already included in user
        # and nice, so omit them rather than double-counting CPU time.
        counters = [int(value) for value in fields[1:9]]
        total = sum(counters)
        idle = counters[3] + (counters[4] if len(counters) > 4 else 0)
        return total, total - idle

    @staticmethod
    def _read_process(process_dir: Path) -> tuple[int, str]:
        stat = (process_dir / "stat").read_text()
        command_end = stat.rfind(")")
        if command_end < 0:
            raise ValueError("invalid process stat")

        command = stat[stat.find("(") + 1 : command_end]
        # Fields after the command begin with field 3 (state). utime and stime
        # are fields 14 and 15, hence indexes 11 and 12 in this slice.
        fields = stat[command_end + 2 :].split()
        ticks = int(fields[11]) + int(fields[12])
        return ticks, command

    def _calculate_usage(
        self,
        previous: _Snapshot,
        current: _Snapshot,
    ) -> tuple[float, list[ProcessCpu]]:
        elapsed_s = current.monotonic_s - previous.monotonic_s
        if elapsed_s <= 0:
            return 0.0, []

        total_delta = current.total_ticks - previous.total_ticks
        busy_delta = current.busy_ticks - previous.busy_ticks
        system_cpu = (
            max(0.0, min(100.0, busy_delta / total_delta * 100.0))
            if total_delta > 0
            else 0.0
        )

        usage: list[ProcessCpu] = []
        for pid, (ticks, command) in current.processes.items():
            old = previous.processes.get(pid)
            if old is None or old[1] != command:
                # The PID is new or was reused by a different command.
                continue
            tick_delta = ticks - old[0]
            if tick_delta < 0:
                continue
            usage.append(
                ProcessCpu(
                    pid=pid,
                    command=command,
                    cpu_pct=tick_delta / self._clock_ticks / elapsed_s * 100.0,
                )
            )

        usage.sort(key=lambda process: process.cpu_pct, reverse=True)
        return system_cpu, usage
