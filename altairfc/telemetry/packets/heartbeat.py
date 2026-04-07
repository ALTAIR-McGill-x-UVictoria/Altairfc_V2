from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x00)
@dataclass
class HeartbeatPacket:
    """
    System heartbeat — sent every telemetry cycle to confirm the flight computer
    is alive and to carry basic system health metrics.

    Packet ID: 0x00
    Payload size: 2 * 8 + 3 * 4 = 28 bytes

    Fields:
        time_unix      — wall-clock UNIX timestamp (float64, s)
        uptime_s       — time.monotonic() seconds since process start (float64, s)
        cpu_load_pct   — 1-minute load average scaled to percent of one core (float32, %)
        mem_used_pct   — RSS / MemTotal from /proc/meminfo (float32, %)
        tasks_running  — number of tasks registered in the scheduler (float32, count)

    DataStore keys (written by TelemetryTask before packet iteration):
        "system.time_unix"
        "system.uptime_s"
        "system.cpu_load_pct"
        "system.mem_used_pct"
        "system.tasks_running"
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "time_unix":     "system.time_unix",
        "uptime_s":      "system.uptime_s",
        "cpu_load_pct":  "system.cpu_load_pct",
        "mem_used_pct":  "system.mem_used_pct",
        "tasks_running": "system.tasks_running",
    }

    time_unix:     float = field(default=0.0, metadata=FieldMeta("d", "UNIX wall-clock time",    "s").as_metadata())
    uptime_s:      float = field(default=0.0, metadata=FieldMeta("d", "Process uptime",          "s").as_metadata())
    cpu_load_pct:  float = field(default=0.0, metadata=FieldMeta("f", "1-min CPU load (1 core)", "%").as_metadata())
    mem_used_pct:  float = field(default=0.0, metadata=FieldMeta("f", "Memory used",             "%").as_metadata())
    tasks_running: float = field(default=0.0, metadata=FieldMeta("f", "Tasks running",           "count").as_metadata())


# ---------------------------------------------------------------------------
# Helpers — called by TelemetryTask to populate system.* DataStore keys
# ---------------------------------------------------------------------------

_BOOT_MONOTONIC: float = time.monotonic()


def read_cpu_load() -> float:
    """
    Returns the 1-minute load average as a percentage of one CPU core.
    Falls back to 0.0 on platforms without os.getloadavg() (e.g. Windows).
    """
    try:
        load1, _, _ = os.getloadavg()
        return float(load1) * 100.0
    except (AttributeError, OSError):
        return 0.0


def read_mem_used_pct() -> float:
    """
    Reads MemTotal and MemAvailable from /proc/meminfo and returns
    used-memory percentage.  Falls back to 0.0 if the file is unavailable.
    """
    try:
        mem: dict[str, int] = {}
        with open("/proc/meminfo", "r") as fh:
            for line in fh:
                if line.startswith(("MemTotal:", "MemAvailable:")):
                    parts = line.split()
                    mem[parts[0].rstrip(":")] = int(parts[1])
                if len(mem) == 2:
                    break
        total = mem.get("MemTotal", 0)
        available = mem.get("MemAvailable", 0)
        if total == 0:
            return 0.0
        return (1.0 - available / total) * 100.0
    except OSError:
        return 0.0


def collect_system_stats(tasks_running: int = 0) -> dict[str, float]:
    """
    Returns a dict of system.* DataStore key → value for the current instant.
    Pass tasks_running from the scheduler so the heartbeat reflects live task count.
    """
    return {
        "system.time_unix":     time.time(),
        "system.uptime_s":      time.monotonic() - _BOOT_MONOTONIC,
        "system.cpu_load_pct":  read_cpu_load(),
        "system.mem_used_pct":  read_mem_used_pct(),
        "system.tasks_running": float(tasks_running),
    }
