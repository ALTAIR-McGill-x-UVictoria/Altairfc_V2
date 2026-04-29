from __future__ import annotations

import os
import re
import subprocess
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
    Payload size: 2 * 8 + 10 * 4 = 56 bytes

    Fields:
        time_unix            — wall-clock UNIX timestamp (float64, s)
        uptime_s             — time.monotonic() seconds since process start (float64, s)
        cpu_load_pct         — 1-minute load average scaled to percent of one core (float32, %)
        mem_used_pct         — RSS / MemTotal from /proc/meminfo (float32, %)
        tasks_running        — number of tasks registered in the scheduler (float32, count)
        pixhawk_connected    — 1.0 if MAVLink heartbeat received, 0.0 otherwise (float32, bool)
        vesc_connected       — 1.0 if VESC serial link is active, 0.0 otherwise (float32, bool)
        power_connected      — 1.0 if power daughterboard is active, 0.0 otherwise (float32, bool)
        photodiode_connected — 1.0 if photodiode daughterboard is active, 0.0 otherwise (float32, bool)
        pps_rms_us           — PPS RMS offset from chrony tracking (float32, µs); 0.0 if unavailable

    DataStore keys (written by TelemetryTask before packet iteration):
        "system.time_unix"
        "system.uptime_s"
        "system.cpu_load_pct"
        "system.mem_used_pct"
        "system.tasks_running"
        "system.pixhawk_connected"
        "system.vesc_connected"
        "system.power_connected"
        "system.photodiode_connected"
        "system.pps_rms_us"
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "time_unix":            "system.time_unix",
        "uptime_s":             "system.uptime_s",
        "cpu_load_pct":         "system.cpu_load_pct",
        "mem_used_pct":         "system.mem_used_pct",
        "tasks_running":        "system.tasks_running",
        "pixhawk_connected":    "system.pixhawk_connected",
        "vesc_connected":       "system.vesc_connected",
        "power_connected":      "system.power_connected",
        "photodiode_connected": "system.photodiode_connected",
        "pps_synced":           "system.pps_synced",
        "pps_rms_us":           "system.pps_rms_us",
    }

    time_unix:            float = field(default=0.0, metadata=FieldMeta("d", "UNIX wall-clock time",       "s").as_metadata())
    uptime_s:             float = field(default=0.0, metadata=FieldMeta("d", "Process uptime",             "s").as_metadata())
    cpu_load_pct:         float = field(default=0.0, metadata=FieldMeta("f", "1-min CPU load (1 core)",   "%").as_metadata())
    mem_used_pct:         float = field(default=0.0, metadata=FieldMeta("f", "Memory used",               "%").as_metadata())
    tasks_running:        float = field(default=0.0, metadata=FieldMeta("f", "Tasks running",             "count").as_metadata())
    pixhawk_connected:    float = field(default=0.0, metadata=FieldMeta("f", "Pixhawk link",              "bool").as_metadata())
    vesc_connected:       float = field(default=0.0, metadata=FieldMeta("f", "VESC link",                 "bool").as_metadata())
    power_connected:      float = field(default=0.0, metadata=FieldMeta("f", "Power board link",          "bool").as_metadata())
    photodiode_connected: float = field(default=0.0, metadata=FieldMeta("f", "Photodiode board link",     "bool").as_metadata())
    pps_synced:           float = field(default=0.0, metadata=FieldMeta("f", "PPS time sync",             "bool").as_metadata())
    pps_rms_us:           float = field(default=0.0, metadata=FieldMeta("f", "PPS RMS offset",            "us").as_metadata())


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


def _chronyc_tracking() -> str:
    """Run 'chronyc tracking' once and return stdout; empty string on failure."""
    try:
        return subprocess.run(
            ["chronyc", "tracking"],
            capture_output=True, text=True, timeout=1.0,
        ).stdout
    except Exception:
        return ""


def read_pps_stats() -> tuple[int, float]:
    """
    Parses 'chronyc tracking' output and returns (pps_synced, pps_rms_us).

    pps_synced: 1 if the current reference source name contains "PPS", 0 otherwise.
    pps_rms_us: RMS offset in µs; 0.0 if chrony is unavailable.
    """
    out = _chronyc_tracking()
    if not out:
        return 0, 0.0

    # Reference ID line: "Reference ID    : 50505300 (PPS)" or "GPS" etc.
    ref_m = re.search(r"Reference ID\s*:\s*\S+\s*\(([^)]+)\)", out)
    pps_synced = 1 if (ref_m and "PPS" in ref_m.group(1).upper()) else 0

    rms_us = 0.0
    rms_m = re.search(r"RMS offset\s*:\s*([\d.e+\-]+)\s*(\w+)", out)
    if rms_m:
        value, unit = float(rms_m.group(1)), rms_m.group(2)
        if unit == "us":
            rms_us = value
        elif unit == "ms":
            rms_us = value * 1e3
        elif unit == "ns":
            rms_us = value * 1e-3
        elif unit == "s":
            rms_us = value * 1e6

    return pps_synced, rms_us


def collect_system_stats(tasks_running: int = 0) -> dict[str, float]:
    """
    Returns a dict of system.* DataStore key → value for the current instant.
    Pass tasks_running from the scheduler so the heartbeat reflects live task count.
    """
    pps_synced, pps_rms_us = read_pps_stats()
    return {
        "system.time_unix":     time.time(),
        "system.uptime_s":      time.monotonic() - _BOOT_MONOTONIC,
        "system.cpu_load_pct":  read_cpu_load(),
        "system.mem_used_pct":  read_mem_used_pct(),
        "system.tasks_running": float(tasks_running),
        "system.pps_synced":    float(pps_synced),
        "system.pps_rms_us":    pps_rms_us,
        # system.pixhawk_connected and system.vesc_connected are written by
        # their respective tasks and must not be overwritten here.
    }
