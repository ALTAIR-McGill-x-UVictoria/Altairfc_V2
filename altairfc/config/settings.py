from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from drivers.port_detect import find_lr900p_port


def _resolve_serial_port(cfg: dict[str, Any]) -> "SerialPortConfig":
    """
    Build a SerialPortConfig, resolving port="auto" by scanning for a CP210x device.
    Raises RuntimeError if auto-detect is requested but no device is found.
    """
    port = cfg.get("port", "")
    if port.lower() == "auto":
        detected = find_lr900p_port()
        if detected is None:
            raise RuntimeError(
                "Telemetry port set to 'auto' but no CP210x (LR-900p) device was detected. "
                "Check the USB connection or set the port explicitly in config/settings.toml."
            )
        port = detected
    return SerialPortConfig(port=port, baud=cfg["baud"])


@dataclass
class SerialPortConfig:
    port: str
    baud: int


@dataclass
class TaskConfig:
    name: str
    enabled: bool
    period_s: float
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class SystemConfig:
    mavlink: SerialPortConfig
    telemetry: SerialPortConfig
    vesc: SerialPortConfig
    tasks: dict[str, TaskConfig]
    log_level: str = "INFO"
    monitor_interval_s: float = 5.0

    @classmethod
    def from_toml(cls, path: Path) -> "SystemConfig":
        with open(path, "rb") as f:
            data = tomllib.load(f)

        mavlink = SerialPortConfig(**data["mavlink"])
        telemetry = _resolve_serial_port(data["telemetry"])
        vesc = SerialPortConfig(**data["vesc"])

        tasks: dict[str, TaskConfig] = {}
        for name, cfg in data.get("tasks", {}).items():
            tasks[name] = TaskConfig(
                name=name,
                enabled=cfg.get("enabled", False),
                period_s=cfg.get("period_s", 1.0),
                extra={k: v for k, v in cfg.items() if k not in ("enabled", "period_s")},
            )

        system = data.get("system", {})
        return cls(
            mavlink=mavlink,
            telemetry=telemetry,
            vesc=vesc,
            tasks=tasks,
            log_level=system.get("log_level", "INFO"),
            monitor_interval_s=system.get("monitor_interval_s", 5.0),
        )

    def get_task(self, name: str) -> TaskConfig | None:
        return self.tasks.get(name)
