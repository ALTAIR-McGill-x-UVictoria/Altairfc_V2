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
class ControllerConfig:
    kp: float
    ki: float
    kd: float
    max: float
    min: float

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
class FlightStageConfig:
    termination_altitude_m:       float = 25000.0
    burst_altitude_m:             float = 30000.0
    burst_altitude_uncertainty_m: float = 2000.0
    ascent_detect_window_s:       float = 30.0
    ascent_detect_gain_m:         float = 50.0
    apogee_fraction:              float = 0.95
    landing_fraction:             float = 0.05
    recovery_stationary_s:        float = 10.0
    termination_confirm_drop_m:   float = 100.0
    termination_confirm_window_s: float = 30.0


@dataclass
class SystemConfig:
    mavlink: SerialPortConfig
    telemetry: SerialPortConfig
    wheel_motor: SerialPortConfig
    momentum_motor: SerialPortConfig
    rw_controller: ControllerConfig
    mm_controller: ControllerConfig
    tasks: dict[str, TaskConfig]
    flight_stage: FlightStageConfig = field(default_factory=FlightStageConfig)
    log_level: str = "INFO"
    monitor_interval_s: float = 5.0

    @classmethod
    def from_toml(cls, path: Path) -> "SystemConfig":
        with open(path, "rb") as f:
            data = tomllib.load(f)

        mavlink = SerialPortConfig(**data["mavlink"])
        telemetry = _resolve_serial_port(data["telemetry"])
        wheel_motor = SerialPortConfig(**data["rw_esc"]) 
        momentum_motor = SerialPortConfig(**data["mm_esc"])
        rw_controller = ControllerConfig(**data["rw_controller"])
        mm_controller = ControllerConfig(**data["mm_controller"])

        tasks: dict[str, TaskConfig] = {}
        for name, cfg in data.get("tasks", {}).items():
            tasks[name] = TaskConfig(
                name=name,
                enabled=cfg.get("enabled", False),
                period_s=cfg.get("period_s", 1.0),
                extra={k: v for k, v in cfg.items() if k not in ("enabled", "period_s")},
            )

        fs_raw = data.get("flight_stage", {})
        flight_stage = FlightStageConfig(
            termination_altitude_m=fs_raw.get("termination_altitude_m", 25000.0),
            burst_altitude_m=fs_raw.get("burst_altitude_m", 30000.0),
            burst_altitude_uncertainty_m=fs_raw.get("burst_altitude_uncertainty_m", 2000.0),
            ascent_detect_window_s=fs_raw.get("ascent_detect_window_s", 30.0),
            ascent_detect_gain_m=fs_raw.get("ascent_detect_gain_m", 50.0),
            apogee_fraction=fs_raw.get("apogee_fraction", 0.95),
            landing_fraction=fs_raw.get("landing_fraction", 0.05),
            recovery_stationary_s=fs_raw.get("recovery_stationary_s", 10.0),
            termination_confirm_drop_m=fs_raw.get("termination_confirm_drop_m", 100.0),
            termination_confirm_window_s=fs_raw.get("termination_confirm_window_s", 30.0),
        )


        system = data.get("system", {})
        return cls(
            mavlink=mavlink,
            telemetry=telemetry,
            wheel_motor=wheel_motor,
            momentum_motor=momentum_motor,
            rw_controller=rw_controller,
            mm_controller=mm_controller,
            tasks=tasks,
            flight_stage=flight_stage,
            log_level=system.get("log_level", "INFO"),
            monitor_interval_s=system.get("monitor_interval_s", 5.0),
        )

    def get_task(self, name: str) -> TaskConfig | None:
        return self.tasks.get(name)
