from __future__ import annotations

import logging
from dataclasses import dataclass

from serial.tools import list_ports
from serial.tools.list_ports_common import ListPortInfo

logger = logging.getLogger(__name__)

# Silicon Labs CP2102 / CP210x USB-to-UART bridge
# VID 0x10C4 is Silicon Labs; PID 0xEA60 covers the full CP210x family.
_CP210X_VID = 0x10C4
_CP210X_PID = 0xEA60


@dataclass
class DetectedPort:
    device: str          # e.g. "/dev/ttyUSB0" or "COM3"
    description: str
    hwid: str
    vid: int
    pid: int


def find_cp210x_ports() -> list[DetectedPort]:
    """Return all connected ports whose USB VID/PID match the CP210x family."""
    results: list[DetectedPort] = []
    for port in list_ports.comports():
        if port.vid == _CP210X_VID and port.pid == _CP210X_PID:
            results.append(
                DetectedPort(
                    device=port.device,
                    description=port.description or "",
                    hwid=port.hwid or "",
                    vid=port.vid,
                    pid=port.pid,
                )
            )
    return results


def find_lr900p_port() -> str | None:
    """
    Return the device path of the first CP210x port found (most likely the LR-900p).

    Returns None if no matching port is detected.
    Logs a warning if multiple CP210x ports are found (ambiguous — pick manually).
    """
    ports = find_cp210x_ports()

    if not ports:
        logger.warning(
            "LR-900p auto-detect: no CP210x device found "
            "(VID=0x%04X PID=0x%04X). Check USB connection and driver.",
            _CP210X_VID,
            _CP210X_PID,
        )
        return None

    if len(ports) > 1:
        devices = [p.device for p in ports]
        logger.warning(
            "LR-900p auto-detect: multiple CP210x devices found %s — "
            "using %s. Set [telemetry] port explicitly in settings.toml to suppress this.",
            devices,
            ports[0].device,
        )

    logger.info("LR-900p auto-detect: found %s (%s)", ports[0].device, ports[0].description)
    return ports[0].device
