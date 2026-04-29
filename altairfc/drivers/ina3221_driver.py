from __future__ import annotations

import ctypes
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

_SO_PATH = Path(__file__).parent / "libina3221_driver.so"


class INA3221Reading(ctypes.Structure):
    _fields_ = [
        ("voltage_v", ctypes.c_float * 3),
        ("current_a", ctypes.c_float * 3),
    ]


def _load_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_SO_PATH))
    lib.ina3221_open.restype  = ctypes.c_int
    lib.ina3221_open.argtypes = [ctypes.c_char_p]
    lib.ina3221_read.restype  = ctypes.c_int
    lib.ina3221_read.argtypes = [ctypes.c_int, ctypes.POINTER(INA3221Reading)]
    lib.ina3221_close.restype  = None
    lib.ina3221_close.argtypes = [ctypes.c_int]
    return lib


class INA3221Driver:
    """
    Thin Python wrapper around libina3221_driver.so.

    Channels (0-indexed):
        0 → 24 V rail
        1 → 12 V rail
        2 →  5 V rail
    """

    RAIL_NAMES = ("24V", "12V", "5V")

    def __init__(self, i2c_dev: str = "/dev/i2c-1") -> None:
        self._lib = _load_lib()
        self._fd = self._lib.ina3221_open(i2c_dev.encode())
        if self._fd < 0:
            raise OSError(
                f"ina3221_open failed on {i2c_dev} — "
                "check I2C bus, address 0x40, and manufacturer ID"
            )
        logger.info("INA3221Driver: opened %s (fd=%d)", i2c_dev, self._fd)

    def read(self) -> INA3221Reading | None:
        reading = INA3221Reading()
        ret = self._lib.ina3221_read(self._fd, ctypes.byref(reading))
        if ret == 0:
            return reading
        logger.warning("INA3221Driver: I2C read error")
        return None

    def close(self) -> None:
        if self._fd >= 0:
            self._lib.ina3221_close(self._fd)
            self._fd = -1
