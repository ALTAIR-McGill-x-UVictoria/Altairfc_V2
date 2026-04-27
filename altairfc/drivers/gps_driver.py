from __future__ import annotations

import ctypes
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

_SO_PATH = Path(__file__).parent / "libgps_driver.so"


class GpsFix(ctypes.Structure):
    _fields_ = [
        ("lat",         ctypes.c_double),
        ("lon",         ctypes.c_double),
        ("alt_msl",     ctypes.c_double),
        ("speed_ms",    ctypes.c_float),
        ("heading_deg", ctypes.c_float),
        ("hdop",        ctypes.c_float),
        ("fix_type",    ctypes.c_uint8),
        ("num_sv",      ctypes.c_uint8),
        ("valid",       ctypes.c_uint8),
        ("_pad",        ctypes.c_uint8),
        ("year",        ctypes.c_uint16),
        ("month",       ctypes.c_uint8),
        ("day",         ctypes.c_uint8),
        ("hour",        ctypes.c_uint8),
        ("min",         ctypes.c_uint8),
        ("sec",         ctypes.c_uint8),
        ("time_valid",  ctypes.c_uint8),
    ]


def _load_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_SO_PATH))
    lib.gps_open.restype  = ctypes.c_int
    lib.gps_open.argtypes = [ctypes.c_char_p]
    lib.gps_read.restype  = ctypes.c_int
    lib.gps_read.argtypes = [ctypes.c_int, ctypes.POINTER(GpsFix)]
    lib.gps_close.restype  = None
    lib.gps_close.argtypes = [ctypes.c_int]
    return lib


class GpsDriver:
    def __init__(self, i2c_dev: str = "/dev/i2c-1") -> None:
        self._lib = _load_lib()
        self._fd = self._lib.gps_open(i2c_dev.encode())
        if self._fd < 0:
            raise OSError(f"gps_open failed on {i2c_dev} — check I2C bus and address 0x42")
        logger.info("GpsDriver: opened %s (fd=%d)", i2c_dev, self._fd)

    def read(self) -> GpsFix | None:
        fix = GpsFix()
        ret = self._lib.gps_read(self._fd, ctypes.byref(fix))
        if ret == 0:
            return fix
        if ret == -1:
            logger.warning("GpsDriver: I2C error during read")
        return None

    def close(self) -> None:
        if self._fd >= 0:
            self._lib.gps_close(self._fd)
            self._fd = -1
