from __future__ import annotations

import ctypes
import logging
import os
from pathlib import Path

logger = logging.getLogger(__name__)

_SO_PATH = Path(__file__).parent / "libgpio_hold_driver.so"


def _load_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_SO_PATH), use_errno=True)

    lib.gpio_hold_open.restype = ctypes.c_void_p
    lib.gpio_hold_open.argtypes = [ctypes.c_char_p, ctypes.c_uint]

    lib.gpio_hold_close.restype = None
    lib.gpio_hold_close.argtypes = [ctypes.c_void_p]
    return lib


class GpioHold:
    """Claim a GPIO line as an output and drive it high for as long as this
    object is open.

    Used to keep chip-select lines belonging to other SPI devices that
    share this bus (but aren't otherwise touched by this process)
    deselected, on boards wired with no pull-up of their own.
    """

    def __init__(self, gpiochip: str, offset: int) -> None:
        self._lib = _load_lib()
        ctypes.set_errno(0)
        self._handle = self._lib.gpio_hold_open(gpiochip.encode(), offset)
        if not self._handle:
            err = ctypes.get_errno()
            reason = os.strerror(err) if err else "unknown error"
            raise OSError(
                err,
                f"gpio_hold_open failed on {gpiochip}:{offset} ({reason}) — "
                "line may already be requested by another process/overlay, "
                "the chip name may be wrong, or the offset may be out of range",
            )
        logger.info("GpioHold: holding %s:%d high", gpiochip, offset)

    def close(self) -> None:
        if self._handle:
            self._lib.gpio_hold_close(self._handle)
            self._handle = None

    def __enter__(self) -> GpioHold:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
