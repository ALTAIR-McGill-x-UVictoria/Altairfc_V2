from __future__ import annotations

import logging
import time
from drivers.vesc_interface import VESCObject

logger = logging.getLogger(__name__)

_STALE_WARN_S = 0.5           # warn when no telemetry reply has arrived for this long
_STALE_WARN_INTERVAL_S = 1.0  # rate limit on that warning

class MMDriver:
    def __init__(self, port: str) -> None:
        self.port_name = port
        self.motor: VESCObject | None = None
        self.connected = False
        self._last_rx = 0.0
        self._last_stale_warn = 0.0

    def connect(self) -> bool:
        try:
            motor = VESCObject(self.port_name)
            data = motor.get_data(timeout=0.3)
            if data is None:
                motor.port.close()
                raise TimeoutError("no data received from MM VESC")
            
            self.motor = motor
            self.connected = True
            self._last_rx = time.monotonic()
            return True

        except Exception as e:
            self.motor = None
            self.connected = False
            logger.error("MMDriver: VESC not connected on %s: %s", self.port_name, e)
            return False

    def read(self):
        if self.motor is None:
            self.connected = False
            return None
        try:
            data = self.motor.poll_data()  # reply to the previous cycle's request
            self.motor.request_data()      # reply is picked up next cycle
        except Exception as e:
            logger.error("MM data read failed: %s", e)
            self.connected = False
            return None
        now = time.monotonic()
        if data is not None:
            self._last_rx = now
            self.connected = True
        elif now - self._last_rx > _STALE_WARN_S and now - self._last_stale_warn > _STALE_WARN_INTERVAL_S:
            logger.warning("MM telemetry stale (%.1fs)", now - self._last_rx)
            self._last_stale_warn = now
        return data

    def set_rpm(self, rpm: int) -> None:
        limited_rpm = int(min(max(rpm, 0), 3000))
        if self.motor is not None:
            self.motor.set_rpm(limited_rpm)
        
    def set_current(self, current: int) -> None:
        if self.motor is not None:
            self.motor.set_current(current)

    def set_brake_current(self, current: int) -> None:
        if self.motor is not None:
            self.motor.set_brake_current(current)

    def stop(self) -> None:
        if self.motor is not None:
            self.motor.set_current(0)

    def close(self) -> None:
        self.stop()
        if self.motor is not None:
            self.motor.port.close()
        self.motor = None
        self.connected = False
