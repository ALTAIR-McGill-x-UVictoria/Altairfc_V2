from __future__ import annotations

import logging
import queue
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drivers.buzzer import Note

logger = logging.getLogger(__name__)

_SENTINEL = object()


class BuzzerPlayer:
    """
    Non-blocking buzzer player.

    Plays tunes on a dedicated daemon thread so callers never block.
    A new play() call replaces any currently queued tune (the running
    note finishes, then the new tune starts immediately).

    Usage:
        player = BuzzerPlayer()
        player.start()
        player.play(TUNE_PENDING)
        ...
        player.stop()
    """

    def __init__(self) -> None:
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._loop, name="buzzer", daemon=True)
        self._pi = None
        self._pin: int = 0
        self._gpio_ok = False

    def start(self) -> None:
        try:
            import pigpio
            from drivers.buzzer import GPIO_PIN
            self._pi = pigpio.pi()
            if not self._pi.connected:
                raise RuntimeError("pigpio daemon not running — start with: sudo pigpiod")
            self._pin = GPIO_PIN
            self._pi.set_mode(self._pin, pigpio.OUTPUT)
            self._pi.write(self._pin, 0)
            self._gpio_ok = True
        except Exception as e:
            logger.warning("BuzzerPlayer: GPIO unavailable (%s) — tunes will be silent", e)
        self._thread.start()

    def stop(self) -> None:
        try:
            self._queue.put_nowait(_SENTINEL)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._queue.put_nowait(_SENTINEL)
        self._thread.join(timeout=3.0)
        if self._gpio_ok and self._pi is not None:
            self._pi.set_PWM_dutycycle(self._pin, 0)
            self._pi.stop()

    def play(self, tune: list[Note]) -> None:
        """Queue a tune, discarding any previously queued (but not yet started) tune."""
        try:
            self._queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait(tune)
        except queue.Full:
            pass

    def _loop(self) -> None:
        while True:
            item = self._queue.get()
            if item is _SENTINEL:
                break
            self._play_blocking(item)

    def _play_blocking(self, tune: list[Note]) -> None:
        for freq, duration in tune:
            if self._gpio_ok and self._pi is not None:
                if freq > 0:
                    self._pi.set_PWM_frequency(self._pin, freq)
                    self._pi.set_PWM_dutycycle(self._pin, 128)  # 50% of 0-255
                else:
                    self._pi.set_PWM_dutycycle(self._pin, 0)
            time.sleep(duration)
        if self._gpio_ok and self._pi is not None:
            self._pi.set_PWM_dutycycle(self._pin, 0)
