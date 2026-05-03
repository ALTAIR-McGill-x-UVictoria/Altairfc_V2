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
        self._pwm = None
        self._gpio_ok = False

    def start(self) -> None:
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            from drivers.buzzer import GPIO_PIN
            GPIO.setup(GPIO_PIN, GPIO.OUT)
            self._pwm = GPIO.PWM(GPIO_PIN, 440)
            self._pwm.start(0)
            self._gpio_ok = True
        except Exception as e:
            logger.warning("BuzzerPlayer: GPIO unavailable (%s) — tunes will be silent", e)
        self._thread.start()

    def stop(self) -> None:
        try:
            self._queue.put_nowait(_SENTINEL)
        except queue.Full:
            # Drain then re-insert sentinel so the loop exits
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._queue.put_nowait(_SENTINEL)
        self._thread.join(timeout=3.0)
        if self._gpio_ok and self._pwm is not None:
            self._pwm.stop()
            import RPi.GPIO as GPIO
            GPIO.cleanup()

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
            if self._gpio_ok and self._pwm is not None:
                if freq > 0:
                    self._pwm.ChangeFrequency(freq)
                    self._pwm.ChangeDutyCycle(50)
                else:
                    self._pwm.ChangeDutyCycle(0)
            time.sleep(duration)
        if self._gpio_ok and self._pwm is not None:
            self._pwm.ChangeDutyCycle(0)
