from __future__ import annotations

import logging
import queue
import threading

import serial

logger = logging.getLogger(__name__)

_SENTINEL = object()


class SerialTransport:
    """
    Non-blocking serial writer for the LR-900p telemetry radio.

    send() enqueues a binary frame and returns immediately.
    A background writer thread dequeues frames and writes them to the serial port.
    This prevents TelemetryTask from blocking on a slow serial port.
    """

    def __init__(self, port: str, baud: int, write_queue_maxsize: int = 64) -> None:
        self.port = port
        self.baud = baud
        self._queue: queue.Queue[bytes | object] = queue.Queue(maxsize=write_queue_maxsize)
        self._serial: serial.Serial | None = None
        self._writer_thread: threading.Thread | None = None

    def open(self) -> None:
        self._serial = serial.Serial(self.port, self.baud, timeout=1.0)
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="telemetry-transport-writer",
            daemon=True,
        )
        self._writer_thread.start()
        logger.info("SerialTransport opened %s @ %d baud", self.port, self.baud)

    def close(self) -> None:
        self._queue.put(_SENTINEL)
        if self._writer_thread is not None:
            self._writer_thread.join(timeout=3.0)
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        logger.info("SerialTransport closed")

    def send(self, frame: bytes) -> None:
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            logger.warning("Telemetry write queue full — dropping frame (%d bytes)", len(frame))

    def _writer_loop(self) -> None:
        assert self._serial is not None
        while True:
            item = self._queue.get()
            if item is _SENTINEL:
                break
            try:
                self._serial.write(item)
            except serial.SerialException:
                logger.exception("SerialTransport write error")
