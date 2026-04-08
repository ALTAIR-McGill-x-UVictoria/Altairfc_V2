from __future__ import annotations

import logging
import queue
import threading
import time

import serial

logger = logging.getLogger(__name__)

_SENTINEL = object()

# Serial framing overhead: 1 start + 8 data + 1 stop = 10 bits per byte
_BITS_PER_BYTE = 10


class SerialTransport:
    """
    Non-blocking serial writer for the LR-900p telemetry radio.

    send() enqueues a binary frame and returns immediately.
    A background writer thread dequeues frames and writes them to the serial port.

    Two strategies prevent queue saturation when the radio can't keep up:

    1. Overwrite-on-full: if the queue is full, the oldest frame is discarded
       to make room for the newest. Stale telemetry is worthless — fresh data
       is always preferred over old data that hasn't been sent yet.

    2. Baud-paced writes: after each serial.write(), the writer thread sleeps
       for the theoretical transmission time of that frame at the configured
       baud rate. This prevents the writer from outrunning the radio's TX
       buffer and blocking on subsequent writes.
    """

    def __init__(self, port: str, baud: int, write_queue_maxsize: int = 64) -> None:
        self.port = port
        self.baud = baud
        self._queue: queue.Queue[bytes | object] = queue.Queue(maxsize=write_queue_maxsize)
        self._serial: serial.Serial | None = None
        self._writer_thread: threading.Thread | None = None
        self._write_lock = threading.Lock()
        # Seconds per byte at this baud rate
        self._secs_per_byte = _BITS_PER_BYTE / baud

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

    def read_available(self) -> bytes:
        """Read all currently available bytes (non-blocking). Safe to call from a separate thread."""
        if self._serial and self._serial.is_open and self._serial.in_waiting:
            return self._serial.read(self._serial.in_waiting)
        return b""

    def send_priority(self, frame: bytes) -> None:
        """Write a frame directly to the serial port, bypassing the write queue.

        Used for ACK frames that must not wait behind queued telemetry frames.
        Acquires a lock shared with the writer thread to prevent interleaved writes.
        Safe to call from any thread.
        """
        if self._serial is None or not self._serial.is_open:
            logger.warning("send_priority: serial not open, dropping %d-byte frame", len(frame))
            return
        with self._write_lock:
            try:
                self._serial.write(frame)
                time.sleep(len(frame) * self._secs_per_byte)
            except serial.SerialException:
                logger.exception("send_priority write error")

    def send(self, frame: bytes) -> None:
        """Enqueue a frame. If the queue is full, drop the oldest to make room."""
        while True:
            try:
                self._queue.put_nowait(frame)
                return
            except queue.Full:
                # Discard the oldest frame so the newest (freshest) data gets through
                try:
                    dropped = self._queue.get_nowait()
                    if isinstance(dropped, bytes):
                        logger.debug(
                            "Queue full — dropped oldest frame (%d bytes) to enqueue newest (%d bytes)",
                            len(dropped), len(frame),
                        )
                except queue.Empty:
                    pass  # race: another thread drained it; retry put

    def _writer_loop(self) -> None:
        assert self._serial is not None
        logger.debug("Telemetry writer loop started")
        while True:
            item = self._queue.get()
            if item is _SENTINEL:
                break
            if isinstance(item, bytes):
                logger.debug("Writing telemetry frame (%d bytes)", len(item))
                try:
                    with self._write_lock:
                        self._serial.write(item)
                        # Pace output to baud rate so we don't flood the radio's TX buffer
                        time.sleep(len(item) * self._secs_per_byte)
                    logger.debug("Wrote telemetry frame successfully")
                except serial.SerialException:
                    logger.exception("SerialTransport write error")
            else:
                logger.warning("Unexpected item in queue: %s", type(item))
