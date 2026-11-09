from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass


SERGEANT_VALID = 0x01
SOLDIER_VALID = 0x02


@dataclass(frozen=True, slots=True)
class PhotodiodeSample:
    """One time-aligned pair of raw 24-bit photodiode ADC conversions."""

    sequence: int
    time_unix_us: int
    monotonic_ns: int
    sergeant_code: int
    soldier_code: int
    valid_flags: int = SERGEANT_VALID | SOLDIER_VALID


class PhotodiodeSampleBuffer:
    """Thread-safe FIFO shared by photodiode acquisition and telemetry.

    The producer appends only after a sample has been written to the local log.
    Telemetry peeks a batch, enqueues its serialized frame, then discards the
    same number of samples. No samples are intentionally dropped here.
    """

    def __init__(self) -> None:
        self._samples: deque[PhotodiodeSample] = deque()
        self._lock = threading.Lock()

    def append(self, sample: PhotodiodeSample) -> None:
        with self._lock:
            self._samples.append(sample)

    def peek_batch(self, max_samples: int) -> tuple[PhotodiodeSample, ...]:
        if max_samples <= 0:
            raise ValueError("max_samples must be greater than zero")
        with self._lock:
            count = min(max_samples, len(self._samples))
            return tuple(self._samples[i] for i in range(count))

    def discard(self, count: int) -> None:
        if count < 0:
            raise ValueError("count cannot be negative")
        with self._lock:
            if count > len(self._samples):
                raise ValueError("cannot discard more samples than are buffered")
            for _ in range(count):
                self._samples.popleft()

    def __len__(self) -> int:
        with self._lock:
            return len(self._samples)
