from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import ClassVar

from core.photodiode_stream import PhotodiodeSample


PHOTODIODE_BATCH_PACKET_ID = 0x0D
PHOTODIODE_BATCH_VERSION = 1
PHOTODIODE_BATCH_SIZE = 50
PHOTODIODE_SAMPLE_PERIOD_US = 10_000

_BATCH_HEADER = struct.Struct("<BBHIQH")
_SAMPLE_SIZE = 7  # validity byte + two signed 24-bit ADC codes
_INT24_MIN = -(1 << 23)
_INT24_MAX = (1 << 23) - 1


def _pack_int24(value: int) -> bytes:
    if not _INT24_MIN <= value <= _INT24_MAX:
        raise ValueError(f"ADC code {value} does not fit in signed 24 bits")
    return (value & 0xFFFFFF).to_bytes(3, "little")


def _unpack_int24(raw: bytes) -> int:
    value = int.from_bytes(raw, "little")
    return value - (1 << 24) if value & 0x800000 else value


@dataclass(frozen=True, slots=True)
class PhotodiodeBatchPacket:
    """Variable-length batch carried by the standard telemetry frame."""

    PACKET_ID: ClassVar[int] = PHOTODIODE_BATCH_PACKET_ID

    samples: tuple[PhotodiodeSample, ...]
    sample_period_us: int = PHOTODIODE_SAMPLE_PERIOD_US
    flags: int = 0

    def pack_payload(self) -> bytes:
        count = len(self.samples)
        if not 1 <= count <= 255:
            raise ValueError("a photodiode batch must contain 1-255 samples")
        if not 0 <= self.sample_period_us <= 0xFFFF:
            raise ValueError("sample_period_us must fit in uint16")

        first = self.samples[0]
        payload = bytearray(
            _BATCH_HEADER.pack(
                PHOTODIODE_BATCH_VERSION,
                count,
                self.flags & 0xFFFF,
                first.sequence & 0xFFFFFFFF,
                first.time_unix_us & 0xFFFFFFFFFFFFFFFF,
                self.sample_period_us,
            )
        )
        for sample in self.samples:
            payload.append(sample.valid_flags & 0xFF)
            payload.extend(_pack_int24(sample.sergeant_code))
            payload.extend(_pack_int24(sample.soldier_code))
        return bytes(payload)

    @classmethod
    def unpack_payload(cls, payload: bytes) -> PhotodiodeBatchPacket:
        if len(payload) < _BATCH_HEADER.size:
            raise ValueError("photodiode batch payload is truncated")

        version, count, flags, first_seq, first_time_us, period_us = (
            _BATCH_HEADER.unpack_from(payload)
        )
        if version != PHOTODIODE_BATCH_VERSION:
            raise ValueError(f"unsupported photodiode batch version {version}")
        expected_size = _BATCH_HEADER.size + count * _SAMPLE_SIZE
        if count == 0 or len(payload) != expected_size:
            raise ValueError(
                f"invalid photodiode batch size: count={count}, "
                f"expected={expected_size}, actual={len(payload)}"
            )

        samples: list[PhotodiodeSample] = []
        offset = _BATCH_HEADER.size
        for index in range(count):
            valid_flags = payload[offset]
            sergeant_code = _unpack_int24(payload[offset + 1 : offset + 4])
            soldier_code = _unpack_int24(payload[offset + 4 : offset + 7])
            samples.append(
                PhotodiodeSample(
                    sequence=(first_seq + index) & 0xFFFFFFFF,
                    time_unix_us=first_time_us + index * period_us,
                    monotonic_ns=0,
                    sergeant_code=sergeant_code,
                    soldier_code=soldier_code,
                    valid_flags=valid_flags,
                )
            )
            offset += _SAMPLE_SIZE

        return cls(
            samples=tuple(samples),
            sample_period_us=period_us,
            flags=flags,
        )
