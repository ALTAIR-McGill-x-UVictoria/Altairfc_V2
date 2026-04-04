from __future__ import annotations

import binascii
import dataclasses
import logging
import struct
import time

from telemetry.registry import packet_registry

logger = logging.getLogger(__name__)

SYNC_BYTE = 0xAA
# Header layout (little-endian): sync(B) pkt_id(B) seq(B) timestamp(d) length(H)
# Total: 1 + 1 + 1 + 8 + 2 = 13 bytes
_HEADER_STRUCT = struct.Struct("<BBBdH")
_CRC_STRUCT = struct.Struct("<H")

HEADER_SIZE = _HEADER_STRUCT.size   # 13
CRC_SIZE = _CRC_STRUCT.size         # 2
MIN_FRAME_SIZE = HEADER_SIZE + CRC_SIZE


class PacketSerializer:
    """
    Stateless serializer. Converts packet dataclass instances to/from wire bytes.

    Wire frame layout:
        [SYNC:1][PKT_ID:1][SEQ:1][TIMESTAMP:8 float64 LE][LEN:2 uint16 LE][DATA:N][CRC16:2 LE]

    CRC-16/CCITT (via binascii.crc_hqx) computed over:
        PKT_ID + SEQ + TIMESTAMP_bytes + LEN_bytes + DATA
    (i.e. everything between SYNC and CRC, exclusive of both.)
    """

    def pack(self, packet: object, seq: int = 0) -> bytes:
        pkt_type = type(packet)
        packet_id = packet_registry.get_id(pkt_type)
        pkt_struct = packet_registry.get_struct(pkt_type)

        if packet_id is None or pkt_struct is None:
            raise ValueError(f"Packet type {pkt_type.__name__} is not registered")

        field_values = [getattr(packet, f.name) for f in dataclasses.fields(packet)]
        payload = pkt_struct.pack(*field_values)

        timestamp = time.monotonic()
        header_body = _HEADER_STRUCT.pack(
            SYNC_BYTE,
            packet_id & 0xFF,
            seq & 0xFF,
            timestamp,
            len(payload),
        )

        # CRC covers everything after SYNC through end of payload
        crc_data = header_body[1:] + payload
        crc = binascii.crc_hqx(crc_data, 0xFFFF)

        return header_body + payload + _CRC_STRUCT.pack(crc)

    def unpack(self, raw: bytes) -> tuple[object, float] | None:
        if len(raw) < MIN_FRAME_SIZE:
            logger.warning("Frame too short: %d bytes", len(raw))
            return None

        sync, packet_id, seq, timestamp, length = _HEADER_STRUCT.unpack_from(raw, 0)

        if sync != SYNC_BYTE:
            logger.warning("Bad sync byte: 0x%02X", sync)
            return None

        expected_len = HEADER_SIZE + length + CRC_SIZE
        if len(raw) < expected_len:
            logger.warning(
                "Frame truncated: expected %d bytes, got %d", expected_len, len(raw)
            )
            return None

        payload = raw[HEADER_SIZE : HEADER_SIZE + length]
        received_crc = _CRC_STRUCT.unpack_from(raw, HEADER_SIZE + length)[0]

        crc_data = raw[1 : HEADER_SIZE + length]
        computed_crc = binascii.crc_hqx(crc_data, 0xFFFF)

        if received_crc != computed_crc:
            logger.warning(
                "CRC mismatch: received 0x%04X, computed 0x%04X", received_crc, computed_crc
            )
            return None

        result = packet_registry.get_by_id(packet_id)
        if result is None:
            logger.warning("Unknown packet ID: 0x%02X", packet_id)
            return None

        pkt_class, pkt_struct = result
        if len(payload) != pkt_struct.size:
            logger.warning(
                "Payload size mismatch for 0x%02X: expected %d, got %d",
                packet_id,
                pkt_struct.size,
                len(payload),
            )
            return None

        values = pkt_struct.unpack(payload)
        fields = dataclasses.fields(pkt_class)
        packet = pkt_class(**{f.name: v for f, v in zip(fields, values)})
        return packet, timestamp
