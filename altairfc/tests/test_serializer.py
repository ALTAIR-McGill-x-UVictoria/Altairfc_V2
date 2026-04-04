from __future__ import annotations

import math

import pytest

# Import packets to populate the registry before tests run
import telemetry.packets.attitude     # noqa: F401
import telemetry.packets.power        # noqa: F401
import telemetry.packets.vesc         # noqa: F401
import telemetry.packets.photodiode   # noqa: F401

from telemetry.packets.attitude import AttitudePacket
from telemetry.packets.power import PowerPacket
from telemetry.serializer import PacketSerializer, SYNC_BYTE, HEADER_SIZE, CRC_SIZE


@pytest.fixture
def serializer() -> PacketSerializer:
    return PacketSerializer()


def test_attitude_round_trip(serializer: PacketSerializer):
    original = AttitudePacket(
        roll=0.1, pitch=-0.2, yaw=3.14,
        rollspeed=0.01, pitchspeed=-0.01, yawspeed=0.0,
    )
    frame = serializer.pack(original, seq=1)
    result = serializer.unpack(frame)
    assert result is not None
    packet, _ts = result
    assert isinstance(packet, AttitudePacket)
    assert math.isclose(packet.roll,       original.roll,       rel_tol=1e-5)
    assert math.isclose(packet.pitch,      original.pitch,      rel_tol=1e-5)
    assert math.isclose(packet.yaw,        original.yaw,        rel_tol=1e-5)
    assert math.isclose(packet.rollspeed,  original.rollspeed,  rel_tol=1e-5)
    assert math.isclose(packet.pitchspeed, original.pitchspeed, rel_tol=1e-5)
    assert math.isclose(packet.yawspeed,   original.yawspeed,   rel_tol=1e-5)


def test_power_round_trip(serializer: PacketSerializer):
    original = PowerPacket(voltage_bus=12.4, current_total=3.2, temperature=45.0)
    frame = serializer.pack(original)
    result = serializer.unpack(frame)
    assert result is not None
    packet, _ = result
    assert isinstance(packet, PowerPacket)
    assert math.isclose(packet.voltage_bus, original.voltage_bus, rel_tol=1e-5)


def test_frame_starts_with_sync(serializer: PacketSerializer):
    frame = serializer.pack(AttitudePacket())
    assert frame[0] == SYNC_BYTE


def test_frame_minimum_size(serializer: PacketSerializer):
    frame = serializer.pack(AttitudePacket())
    # 13 header + 24 payload + 2 CRC = 39 bytes
    assert len(frame) == HEADER_SIZE + 24 + CRC_SIZE


def test_crc_corruption_detected(serializer: PacketSerializer):
    frame = bytearray(serializer.pack(AttitudePacket()))
    # Flip a bit in the payload
    frame[HEADER_SIZE] ^= 0xFF
    result = serializer.unpack(bytes(frame))
    assert result is None


def test_bad_sync_byte(serializer: PacketSerializer):
    frame = bytearray(serializer.pack(AttitudePacket()))
    frame[0] = 0x00
    result = serializer.unpack(bytes(frame))
    assert result is None


def test_truncated_frame(serializer: PacketSerializer):
    frame = serializer.pack(AttitudePacket())
    result = serializer.unpack(frame[:5])
    assert result is None


def test_seq_counter_wraps(serializer: PacketSerializer):
    pkt = AttitudePacket()
    frame_255 = serializer.pack(pkt, seq=255)
    frame_0 = serializer.pack(pkt, seq=0)
    # Both must unpack successfully
    assert serializer.unpack(frame_255) is not None
    assert serializer.unpack(frame_0) is not None


def test_timestamp_is_positive(serializer: PacketSerializer):
    frame = serializer.pack(AttitudePacket())
    result = serializer.unpack(frame)
    assert result is not None
    _, ts = result
    assert ts > 0.0
