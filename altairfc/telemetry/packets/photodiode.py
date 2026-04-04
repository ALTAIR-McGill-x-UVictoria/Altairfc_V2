from __future__ import annotations

from dataclasses import dataclass, field

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x04)
@dataclass
class PhotodiodePacket:
    """
    Photodiode interface HAT telemetry.
    Packet ID: 0x04

    DataStore keys (read by TelemetryTask):
        "photodiode.channel_0"
        "photodiode.channel_1"
        "photodiode.channel_2"
        "photodiode.channel_3"
    """

    DATASTORE_KEYS: dict[str, str] = {
        "channel_0": "photodiode.channel_0",
        "channel_1": "photodiode.channel_1",
        "channel_2": "photodiode.channel_2",
        "channel_3": "photodiode.channel_3",
    }

    channel_0: float = field(default=0.0, metadata=FieldMeta("f", "Photodiode channel 0", "V"))
    channel_1: float = field(default=0.0, metadata=FieldMeta("f", "Photodiode channel 1", "V"))
    channel_2: float = field(default=0.0, metadata=FieldMeta("f", "Photodiode channel 2", "V"))
    channel_3: float = field(default=0.0, metadata=FieldMeta("f", "Photodiode channel 3", "V"))
