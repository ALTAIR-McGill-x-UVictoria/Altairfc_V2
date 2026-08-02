from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x0E)
@dataclass
class LightingPacket:
    """
    Sphere source / spotter beacon interlock state.
    Packet ID: 0x0E
    Source: LightingTask -> DataStore "lighting.*" keys
    """

    TX_RATE_HZ: ClassVar[float] = 1.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "sphere_on":               "lighting.sphere_on",
        "beacon_on":               "lighting.beacon_on",
        "observation_active":      "lighting.observation_active",
        "next_beacon_flash_in_s":  "lighting.next_beacon_flash_in_s",
    }

    sphere_on:              int   = field(default=0,   metadata=FieldMeta("B", "Sphere source on",                 "").as_metadata())
    beacon_on:              int   = field(default=0,   metadata=FieldMeta("B", "Spotter beacon on",                "").as_metadata())
    observation_active:     int   = field(default=0,   metadata=FieldMeta("B", "Sphere observation window active", "").as_metadata())
    next_beacon_flash_in_s: float = field(default=0.0, metadata=FieldMeta("f", "Time to next beacon flash",        "s").as_metadata())
