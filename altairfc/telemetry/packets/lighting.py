from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x0B)
@dataclass
class LightingPacket:
    """
    Sphere source / spotter beacon interlock state.
    Packet ID: 0x0B
    Source: LightingTask -> DataStore "lighting.*" keys
    """

    TX_RATE_HZ: ClassVar[float] = 1.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "sphere_on":        "lighting.sphere_on",
        "beacon_on":        "lighting.beacon_on",
        "window_active":    "lighting.window_active",
        "next_window_in_s": "lighting.next_window_in_s",
    }

    sphere_on:        int   = field(default=0,   metadata=FieldMeta("B", "Sphere source on",     "").as_metadata())
    beacon_on:        int   = field(default=0,   metadata=FieldMeta("B", "Spotter beacon on",     "").as_metadata())
    window_active:    int   = field(default=0,   metadata=FieldMeta("B", "Imaging window active", "").as_metadata())
    next_window_in_s: float = field(default=0.0, metadata=FieldMeta("f", "Time to next window",   "s").as_metadata())
