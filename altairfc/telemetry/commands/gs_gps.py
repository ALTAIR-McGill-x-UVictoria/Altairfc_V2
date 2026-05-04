from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.command_registry import command_registry
from telemetry.registry import FieldMeta


@command_registry.register(packet_id=0xC4)
@dataclass
class GsGpsCommandPacket:
    """
    GS_GPS command sent from GS to FC.
    Command ID: 0xC4
    Payload: 12 bytes (3 × float32)

    Carries the ground station's current GPS position so the flight computer
    can point its downward-facing light source toward the GS for optical
    observation. The GS sends this packet every 2 seconds while connected.

    Effect: sets DataStore keys:
        "command.gs_lat"  — GS geodetic latitude  (degrees, +N)
        "command.gs_lon"  — GS geodetic longitude (degrees, +E)
        "command.gs_alt"  — GS altitude MSL       (metres)
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "lat": "command.gs_lat",
        "lon": "command.gs_lon",
        "alt": "command.gs_alt",
    }

    lat: float = field(
        default=0.0,
        metadata=FieldMeta("f", "GS latitude",  "deg", min_val=-90.0,  max_val=90.0).as_metadata(),
    )
    lon: float = field(
        default=0.0,
        metadata=FieldMeta("f", "GS longitude", "deg", min_val=-180.0, max_val=180.0).as_metadata(),
    )
    alt: float = field(
        default=0.0,
        metadata=FieldMeta("f", "GS altitude",  "m",   min_val=-500.0, max_val=50000.0).as_metadata(),
    )
