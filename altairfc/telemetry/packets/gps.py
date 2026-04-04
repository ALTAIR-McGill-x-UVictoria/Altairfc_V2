from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x05)
@dataclass
class GpsPacket:
    """
    Fused GPS position from Pixhawk GLOBAL_POSITION_INT.
    Packet ID: 0x05
    Payload size: 5 * 4 = 20 bytes

    Source MAVLink message: GLOBAL_POSITION_INT
      lat/lon converted from 1e-7 deg -> deg
      alt/relative_alt converted from mm -> m
      hdg converted from cdeg -> deg

    DataStore keys (read by TelemetryTask):
        "mavlink.gps.lat"
        "mavlink.gps.lon"
        "mavlink.gps.alt"
        "mavlink.gps.relative_alt"
        "mavlink.gps.hdg"
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "lat":          "mavlink.gps.lat",
        "lon":          "mavlink.gps.lon",
        "alt":          "mavlink.gps.alt",
        "relative_alt": "mavlink.gps.relative_alt",
        "hdg":          "mavlink.gps.hdg",
    }

    lat:          float = field(default=0.0, metadata=FieldMeta("f", "Latitude",          "deg").as_metadata())
    lon:          float = field(default=0.0, metadata=FieldMeta("f", "Longitude",         "deg").as_metadata())
    alt:          float = field(default=0.0, metadata=FieldMeta("f", "Altitude MSL",      "m").as_metadata())
    relative_alt: float = field(default=0.0, metadata=FieldMeta("f", "Altitude AGL",      "m").as_metadata())
    hdg:          float = field(default=0.0, metadata=FieldMeta("f", "Heading",           "deg").as_metadata())
