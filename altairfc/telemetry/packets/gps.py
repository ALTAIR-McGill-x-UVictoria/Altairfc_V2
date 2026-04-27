from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x05)
@dataclass
class GpsPacket:
    """
    GPS position and status.
    Packet ID: 0x05
    Payload size: 3*8 + 4*4 + 3*1 = 43 bytes (padded to 44 by struct alignment)

    lat/lon/alt_msl sourced from direct MAX-M10M GPS driver ("gps.*" keys).
    relative_alt sourced from Pixhawk LOCAL_POSITION_NED ("mavlink.gps.relative_alt").
    speed_ms, heading_deg, fix_type, num_sv sourced from GPS driver.

    DataStore keys (read by TelemetryTask):
        "gps.lat"
        "gps.lon"
        "gps.alt_msl"
        "mavlink.gps.relative_alt"
        "gps.speed_ms"
        "gps.heading_deg"
        "gps.fix_type"
        "gps.num_sv"
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "lat":          "gps.lat",
        "lon":          "gps.lon",
        "alt_msl":      "gps.alt_msl",
        "relative_alt": "mavlink.gps.relative_alt",
        "speed_ms":     "gps.speed_ms",
        "heading_deg":  "gps.heading_deg",
        "fix_type":     "gps.fix_type",
        "num_sv":       "gps.num_sv",
    }

    lat:          float = field(default=0.0, metadata=FieldMeta("f", "Latitude",      "deg").as_metadata())
    lon:          float = field(default=0.0, metadata=FieldMeta("f", "Longitude",     "deg").as_metadata())
    alt_msl:      float = field(default=0.0, metadata=FieldMeta("f", "Altitude MSL",  "m").as_metadata())
    relative_alt: float = field(default=0.0, metadata=FieldMeta("f", "Altitude AGL",  "m").as_metadata())
    speed_ms:     float = field(default=0.0, metadata=FieldMeta("f", "Ground speed",  "m/s").as_metadata())
    heading_deg:  float = field(default=0.0, metadata=FieldMeta("f", "Heading",       "deg").as_metadata())
    fix_type:     int   = field(default=0,   metadata=FieldMeta("B", "Fix type",      "").as_metadata())
    num_sv:       int   = field(default=0,   metadata=FieldMeta("B", "Satellites",    "").as_metadata())
