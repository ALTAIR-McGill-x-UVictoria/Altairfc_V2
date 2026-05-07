from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x05)
@dataclass
class LocalGpsPacket:
    """
    Position and status from the onboard MAX-M10M GPS module (I2C, address 0x42).
    Packet ID: 0x05
    Source: GpsTask -> DataStore "gps.*" keys
    """

    TX_RATE_HZ: ClassVar[float] = 1.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "active":      "gps.active",
        "lat":         "gps.lat",
        "lon":         "gps.lon",
        "alt_msl":     "gps.alt_msl",
        "speed_ms":    "gps.speed_ms",
        "heading_deg": "gps.heading_deg",
        "fix_type":    "gps.fix_type",
        "num_sv":      "gps.num_sv",
    }

    active:      int   = field(default=0,   metadata=FieldMeta("B", "Module active", "").as_metadata())
    lat:         float = field(default=0.0, metadata=FieldMeta("f", "Latitude",       "deg").as_metadata())
    lon:         float = field(default=0.0, metadata=FieldMeta("f", "Longitude",      "deg").as_metadata())
    alt_msl:     float = field(default=0.0, metadata=FieldMeta("f", "Altitude MSL",   "m").as_metadata())
    speed_ms:    float = field(default=0.0, metadata=FieldMeta("f", "Ground speed",   "m/s").as_metadata())
    heading_deg: float = field(default=0.0, metadata=FieldMeta("f", "Heading",        "deg").as_metadata())
    fix_type:    int   = field(default=0,   metadata=FieldMeta("B", "Fix type",       "").as_metadata())
    num_sv:      int   = field(default=0,   metadata=FieldMeta("B", "Satellites",     "").as_metadata())


@packet_registry.register(packet_id=0x08)
@dataclass
class MavlinkGpsPacket:
    """
    GPS position fused by the Pixhawk flight controller via MAVLink.
    Packet ID: 0x08
    Source: MavlinkTask -> DataStore "mavlink.gps.*" keys

    lat/lon/alt from GPS_RAW_INT, relative_alt from LOCAL_POSITION_NED,
    hdg from GPS_RAW_INT course-over-ground.
    """

    TX_RATE_HZ: ClassVar[float] = 1.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "lat":          "mavlink.gps.lat",
        "lon":          "mavlink.gps.lon",
        "alt":          "mavlink.gps.alt",
        "relative_alt": "mavlink.gps.relative_alt",
        "hdg":          "mavlink.gps.hdg",
        "num_sv":       "mavlink.gps.num_sv",
    }

    lat:          float = field(default=0.0, metadata=FieldMeta("f", "Latitude",     "deg").as_metadata())
    lon:          float = field(default=0.0, metadata=FieldMeta("f", "Longitude",    "deg").as_metadata())
    alt:          float = field(default=0.0, metadata=FieldMeta("f", "Altitude MSL", "m").as_metadata())
    relative_alt: float = field(default=0.0, metadata=FieldMeta("f", "Altitude AGL", "m").as_metadata())
    hdg:          float = field(default=0.0, metadata=FieldMeta("f", "Heading",      "deg").as_metadata())
    num_sv:       int   = field(default=0,   metadata=FieldMeta("B", "Satellites",   "").as_metadata())
