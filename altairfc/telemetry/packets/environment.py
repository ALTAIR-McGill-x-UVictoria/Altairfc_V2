from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x06)
@dataclass
class EnvironmentPacket:
    """
    Barometric and airspeed telemetry from the Pixhawk.

    Packet ID: 0x06
    Payload size: 7 * 4 = 28 bytes

    Sources:
        SCALED_PRESSURE (MAVLink ID 29):
            press_abs   — absolute (static) pressure, hPa
            press_diff  — differential pressure, hPa
            temperature — air temperature, centidegrees → °C

        VFR_HUD (MAVLink ID 74):
            baro_alt    — barometric altitude MSL, m
            climb       — ascent/descent rate (positive = up), m/s
            airspeed    — indicated airspeed, m/s
            groundspeed — GPS ground speed, m/s

    DataStore keys (written by MavlinkTask):
        "mavlink.environment.press_abs"
        "mavlink.environment.press_diff"
        "mavlink.environment.temperature"
        "mavlink.environment.baro_alt"
        "mavlink.environment.climb"
        "mavlink.environment.airspeed"
        "mavlink.environment.groundspeed"
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "press_abs":   "mavlink.environment.press_abs",
        "press_diff":  "mavlink.environment.press_diff",
        "temperature": "mavlink.environment.temperature",
        "baro_alt":    "mavlink.environment.baro_alt",
        "climb":       "mavlink.environment.climb",
        "airspeed":    "mavlink.environment.airspeed",
        "groundspeed": "mavlink.environment.groundspeed",
    }

    press_abs:   float = field(default=0.0, metadata=FieldMeta("f", "Absolute pressure",    "hPa").as_metadata())
    press_diff:  float = field(default=0.0, metadata=FieldMeta("f", "Differential pressure", "hPa").as_metadata())
    temperature: float = field(default=0.0, metadata=FieldMeta("f", "Air temperature",       "degC").as_metadata())
    baro_alt:    float = field(default=0.0, metadata=FieldMeta("f", "Barometric altitude",   "m").as_metadata())
    climb:       float = field(default=0.0, metadata=FieldMeta("f", "Ascent/descent rate",   "m/s").as_metadata())
    airspeed:    float = field(default=0.0, metadata=FieldMeta("f", "Airspeed",              "m/s").as_metadata())
    groundspeed: float = field(default=0.0, metadata=FieldMeta("f", "Ground speed",          "m/s").as_metadata())
