from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x0A)
@dataclass
class PointingPacket:
    """
    Light-source pointing state — yaw and nadir angles with their errors.
    Packet ID: 0x0A
    Payload size: 4 * 4 = 16 bytes (all float32)

    DataStore keys (written by RWTask each control cycle):
        pointing.target_heading_rad   — target yaw toward GS (rad, -π..+π)
        pointing.heading_error_rad    — azimuth error to GS in body frame (rad)
        pointing.source_angle_deg     — commanded source nadir angle (deg)
        pointing.source_angle_error_deg — pitch error converted to source angle error (deg)
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "target_heading_rad":      "pointing.target_heading_rad",
        "heading_error_rad":       "pointing.heading_error_rad",
        "source_angle_deg":        "pointing.source_angle_deg",
        "source_angle_error_deg":  "pointing.source_angle_error_deg",
    }

    target_heading_rad:     float = field(default=0.0, metadata=FieldMeta("f", "Target yaw toward GS",           "rad", min_val=-3.1416, max_val=3.1416).as_metadata())
    heading_error_rad:      float = field(default=0.0, metadata=FieldMeta("f", "Azimuth error to GS",            "rad", min_val=-3.1416, max_val=3.1416).as_metadata())
    source_angle_deg:       float = field(default=0.0, metadata=FieldMeta("f", "Commanded source nadir angle",   "deg", min_val=0.0,     max_val=90.0  ).as_metadata())
    source_angle_error_deg: float = field(default=0.0, metadata=FieldMeta("f", "Source nadir angle error to GS", "deg", min_val=-90.0,   max_val=90.0  ).as_metadata())
