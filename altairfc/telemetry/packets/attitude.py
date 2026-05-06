from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x01)
@dataclass
class AttitudePacket:
    """
    MAVLink ATTITUDE data from Pixhawk 6X mini.
    Packet ID: 0x01
    Payload size: 6 * 4 = 24 bytes

    DataStore keys (read by TelemetryTask):
        "mavlink.attitude.roll"
        "mavlink.attitude.pitch"
        "mavlink.attitude.yaw"
        "mavlink.attitude.rollspeed"
        "mavlink.attitude.pitchspeed"
        "mavlink.attitude.yawspeed"
    """

    TX_RATE_HZ: ClassVar[float] = 4.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "roll":       "mavlink.attitude.roll",
        "pitch":      "mavlink.attitude.pitch",
        "yaw":        "mavlink.attitude.yaw",
        "rollspeed":  "mavlink.attitude.rollspeed",
        "pitchspeed": "mavlink.attitude.pitchspeed",
        "yawspeed":   "mavlink.attitude.yawspeed",
    }

    roll:       float = field(default=0.0, metadata=FieldMeta("f", "Roll angle",  "rad").as_metadata())
    pitch:      float = field(default=0.0, metadata=FieldMeta("f", "Pitch angle", "rad").as_metadata())
    yaw:        float = field(default=0.0, metadata=FieldMeta("f", "Yaw angle",   "rad").as_metadata())
    rollspeed:  float = field(default=0.0, metadata=FieldMeta("f", "Roll rate",   "rad/s").as_metadata())
    pitchspeed: float = field(default=0.0, metadata=FieldMeta("f", "Pitch rate",  "rad/s").as_metadata())
    yawspeed:   float = field(default=0.0, metadata=FieldMeta("f", "Yaw rate",    "rad/s").as_metadata())
