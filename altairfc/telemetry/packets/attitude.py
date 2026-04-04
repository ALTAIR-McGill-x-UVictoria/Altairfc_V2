from __future__ import annotations

from dataclasses import dataclass, field

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

    DATASTORE_KEYS: dict[str, str] = {
        "roll":       "mavlink.attitude.roll",
        "pitch":      "mavlink.attitude.pitch",
        "yaw":        "mavlink.attitude.yaw",
        "rollspeed":  "mavlink.attitude.rollspeed",
        "pitchspeed": "mavlink.attitude.pitchspeed",
        "yawspeed":   "mavlink.attitude.yawspeed",
    }

    roll:       float = field(default=0.0, metadata=FieldMeta("f", "Roll angle",  "rad"))
    pitch:      float = field(default=0.0, metadata=FieldMeta("f", "Pitch angle", "rad"))
    yaw:        float = field(default=0.0, metadata=FieldMeta("f", "Yaw angle",   "rad"))
    rollspeed:  float = field(default=0.0, metadata=FieldMeta("f", "Roll rate",   "rad/s"))
    pitchspeed: float = field(default=0.0, metadata=FieldMeta("f", "Pitch rate",  "rad/s"))
    yawspeed:   float = field(default=0.0, metadata=FieldMeta("f", "Yaw rate",    "rad/s"))
