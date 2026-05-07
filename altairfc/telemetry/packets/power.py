from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x02)
@dataclass
class PowerPacket:
    """
    INA3221 three-channel power monitor telemetry.
    Packet ID: 0x02

    Channel mapping:
        CH1 → 24 V rail   (power.voltage_24v, power.current_24v)
        CH2 → 12 V rail   (power.voltage_12v, power.current_12v)
        CH3 →  5 V rail   (power.voltage_5v,  power.current_5v)
    """

    TX_RATE_HZ: ClassVar[float] = 1.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "voltage_24v": "power.voltage_24v",
        "current_24v": "power.current_24v",
        "voltage_12v": "power.voltage_12v",
        "current_12v": "power.current_12v",
        "voltage_5v":  "power.voltage_5v",
        "current_5v":  "power.current_5v",
    }

    voltage_24v: float = field(default=0.0, metadata=FieldMeta("f", "24 V rail voltage", "V").as_metadata())
    current_24v: float = field(default=0.0, metadata=FieldMeta("f", "24 V rail current", "A").as_metadata())
    voltage_12v: float = field(default=0.0, metadata=FieldMeta("f", "12 V rail voltage", "V").as_metadata())
    current_12v: float = field(default=0.0, metadata=FieldMeta("f", "12 V rail current", "A").as_metadata())
    voltage_5v:  float = field(default=0.0, metadata=FieldMeta("f", "5 V rail voltage",  "V").as_metadata())
    current_5v:  float = field(default=0.0, metadata=FieldMeta("f", "5 V rail current",  "A").as_metadata())
