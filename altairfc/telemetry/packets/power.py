from __future__ import annotations

from dataclasses import dataclass, field

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x02)
@dataclass
class PowerPacket:
    """
    Power distribution HAT telemetry.
    Packet ID: 0x02

    DataStore keys (read by TelemetryTask):
        "power.voltage_bus"
        "power.current_total"
        "power.temperature"
    """

    DATASTORE_KEYS: dict[str, str] = {
        "voltage_bus":    "power.voltage_bus",
        "current_total":  "power.current_total",
        "temperature":    "power.temperature",
    }

    voltage_bus:   float = field(default=0.0, metadata=FieldMeta("f", "Bus voltage",        "V").as_metadata())
    current_total: float = field(default=0.0, metadata=FieldMeta("f", "Total current draw", "A").as_metadata())
    temperature:   float = field(default=0.0, metadata=FieldMeta("f", "Board temperature",  "degC").as_metadata())
