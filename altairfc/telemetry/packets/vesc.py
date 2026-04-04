from __future__ import annotations

from dataclasses import dataclass, field

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x03)
@dataclass
class VescPacket:
    """
    VESC motor controller telemetry.
    Packet ID: 0x03

    DataStore keys (read by TelemetryTask):
        "vesc.rpm"
        "vesc.duty_cycle"
        "vesc.motor_current"
        "vesc.input_voltage"
        "vesc.temperature_mos"
    """

    DATASTORE_KEYS: dict[str, str] = {
        "rpm":           "vesc.rpm",
        "duty_cycle":    "vesc.duty_cycle",
        "motor_current": "vesc.motor_current",
        "input_voltage": "vesc.input_voltage",
        "temperature_mos": "vesc.temperature_mos",
    }

    rpm:             float = field(default=0.0, metadata=FieldMeta("f", "Motor RPM",               "rpm"))
    duty_cycle:      float = field(default=0.0, metadata=FieldMeta("f", "Duty cycle",              "%"))
    motor_current:   float = field(default=0.0, metadata=FieldMeta("f", "Motor phase current",     "A"))
    input_voltage:   float = field(default=0.0, metadata=FieldMeta("f", "Input voltage",           "V"))
    temperature_mos: float = field(default=0.0, metadata=FieldMeta("f", "MOSFET temperature",      "degC"))
