from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x03)
@dataclass
class VescPacket:
    """
    VESC motor controller telemetry.
    Packet ID: 0x03

    DataStore keys (read by TelemetryTask):
        "rw.rpm"
        "rw.duty_cycle_now"
        "rw.avg_motor_current"
        "rw.v_in"
        "rw.temp_fet"
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "rpm":             "rw.rpm",
        "duty_cycle":      "rw.duty_cycle_now",
        "motor_current":   "rw.avg_motor_current",
        "input_voltage":   "rw.v_in",
        "temperature_mos": "rw.temp_fet",
    }

    rpm:             float = field(default=0.0, metadata=FieldMeta("f", "Motor RPM",          "rpm").as_metadata())
    duty_cycle:      float = field(default=0.0, metadata=FieldMeta("f", "Duty cycle",         "%").as_metadata())
    motor_current:   float = field(default=0.0, metadata=FieldMeta("f", "Motor phase current","A").as_metadata())
    input_voltage:   float = field(default=0.0, metadata=FieldMeta("f", "Input voltage",      "V").as_metadata())
    temperature_mos: float = field(default=0.0, metadata=FieldMeta("f", "MOSFET temperature", "degC").as_metadata())
