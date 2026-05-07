from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x03)
@dataclass
class RWVescPacket:
    """
    Reaction wheel VESC telemetry.
    Packet ID: 0x03

    DataStore keys (written by RWTask._store()):
        rw.rpm
        rw.duty_now
        rw.current_motor
        rw.v_in
        rw.temp_pcb
        rw.mc_fault_code
    """

    TX_RATE_HZ: ClassVar[float] = 2.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "rpm":           "rw.rpm",
        "duty_cycle":    "rw.duty_now",
        "motor_current": "rw.current_motor",
        "input_voltage": "rw.v_in",
        "temperature":   "rw.temp_pcb",
        "fault_code":    "rw.mc_fault_code",
    }

    rpm:           float = field(default=0.0, metadata=FieldMeta("f", "Motor RPM",          "rpm").as_metadata())
    duty_cycle:    float = field(default=0.0, metadata=FieldMeta("f", "Duty cycle",         "%").as_metadata())
    motor_current: float = field(default=0.0, metadata=FieldMeta("f", "Motor phase current","A").as_metadata())
    input_voltage: float = field(default=0.0, metadata=FieldMeta("f", "Input voltage",      "V").as_metadata())
    temperature:   float = field(default=0.0, metadata=FieldMeta("f", "PCB temperature",    "degC").as_metadata())
    fault_code:    int   = field(default=0,   metadata=FieldMeta("B", "Fault code",         "").as_metadata())


@packet_registry.register(packet_id=0x0B)
@dataclass
class MMVescPacket:
    """
    Momentum management VESC telemetry.
    Packet ID: 0x0B

    DataStore keys (written by MMTask._store()):
        mm.rpm
        mm.duty_now
        mm.current_motor
        mm.v_in
        mm.temp_pcb
        mm.mc_fault_code
    """

    TX_RATE_HZ: ClassVar[float] = 2.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "rpm":           "mm.rpm",
        "duty_cycle":    "mm.duty_now",
        "motor_current": "mm.current_motor",
        "input_voltage": "mm.v_in",
        "temperature":   "mm.temp_pcb",
        "fault_code":    "mm.mc_fault_code",
    }

    rpm:           float = field(default=0.0, metadata=FieldMeta("f", "Motor RPM",          "rpm").as_metadata())
    duty_cycle:    float = field(default=0.0, metadata=FieldMeta("f", "Duty cycle",         "%").as_metadata())
    motor_current: float = field(default=0.0, metadata=FieldMeta("f", "Motor phase current","A").as_metadata())
    input_voltage: float = field(default=0.0, metadata=FieldMeta("f", "Input voltage",      "V").as_metadata())
    temperature:   float = field(default=0.0, metadata=FieldMeta("f", "PCB temperature",    "degC").as_metadata())
    fault_code:    int   = field(default=0,   metadata=FieldMeta("B", "Fault code",         "").as_metadata())
