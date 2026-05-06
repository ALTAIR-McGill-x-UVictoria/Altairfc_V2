from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x07)
@dataclass
class EventPacket:
    """
    Flight event flags and stage indicator.

    Packet ID: 0x07
    Payload size: 1 + 10 * 1 = 11 bytes

    Fields:
        flight_stage        — current mission phase (uint8 enum, see FLIGHT_STAGES)
        arm_state           — motors armed (uint8 bool, 0/1)
        launch_detected     — launch detected (uint8 bool)
        ascent_active       — balloon in ascent phase (uint8 bool)
        termination_fired   — cutdown confirmed and descent initiated (uint8 bool)
        burst_detected      — natural balloon burst detected (uint8 bool)
        descent_active      — under parachute / descent (uint8 bool)
        landing_detected    — touchdown detected (uint8 bool)
        cutdown_fired       — cutdown mechanism fired (uint8 bool)
        recovery_active     — recovery beacon active (uint8 bool)
        data_logging_active — onboard SD logging active (uint8 bool)

    DataStore keys (written by FlightStageTask):
        "event.flight_stage"
        "event.arm_state"
        "event.launch_detected"
        "event.ascent_active"
        "event.termination_fired"
        "event.burst_detected"
        "event.descent_active"
        "event.landing_detected"
        "event.cutdown_fired"
        "event.recovery_active"
        "event.data_logging_active"

    FLIGHT_STAGES:
        0 — Pre-flight
        1 — Armed
        2 — Launch
        3 — Ascent
        4 — Termination
        5 — Burst
        6 — Descent
        7 — Landing
        8 — Recovery
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "flight_stage":       "event.flight_stage",
        "arm_state":          "event.arm_state",
        "preflight_ok":       "event.preflight_ok",
        "arm_checks_ok":      "event.arm_checks_ok",
        "launch_detected":    "event.launch_detected",
        "ascent_active":      "event.ascent_active",
        "termination_fired":  "event.termination_fired",
        "burst_detected":     "event.burst_detected",
        "descent_active":     "event.descent_active",
        "landing_detected":   "event.landing_detected",
        "cutdown_fired":      "event.cutdown_fired",
        "recovery_active":    "event.recovery_active",
        "data_logging_active":"event.data_logging_active",
    }

    flight_stage:        int = field(default=0, metadata=FieldMeta("B", "Flight stage",         "stage").as_metadata())
    arm_state:           int = field(default=0, metadata=FieldMeta("B", "Arm state",            "bool").as_metadata())
    preflight_ok:        int = field(default=0, metadata=FieldMeta("B", "Preflight checks OK",  "bool").as_metadata())
    arm_checks_ok:       int = field(default=0, metadata=FieldMeta("B", "Arm checks OK",        "bool").as_metadata())
    launch_detected:     int = field(default=0, metadata=FieldMeta("B", "Launch detected",      "bool").as_metadata())
    ascent_active:       int = field(default=0, metadata=FieldMeta("B", "Ascent active",        "bool").as_metadata())
    termination_fired:   int = field(default=0, metadata=FieldMeta("B", "Termination fired",    "bool").as_metadata())
    burst_detected:      int = field(default=0, metadata=FieldMeta("B", "Burst detected",       "bool").as_metadata())
    descent_active:      int = field(default=0, metadata=FieldMeta("B", "Descent active",       "bool").as_metadata())
    landing_detected:    int = field(default=0, metadata=FieldMeta("B", "Landing detected",     "bool").as_metadata())
    cutdown_fired:       int = field(default=0, metadata=FieldMeta("B", "Cutdown fired",        "bool").as_metadata())
    recovery_active:     int = field(default=0, metadata=FieldMeta("B", "Recovery active",      "bool").as_metadata())
    data_logging_active: int = field(default=0, metadata=FieldMeta("B", "Data logging active",  "bool").as_metadata())
