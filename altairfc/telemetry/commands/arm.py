from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.command_registry import command_registry
from telemetry.registry import FieldMeta


@command_registry.register(packet_id=0xC0)
@dataclass
class ArmCommandPacket:
    """
    ARM command sent from GS to FC.
    Command ID: 0xC0
    Payload: 1 byte

    Effect: sets DataStore key "command.arm" to 1.0.
    FlightStageTask reads and clears this key, then sets event.arm_state = 1.
    """

    DATASTORE_KEY: ClassVar[str] = "command.arm"

    arm_state: int = field(
        default=1,
        metadata=FieldMeta("B", "Arm state", "bool").as_metadata(),
    )
