from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.command_registry import command_registry
from telemetry.registry import FieldMeta


@command_registry.register(packet_id=0xC1)
@dataclass
class LaunchOkCommandPacket:
    """
    LAUNCH_OK command sent from GS to FC.
    Command ID: 0xC1
    Payload: 1 byte

    Effect: sets DataStore key "command.launch_ok" to 1.0.
    FlightStageTask reads and clears this key, bypassing altitude-gain detection
    and advancing directly to STAGE_LAUNCH when in STAGE_ARMED.
    """

    DATASTORE_KEY: ClassVar[str] = "command.launch_ok"

    confirm: int = field(
        default=1,
        metadata=FieldMeta("B", "Launch confirm", "bool").as_metadata(),
    )
