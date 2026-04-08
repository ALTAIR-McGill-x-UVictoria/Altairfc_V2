from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.command_registry import command_registry
from telemetry.registry import FieldMeta


@command_registry.register(packet_id=0xC2)
@dataclass
class PingCommandPacket:
    """
    PING command sent from GS to FC.
    Command ID: 0xC2
    Payload: 1 byte (token echoed back in the ACK cmd_seq field)

    No DataStore side effect. CommandReceiverTask immediately sends an ACK,
    confirming the round-trip link is alive. The GS measures latency from
    send time to ACK receipt.
    """

    token: int = field(
        default=0,
        metadata=FieldMeta("B", "Ping token", "id").as_metadata(),
    )
