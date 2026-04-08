from __future__ import annotations

from dataclasses import dataclass, field

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0xA0)
@dataclass
class AckPacket:
    """
    FC→GS acknowledgement for a received command frame.

    Packet ID: 0xA0
    Payload size: 3 bytes
    Direction: FC → GS (sent by CommandReceiverTask, never by TelemetryTask)

    TelemetryTask skips this packet because it has no DATASTORE_KEYS.
    CommandReceiverTask builds and sends it manually via SerialTransport.send().

    Fields:
        cmd_id  — packet ID of the command being acknowledged (e.g. 0xC0 for ARM)
        cmd_seq — sequence number echoed from the command frame header
        status  — 0 = accepted, 1 = rejected (e.g. LAUNCH_OK in wrong stage)
    """

    cmd_id:  int = field(default=0,   metadata=FieldMeta("B", "Command ID",       "id").as_metadata())
    cmd_seq: int = field(default=0,   metadata=FieldMeta("B", "Command sequence",  "count").as_metadata())
    status:  int = field(default=0,   metadata=FieldMeta("B", "ACK status",        "0=ok,1=rej").as_metadata())
