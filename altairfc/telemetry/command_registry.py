from __future__ import annotations

from telemetry.registry import PacketRegistry


class CommandRegistry(PacketRegistry):
    """
    Independent registry for GS→FC command packets.

    Structurally identical to PacketRegistry but uses a separate singleton
    so command IDs never collide with telemetry packet IDs and command classes
    are never iterated by TelemetryTask.

    Usage:
        @command_registry.register(packet_id=0xC0)
        @dataclass
        class ArmCommandPacket:
            ...
    """

    _instance = None  # Independent singleton — not shared with PacketRegistry


command_registry = CommandRegistry()
