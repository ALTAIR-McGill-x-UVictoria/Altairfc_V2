from __future__ import annotations

import dataclasses
import struct
from typing import Callable, ClassVar, Optional


@dataclasses.dataclass(frozen=True)
class FieldMeta:
    """
    Metadata descriptor for a packet dataclass field.

    struct_char: Python struct format character for this field.
      Common values:
        'f' — float32 (4 bytes)
        'd' — float64 (8 bytes)
        'i' — int32   (4 bytes)
        'I' — uint32  (4 bytes)
        'h' — int16   (2 bytes)
        'H' — uint16  (2 bytes)
        'b' — int8    (1 byte)
        'B' — uint8   (1 byte)
    description: Human-readable field description.
    units: Physical units string (e.g. "rad", "m/s", "V").
    group: Display group name for UI organisation (e.g. "Flight Parameters").
           Empty string means ungrouped.
    min_val: Minimum valid value (inclusive). None means no lower bound.
    max_val: Maximum valid value (inclusive). None means no upper bound.

    Usage with dataclasses.field():
        roll: float = field(default=0.0, metadata=FieldMeta("f", "Roll angle", "rad").as_metadata())
    """

    struct_char: str
    description: str
    units: str
    group:   str            = ""
    min_val: Optional[float] = None
    max_val: Optional[float] = None

    def as_metadata(self) -> dict:
        """Wrap in a dict so dataclasses.field(metadata=...) accepts it."""
        return {"field_meta": self}


class _RegistryEntry:
    __slots__ = ("packet_class", "packet_struct", "packet_id")

    def __init__(self, packet_class: type, packet_struct: struct.Struct, packet_id: int) -> None:
        self.packet_class = packet_class
        self.packet_struct = packet_struct
        self.packet_id = packet_id


class PacketRegistry:
    """
    Singleton registry that maps packet IDs to packet dataclasses.

    Populated at module import time via the @register decorator.
    The serializer queries this registry to look up packet_id and the
    pre-compiled struct.Struct for any given packet type.

    Usage:
        @packet_registry.register(packet_id=0x01)
        @dataclass
        class AttitudePacket:
            roll: float = field(metadata=FieldMeta("f", "Roll angle", "rad"))
            ...
    """

    _instance: ClassVar["PacketRegistry | None"] = None

    def __new__(cls) -> "PacketRegistry":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._id_to_entry: dict[int, _RegistryEntry] = {}
            cls._instance._type_to_id: dict[type, int] = {}
        return cls._instance

    def register(self, packet_id: int) -> Callable[[type], type]:
        """Class decorator. Reads field FieldMeta, compiles struct.Struct, registers the type."""

        def decorator(cls_: type) -> type:
            if packet_id in self._id_to_entry:
                existing = self._id_to_entry[packet_id].packet_class.__name__
                raise ValueError(
                    f"Packet ID 0x{packet_id:02X} already registered by {existing}"
                )
            if not dataclasses.is_dataclass(cls_):
                raise TypeError(f"{cls_.__name__} must be a @dataclass to be registered")

            fmt_chars = []
            for f in dataclasses.fields(cls_):
                meta = f.metadata
                if "field_meta" in meta:
                    fmt_chars.append(meta["field_meta"].struct_char)
                else:
                    raise TypeError(
                        f"Field '{f.name}' in {cls_.__name__} is missing FieldMeta metadata. "
                        f"Use: field(default=..., metadata=FieldMeta(...).as_metadata())"
                    )

            fmt = "<" + "".join(fmt_chars)
            compiled = struct.Struct(fmt)

            entry = _RegistryEntry(cls_, compiled, packet_id)
            self._id_to_entry[packet_id] = entry
            self._type_to_id[cls_] = packet_id
            return cls_

        return decorator

    def get_by_id(self, packet_id: int) -> tuple[type, struct.Struct] | None:
        entry = self._id_to_entry.get(packet_id)
        if entry is None:
            return None
        return entry.packet_class, entry.packet_struct

    def get_id(self, packet_type: type) -> int | None:
        return self._type_to_id.get(packet_type)

    def get_struct(self, packet_type: type) -> struct.Struct | None:
        pid = self._type_to_id.get(packet_type)
        if pid is None:
            return None
        return self._id_to_entry[pid].packet_struct

    def all_packets(self) -> dict[int, type]:
        return {pid: e.packet_class for pid, e in self._id_to_entry.items()}


packet_registry = PacketRegistry()
