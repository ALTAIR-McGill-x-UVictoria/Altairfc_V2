from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x0E)
@dataclass
class LightingPacket:
    """
    Sphere source / spotter beacon state.
    Packet ID: 0x0E
    Source: LightingTask -> DataStore "lighting.*" keys

    sphere_dac_code/beacon_dac_code are the live MCP4728 codes actually
    written to each channel (SphereLedSource.code / BeaconChannel.code) —
    not the configured setpoints, which may differ while the sphere's PI
    loop is still converging. *_current_a are ADS1115 current-sense
    readings, not commanded targets.
    """

    TX_RATE_HZ: ClassVar[float] = 1.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "sphere_on":               "lighting.sphere_on",
        "beacon_on":               "lighting.beacon_on",
        "observation_active":      "lighting.observation_active",
        "next_beacon_flash_in_s":  "lighting.next_beacon_flash_in_s",
        "sphere_dac_code":         "lighting.sphere_dac_code",
        "sphere_current_a":        "lighting.sphere_current_a",
        "sphere_temperature_c":    "lighting.sphere_temperature_c",
        "beacon_dac_code":         "lighting.beacon_dac_code",
        "beacon_current_a":        "lighting.beacon_current_a",
    }

    sphere_on:              int   = field(default=0,   metadata=FieldMeta("B", "Sphere source on",                 "").as_metadata())
    beacon_on:              int   = field(default=0,   metadata=FieldMeta("B", "Spotter beacon on",                "").as_metadata())
    observation_active:     int   = field(default=0,   metadata=FieldMeta("B", "Sphere observation window active", "").as_metadata())
    next_beacon_flash_in_s: float = field(default=0.0, metadata=FieldMeta("f", "Time to next beacon flash",        "s").as_metadata())
    sphere_dac_code:        int   = field(default=0,   metadata=FieldMeta("H", "Sphere DAC output code",           "code").as_metadata())
    sphere_current_a:       float = field(default=0.0, metadata=FieldMeta("f", "Sphere measured current",         "A").as_metadata())
    sphere_temperature_c:   float = field(default=0.0, metadata=FieldMeta("f", "Sphere source temperature",       "C").as_metadata())
    beacon_dac_code:        int   = field(default=0,   metadata=FieldMeta("H", "Beacon DAC output code",           "code").as_metadata())
    beacon_current_a:       float = field(default=0.0, metadata=FieldMeta("f", "Beacon measured current",         "A").as_metadata())
