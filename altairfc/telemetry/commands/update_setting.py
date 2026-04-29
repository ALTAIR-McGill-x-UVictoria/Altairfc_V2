from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.command_registry import command_registry
from telemetry.registry import FieldMeta

# Ordered mapping of field_id (index) → DataStore key.
# Must stay in sync with FlightSettingsPacket.DATASTORE_KEYS field order
# and with SETTING_KEYS in ground/receiver.py.
SETTING_KEYS: tuple[str, ...] = (
    "settings.termination_altitude_m",        # 0
    "settings.burst_altitude_m",              # 1
    "settings.burst_altitude_uncertainty_m",  # 2
    "settings.ascent_detect_window_s",        # 3
    "settings.ascent_detect_gain_m",          # 4
    "settings.apogee_fraction",               # 5
    "settings.landing_fraction",              # 6
    "settings.recovery_stationary_s",         # 7
    "settings.termination_confirm_drop_m",    # 8
    "settings.termination_confirm_window_s",  # 9
    "settings.rw_kp",                         # 10
    "settings.rw_kd",                         # 11
    "settings.rw_max_rpm",                    # 12
    "settings.mm_kp",                         # 13
    "settings.mm_kd",                         # 14
    "settings.mm_max_current",                # 15
)


@command_registry.register(packet_id=0xC3)
@dataclass
class UpdateSettingCommandPacket:
    """
    GS→FC: update one flight setting by index.

    Command ID: 0xC3
    Payload: 5 bytes (uint8 field_id + float32 value)

    field_id indexes into SETTING_KEYS above. CommandReceiverTask detects
    this command via the SETTING_DISPATCH marker and writes the value to
    the corresponding DataStore key.
    """

    SETTING_DISPATCH: ClassVar[bool] = True

    # Maps bare field name (without "settings." prefix) → field_id integer.
    # Derived from SETTING_KEYS so any reordering here stays in sync automatically.
    SETTING_FIELD_IDS: ClassVar[dict[str, int]] = {
        key.removeprefix("settings."): idx
        for idx, key in enumerate(SETTING_KEYS)
    }

    field_id: int   = field(default=0,   metadata=FieldMeta("B", "Setting field ID", "index").as_metadata())
    value:    float = field(default=0.0, metadata=FieldMeta("f", "Setting value",    "varies").as_metadata())
