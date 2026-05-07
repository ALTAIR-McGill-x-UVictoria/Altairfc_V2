from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x09)
@dataclass
class FlightSettingsPacket:
    """
    Active flight configuration snapshot — sent every telemetry cycle so the
    ground station can verify settings and detect when an UpdateSetting command
    has taken effect.

    Packet ID: 0x09
    Payload size: 18 * 4 = 72 bytes (all float32)

    DataStore keys (written by main.py at startup; updated by UpdateSettingCommand):
        settings.termination_altitude_m
        settings.burst_altitude_m
        settings.burst_altitude_uncertainty_m
        settings.ascent_detect_window_s
        settings.ascent_detect_gain_m
        settings.apogee_fraction
        settings.landing_fraction
        settings.recovery_stationary_s
        settings.termination_confirm_drop_m
        settings.termination_confirm_window_s
        settings.rw_kp
        settings.rw_kd
        settings.rw_max_rpm
        settings.mm_kp
        settings.mm_kd
        settings.mm_max_current
        settings.pointing_activate_altitude_m
        settings.pointing_duration_min
    """

    TX_RATE_HZ: ClassVar[float] = 0.2

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "termination_altitude_m":       "settings.termination_altitude_m",
        "burst_altitude_m":             "settings.burst_altitude_m",
        "burst_altitude_uncertainty_m": "settings.burst_altitude_uncertainty_m",
        "ascent_detect_window_s":       "settings.ascent_detect_window_s",
        "ascent_detect_gain_m":         "settings.ascent_detect_gain_m",
        "apogee_fraction":              "settings.apogee_fraction",
        "landing_fraction":             "settings.landing_fraction",
        "recovery_stationary_s":        "settings.recovery_stationary_s",
        "termination_confirm_drop_m":   "settings.termination_confirm_drop_m",
        "termination_confirm_window_s": "settings.termination_confirm_window_s",
        "rw_kp":                        "settings.rw_kp",
        "rw_kd":                        "settings.rw_kd",
        "rw_max_rpm":                   "settings.rw_max_rpm",
        "mm_kp":                        "settings.mm_kp",
        "mm_kd":                        "settings.mm_kd",
        "mm_max_current":               "settings.mm_max_current",
        "pointing_activate_altitude_m": "settings.pointing_activate_altitude_m",
        "pointing_duration_min":        "settings.pointing_duration_min",
    }

    _FP  = "Flight Parameters"
    _RW  = "Reaction Wheel PID"
    _MM  = "Momentum Management PID"
    _PT  = "Pointing"

    termination_altitude_m:       float = field(default=0.0, metadata=FieldMeta("f", "Cutdown altitude",                   "m",        group=_FP, min_val=0.0,     max_val=40000.0).as_metadata())
    burst_altitude_m:             float = field(default=0.0, metadata=FieldMeta("f", "Expected burst altitude",            "m",        group=_FP, min_val=0.0,     max_val=40000.0).as_metadata())
    burst_altitude_uncertainty_m: float = field(default=0.0, metadata=FieldMeta("f", "Burst altitude window",              "m",        group=_FP, min_val=0.0,     max_val=5000.0 ).as_metadata())
    ascent_detect_window_s:       float = field(default=0.0, metadata=FieldMeta("f", "Ascent detect window",               "s",        group=_FP, min_val=1.0,     max_val=300.0  ).as_metadata())
    ascent_detect_gain_m:         float = field(default=0.0, metadata=FieldMeta("f", "Ascent detect gain",                 "m",        group=_FP, min_val=0.0,     max_val=500.0  ).as_metadata())
    apogee_fraction:              float = field(default=0.0, metadata=FieldMeta("f", "Descent trigger fraction of apogee", "fraction", group=_FP, min_val=0.0,     max_val=1.0    ).as_metadata())
    landing_fraction:             float = field(default=0.0, metadata=FieldMeta("f", "Landing trigger fraction of apogee","fraction", group=_FP, min_val=0.0,     max_val=1.0    ).as_metadata())
    recovery_stationary_s:        float = field(default=0.0, metadata=FieldMeta("f", "Recovery stationary time",          "s",        group=_FP, min_val=0.0,     max_val=600.0  ).as_metadata())
    termination_confirm_drop_m:   float = field(default=0.0, metadata=FieldMeta("f", "Termination confirm drop",          "m",        group=_FP, min_val=0.0,     max_val=1000.0 ).as_metadata())
    termination_confirm_window_s: float = field(default=0.0, metadata=FieldMeta("f", "Termination confirm window",        "s",        group=_FP, min_val=0.0,     max_val=60.0   ).as_metadata())
    rw_kp:                        float = field(default=0.0, metadata=FieldMeta("f", "Reaction wheel Kp",                 "RPM/rad/s",group=_RW, min_val=0.0,     max_val=1000.0 ).as_metadata())
    rw_kd:                        float = field(default=0.0, metadata=FieldMeta("f", "Reaction wheel Kd",                 "RPM/rad/s²",group=_RW,min_val=0.0,     max_val=1000.0 ).as_metadata())
    rw_max_rpm:                   float = field(default=0.0, metadata=FieldMeta("f", "Reaction wheel max RPM",            "RPM",      group=_RW, min_val=0.0,     max_val=10000.0).as_metadata())
    mm_kp:                        float = field(default=0.0, metadata=FieldMeta("f", "Momentum management Kp",            "A/rad/s",  group=_MM, min_val=0.0,     max_val=100.0  ).as_metadata())
    mm_kd:                        float = field(default=0.0, metadata=FieldMeta("f", "Momentum management Kd",            "A/rad/s²", group=_MM, min_val=0.0,     max_val=100.0  ).as_metadata())
    mm_max_current:               float = field(default=0.0, metadata=FieldMeta("f", "Momentum management max current",   "mA",       group=_MM, min_val=0.0,     max_val=5000.0 ).as_metadata())
    pointing_activate_altitude_m: float = field(default=0.0, metadata=FieldMeta("f", "Pointing activate altitude",        "m",        group=_PT, min_val=0.0,     max_val=40000.0).as_metadata())
    pointing_duration_min:        float = field(default=0.0, metadata=FieldMeta("f", "Pointing duration",                 "min",      group=_PT, min_val=0.0,     max_val=300.0  ).as_metadata())
