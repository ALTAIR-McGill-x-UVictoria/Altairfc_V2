# Ground Station Command Protocol

This document describes how the ground station should form and send command packets to the ALTAIR V2 flight computer over the LR-900p radio link.

---

## Wire Frame Format

All frames — both telemetry (FC→GS) and commands (GS→FC) — share the same structure:

```
 Offset  Size  Type        Field
 ------  ----  ----------  -----
      0     1  uint8       SYNC (always 0xAA)
      1     1  uint8       CMD_ID
      2     1  uint8       SEQ
      3     8  float64 LE  TIMESTAMP (Unix epoch, seconds)
     11     2  uint16 LE   LEN (payload byte count)
     13   LEN  bytes       PAYLOAD
  13+LEN    2  uint16 LE   CRC16
```

**Total frame size:** 13 + LEN + 2 bytes.

### CRC

CRC-16/CCITT using Python's `binascii.crc_hqx` with initial value `0xFFFF`.

The CRC is computed over every byte **after** SYNC and **before** the CRC field itself:

```
crc_input = frame[1 : 13 + LEN]   # CMD_ID through end of payload
crc = binascii.crc_hqx(crc_input, 0xFFFF)
```

### Encoding a frame (Python)

```python
import binascii, struct, time

_HEADER  = struct.Struct("<BBBdH")   # sync, cmd_id, seq, timestamp, length
_CRC_FMT = struct.Struct("<H")

def build_command_frame(cmd_id: int, payload: bytes, seq: int) -> bytes:
    header = _HEADER.pack(0xAA, cmd_id, seq & 0xFF, time.time(), len(payload))
    crc    = binascii.crc_hqx(header[1:] + payload, 0xFFFF)
    return header + payload + _CRC_FMT.pack(crc)
```

---

## Command Catalogue

### `0xC0` — ARM

Arms the flight computer. `FlightStageTask` latches this on its next cycle and advances to STAGE_ARMED.

| Field | Type | Value |
|---|---|---|
| `arm_state` | uint8 | `0x01` |

**Payload:** `struct.pack("<B", 1)` → 1 byte

**Precondition:** Any stage (typically PREFLIGHT).

---

### `0xC1` — LAUNCH_OK

Clears the launch inhibit. `FlightStageTask` only accepts this in STAGE_ARMED; it is rejected (ACK status = 1) in any other stage.

| Field | Type | Value |
|---|---|---|
| `launch_ok` | uint8 | `0x01` |

**Payload:** `struct.pack("<B", 1)` → 1 byte

**Precondition:** `event.flight_stage == 1` (ARMED). The FC will reject and ACK with status `1` otherwise.

---

### `0xC2` — PING

Link health check. The FC sends an ACK immediately with no DataStore side-effect.

| Field | Type | Value |
|---|---|---|
| `ping` | uint8 | `0x01` |

**Payload:** `struct.pack("<B", 1)` → 1 byte

---

### `0xC3` — UPDATE_SETTING

Updates one flight parameter at runtime. Takes effect on the next `FlightStageTask`, `RWTask`, or `MMTask` execution cycle without a reboot.

| Field | Type | Description |
|---|---|---|
| `field_id` | uint8 | Index into the setting table below |
| `value` | float32 LE | New value in the field's native units |

**Payload:** `struct.pack("<Bf", field_id, value)` → 5 bytes

#### Setting field IDs

| ID | Setting key | Units | Notes |
|---|---|---|---|
| 0 | `termination_altitude_m` | m | Altitude at which cutdown fires |
| 1 | `burst_altitude_m` | m | Expected natural burst altitude |
| 2 | `burst_altitude_uncertainty_m` | m | ± detection window around burst altitude |
| 3 | `ascent_detect_window_s` | s | Rolling window for ascent confirmation |
| 4 | `ascent_detect_gain_m` | m | Altitude gain over window to confirm ascent |
| 5 | `apogee_fraction` | fraction (0–1) | Descent declared when alt ≤ fraction × apogee |
| 6 | `landing_fraction` | fraction (0–1) | Landing declared when alt ≤ fraction × apogee |
| 7 | `recovery_stationary_s` | s | Seconds stationary (±2 m band) → recovery |
| 8 | `termination_confirm_drop_m` | m | Required altitude drop to confirm cutdown |
| 9 | `termination_confirm_window_s` | s | Time window for cutdown confirmation |
| 10 | `rw_kp` | RPM / (rad/s) | Reaction wheel proportional gain |
| 11 | `rw_kd` | RPM / (rad/s²) | Reaction wheel derivative gain |
| 12 | `rw_max_rpm` | RPM | Reaction wheel output clamp |
| 13 | `mm_kp` | A / (rad/s) | Momentum management proportional gain |
| 14 | `mm_kd` | A / (rad/s²) | Momentum management derivative gain |
| 15 | `mm_max_current` | mA | Momentum management output clamp |

The FC echoes the updated values back in the next `FlightSettingsPacket` (ID `0x09`), so the GS can verify the change took effect.

**Out-of-range `field_id`** (> 15): FC sends ACK with status `1` (rejected) and ignores the command.

---

## Sequence Counter

The GS maintains its own command sequence counter, separate from the FC's telemetry sequence counters. Increment it for every frame sent and wrap at 255 → 0.

The FC echoes `cmd_seq` back in the ACK so the GS can match ACKs to the commands that triggered them.

---

## ACK Packet (FC→GS, ID `0xA0`)

After receiving any command, the FC transmits an ACK frame on the same radio link.

| Field | Type | Description |
|---|---|---|
| `cmd_id` | uint8 | ID of the command being acknowledged |
| `cmd_seq` | uint8 | SEQ echoed from the command header |
| `status` | uint8 | `0` = accepted, `1` = rejected |

**Payload size:** 3 bytes. Full frame size: 18 bytes.

The GS should listen for this ACK within a reasonable timeout (recommend 2 s given the half-duplex radio). If no ACK is received, retransmit with the same `seq` and compare the echoed `cmd_seq` in the response to detect duplicates.

---

## Complete Python Example

```python
import binascii, serial, struct, time

_HEADER  = struct.Struct("<BBBdH")
_CRC_FMT = struct.Struct("<H")
_seq     = 0

def _build(cmd_id: int, payload: bytes) -> bytes:
    global _seq
    header = _HEADER.pack(0xAA, cmd_id, _seq & 0xFF, time.time(), len(payload))
    crc    = binascii.crc_hqx(header[1:] + payload, 0xFFFF)
    _seq   = (_seq + 1) & 0xFF
    return header + payload + _CRC_FMT.pack(crc)

ser = serial.Serial("COM5", 57600, timeout=2.0)

# Arm the flight computer
ser.write(_build(0xC0, struct.pack("<B", 1)))

# Lower the cutdown altitude to 20 km (field_id=0)
ser.write(_build(0xC3, struct.pack("<Bf", 0, 20000.0)))

# Tune reaction wheel Kp (field_id=10)
ser.write(_build(0xC3, struct.pack("<Bf", 10, 15000.0)))
```

The same helper is available in `ground/receiver.py` as `send_settings_update_command(ser, field_id, value)`.
