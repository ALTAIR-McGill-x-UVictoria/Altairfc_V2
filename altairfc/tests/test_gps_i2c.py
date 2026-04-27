import smbus2
import time
import sys

ADDR = 0x42
REG_BYTES_HI = 0xFD
REG_BYTES_LO = 0xFE
REG_DATA     = 0xFF
MAX_READ     = 32  # smbus2 block read limit

def check_device_present(bus):
    try:
        bus.read_byte(ADDR)
        print(f"[OK] Device responds at 0x{ADDR:02X}")
        return True
    except OSError:
        print(f"[FAIL] No device at 0x{ADDR:02X}")
        return False

def check_bytes_available(bus):
    try:
        hi = bus.read_byte_data(ADDR, REG_BYTES_HI)
        lo = bus.read_byte_data(ADDR, REG_BYTES_LO)
        count = (hi << 8) | lo
        print(f"[OK] DDC buffer registers readable — {count} bytes available")
        return True
    except OSError as e:
        print(f"[FAIL] Could not read DDC buffer registers: {e}")
        return False

def read_available(bus):
    """Read all bytes from the DDC buffer in 32-byte chunks."""
    hi = bus.read_byte_data(ADDR, REG_BYTES_HI)
    lo = bus.read_byte_data(ADDR, REG_BYTES_LO)
    count = (hi << 8) | lo
    if count == 0:
        return []
    data = []
    while count > 0:
        chunk = min(count, MAX_READ)
        data += bus.read_i2c_block_data(ADDR, REG_DATA, chunk)
        count -= chunk
    return data

def check_ubx_response(bus):
    length = 0
    ck_a, ck_b = 0, 0
    for b in [0x0A, 0x04, 0x00, 0x00]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF

    frame = [0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, ck_a, ck_b]

    try:
        bus.write_i2c_block_data(ADDR, 0xFF, frame)
        print(f"[OK] UBX-MON-VER poll sent")
    except OSError as e:
        print(f"[FAIL] Could not send UBX-MON-VER poll: {e}")
        return False

    time.sleep(0.5)

    data = read_available(bus)

    if not data:
        print(f"[FAIL] No response after UBX-MON-VER poll")
        return False

    for i in range(len(data) - 1):
        if data[i] == 0xB5 and data[i+1] == 0x62:
            print(f"[OK] UBX sync pattern found — confirmed u-blox module")
            if len(data) > i + 4 and data[i+2] == 0x0A and data[i+3] == 0x04:
                payload_start = i + 6
                ver_bytes = data[payload_start:payload_start+30]
                ver_str = ''.join(chr(b) for b in ver_bytes if 32 <= b < 127)
                if ver_str:
                    print(f"[OK] Firmware version: {ver_str.strip()}")
            return True

    print(f"[FAIL] No UBX sync pattern in response")
    print(f"       Raw bytes: {[hex(b) for b in data]}")
    return False

def main():
    print(f"=== MAX-M10M verification at 0x{ADDR:02X} ===\n")

    try:
        bus = smbus2.SMBus(1)
    except Exception as e:
        print(f"[FAIL] Could not open I2C bus: {e}")
        sys.exit(1)

    results = []
    results.append(check_device_present(bus))
    if not results[-1]:
        print("\nDevice not found, aborting.")
        bus.close()
        sys.exit(1)

    results.append(check_bytes_available(bus))
    results.append(check_ubx_response(bus))

    bus.close()

    print(f"\n=== Results: {sum(results)}/{len(results)} checks passed ===")
    if all(results):
        print("Device confirmed as u-blox MAX-M10M (or compatible u-blox module)")
    else:
        print("Device at 0x42 did not fully verify as MAX-M10M")
    sys.exit(0 if all(results) else 1)

if __name__ == "__main__":
    main()
