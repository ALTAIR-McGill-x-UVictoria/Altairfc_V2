import smbus2
import time
import sys

IODIRA = 0x00
IODIRB = 0x01
GPIOA  = 0x12
GPIOB  = 0x13

DEFAULT_ADDR = 0x24
INTERVAL = 0.5

def parse_args():
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print(f"Usage: {sys.argv[0]} <pin 0-15> [i2c_address]")
        print(f"  pin 0-7  = GPA0-GPA7")
        print(f"  pin 8-15 = GPB0-GPB7")
        sys.exit(1)

    try:
        pin = int(sys.argv[1])
    except ValueError:
        print(f"Error: '{sys.argv[1]}' is not a valid pin number")
        sys.exit(1)

    if not 0 <= pin <= 15:
        print(f"Error: pin must be 0-15")
        sys.exit(1)

    addr = DEFAULT_ADDR
    if len(sys.argv) == 3:
        try:
            addr = int(sys.argv[2], 0)
        except ValueError:
            print(f"Error: '{sys.argv[2]}' is not a valid I2C address")
            sys.exit(1)

    return pin, addr

def main():
    pin, addr = parse_args()

    port  = 0 if pin < 8 else 1
    bit   = pin % 8
    mask  = 1 << bit
    iodir = IODIRA if port == 0 else IODIRB
    gpio  = GPIOA  if port == 0 else GPIOB
    port_name = "A" if port == 0 else "B"

    bus = smbus2.SMBus(1)

    current_dir = bus.read_byte_data(addr, iodir)
    bus.write_byte_data(addr, iodir, current_dir & ~mask)

    print(f"Blinking MCP23017 GP{port_name}{bit} (pin {pin}) at 0x{addr:02X}, Ctrl+C to stop")

    state = 0
    try:
        while True:
            state ^= mask
            current = bus.read_byte_data(addr, gpio)
            bus.write_byte_data(addr, gpio, (current & ~mask) | state)
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        current = bus.read_byte_data(addr, gpio)
        bus.write_byte_data(addr, gpio, current & ~mask)
        bus.close()
        print("Done")

if __name__ == "__main__":
    main()
