import time
import argparse
import struct
import sys
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Default serial device selection (Windows vs POSIX)
import os
if os.name == 'nt':
    DEFAULT_DEVICE = 'COM3'
else:
    DEFAULT_DEVICE = '/dev/ttyACM0'

# Control table configuration: (name, addr, size_bytes, signed, is_float)
# IMPORTANT: These are educated guesses. Adjust sizes/types for your firmware/control table.
CONTROL_TABLE = [
    ("ADDR_MODEL_INFORM", 2, 4, False, False),  # uint32_t

    ("ADDR_MILLIS", 10, 4, False, False),  # uint32_t

    ("ADDR_DEBUG_MODE", 14, 1, False, False),  # bool
    ("ADDR_CONNECT_ROS2", 15, 1, False, False),  # bool
    ("ADDR_CONNECT_MANIP", 16, 1, False, False),  # bool

    ("ADDR_DEVICE_STATUS", 18, 1, True, False),  # int8_t
    ("ADDR_HEARTBEAT", 19, 1, False, False),  # uint8_t

    ("ADDR_USER_LED_1", 20, 1, False, False),
    ("ADDR_USER_LED_2", 21, 1, False, False),
    ("ADDR_USER_LED_3", 22, 1, False, False),
    ("ADDR_USER_LED_4", 23, 1, False, False),

    ("ADDR_BUTTON_1", 26, 1, False, False),
    ("ADDR_BUTTON_2", 27, 1, False, False),
    ("ADDR_BUMPER_1", 28, 1, False, False),
    ("ADDR_BUMPER_2", 29, 1, False, False),

    ("ADDR_ILLUMINATION", 30, 2, False, False),  # uint16_t
    ("ADDR_IR", 34, 4, False, False),             # uint32_t
    ("ADDR_SORNA", 38, 4, True, True),            # float

    ("ADDR_BATTERY_VOLTAGE", 42, 4, False, False),  # uint32_t (x100)
    ("ADDR_BATTERY_PERCENT", 46, 4, False, False),  # uint32_t (x100)

    ("ADDR_SOUND", 50, 1, False, False),  # uint8_t

    ("ADDR_IMU_RECALIBRATION", 59, 1, False, False),  # bool
    ("ADDR_ANGULAR_VELOCITY_X", 60, 4, True, True),
    ("ADDR_ANGULAR_VELOCITY_Y", 64, 4, True, True),
    ("ADDR_ANGULAR_VELOCITY_Z", 68, 4, True, True),
    ("ADDR_LINEAR_ACC_X", 72, 4, True, True),
    ("ADDR_LINEAR_ACC_Y", 76, 4, True, True),
    ("ADDR_LINEAR_ACC_Z", 80, 4, True, True),
    ("ADDR_MAGNETIC_X", 84, 4, True, True),
    ("ADDR_MAGNETIC_Y", 88, 4, True, True),
    ("ADDR_MAGNETIC_Z", 92, 4, True, True),
    ("ADDR_ORIENTATION_W", 96, 4, True, True),
    ("ADDR_ORIENTATION_X", 100, 4, True, True),
    ("ADDR_ORIENTATION_Y", 104, 4, True, True),
    ("ADDR_ORIENTATION_Z", 108, 4, True, True),

    # Present currents/velocities/positions (int32_t), order: BL, BR, FL, FR
    ("ADDR_PRESENT_CURRENT_BL", 120, 4, True, False),
    ("ADDR_PRESENT_CURRENT_BR", 124, 4, True, False),
    ("ADDR_PRESENT_CURRENT_FL", 128, 4, True, False),
    ("ADDR_PRESENT_CURRENT_FR", 132, 4, True, False),
    ("ADDR_PRESENT_VELOCITY_BL", 136, 4, True, False),
    ("ADDR_PRESENT_VELOCITY_BR", 140, 4, True, False),
    ("ADDR_PRESENT_VELOCITY_FL", 144, 4, True, False),
    ("ADDR_PRESENT_VELOCITY_FR", 148, 4, True, False),
    ("ADDR_PRESENT_POSITION_BL", 152, 4, True, False),
    ("ADDR_PRESENT_POSITION_BR", 156, 4, True, False),
    ("ADDR_PRESENT_POSITION_FL", 160, 4, True, False),
    ("ADDR_PRESENT_POSITION_FR", 164, 4, True, False),

    ("ADDR_MOTOR_CONNECT", 168, 1, False, False),  # bool
    ("ADDR_MOTOR_TORQUE", 169, 1, False, False),   # bool
    ("ADDR_CMD_VEL_LINEAR_X", 170, 4, True, False),
    ("ADDR_CMD_VEL_LINEAR_Y", 174, 4, True, False),
    ("ADDR_CMD_VEL_LINEAR_Z", 178, 4, True, False),
    ("ADDR_CMD_VEL_ANGULAR_X", 182, 4, True, False),
    ("ADDR_CMD_VEL_ANGULAR_Y", 186, 4, True, False),
    ("ADDR_CMD_VEL_ANGULAR_Z", 190, 4, True, False),
    ("ADDR_PROFILE_ACC_BL", 194, 4, False, False),
    ("ADDR_PROFILE_ACC_BR", 198, 4, False, False),
    ("ADDR_PROFILE_ACC_FL", 202, 4, False, False),
    ("ADDR_PROFILE_ACC_FR", 206, 4, False, False),
    
    # AX (Protocol 1.0) window mapped as 32-bit values for goals/present
    ("AX_ADDR_TORQUE", 210, 1, False, False),  # bool
    ("AX_ADDR_NECK_GOAL", 214, 4, True, False),
    ("AX_ADDR_GRABBER_LEFT_GOAL", 218, 4, True, False),
    ("AX_ADDR_GRABBER_RIGHT_GOAL", 222, 4, True, False),

    ("AX_ADDR_PRESENT_NECK_POSITION", 226, 4, False, False),
    ("AX_ADDR_PRESENT_GRABBER_LEFT_POSITION", 230, 4, False, False),
    ("AX_ADDR_PRESENT_GRABBER_RIGHT_POSITION", 234, 4, False, False),
]

# Helper read functions using PacketHandler

def read_value(packetHandler, portHandler, dxl_id, addr, size):
    """Read `size` bytes from dxl_id at addr. Returns raw integer or raises an exception."""
    if size == 1:
        val, comm_result, err = packetHandler.read1ByteTxRx(portHandler, dxl_id, addr)
    elif size == 2:
        val, comm_result, err = packetHandler.read2ByteTxRx(portHandler, dxl_id, addr)
    elif size == 4:
        val, comm_result, err = packetHandler.read4ByteTxRx(portHandler, dxl_id, addr)
    else:
        # For nonstandard sizes, read multiple 1-byte values and compose little-endian
        val = 0
        comm_result = COMM_SUCCESS
        err = 0
        for i in range(size):
            b, cr, e = packetHandler.read1ByteTxRx(portHandler, dxl_id, addr + i)
            if cr != COMM_SUCCESS:
                comm_result = cr
            if e != 0:
                err = e
            val |= (b & 0xFF) << (8 * i)

    if comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Comm error reading addr {addr}: {comm_result}")
    if err != 0:
        # Device error reported; still return value but warn
        print(f"Warning: device error at addr {addr}: {err}")
    return val


def interpret_raw(raw, size, signed=False, is_float=False):
    """Convert raw integer read from device into python type depending on flags."""
    if is_float and size == 4:
        # treat raw as uint32 raw bits and reinterpret as float32 little-endian
        b = struct.pack('<I', raw & 0xFFFFFFFF)
        return struct.unpack('<f', b)[0]
    else:
        # signed vs unsigned integer
        if signed:
            # convert from two's complement
            sign_bit = 1 << (size * 8 - 1)
            mask = (1 << (size * 8)) - 1
            raw &= mask
            return (raw ^ sign_bit) - sign_bit
        else:
            return raw


def build_parser():
    p = argparse.ArgumentParser(description='Monitor OpenCR control table values')
    p.add_argument('--device', default=DEFAULT_DEVICE, help='Serial device (COMx or /dev/tty...)')
    p.add_argument('--baud', type=int, default=115200, help='Baudrate')
    p.add_argument('--id', type=int, default=200, help='Dynamixel/OpenCR ID')
    p.add_argument('--interval', type=float, default=1.0, help='Poll interval in seconds')
    p.add_argument('--timeout', type=float, default=0.5, help='Port timeout seconds')
    p.add_argument('--only', nargs='*', help='Optional list of address names to monitor (e.g. ADDR_BATTERY_VOLTAGE)')
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    portHandler = PortHandler(args.device)
    packetHandler = PacketHandler(2.0)

    if not portHandler.openPort():
        print(f"Failed to open port {args.device}")
        sys.exit(1)
    if not portHandler.setBaudRate(args.baud):
        print(f"Failed to set baudrate {args.baud}")
        sys.exit(1)

    # Filter entries if user specified --only
    if args.only:
        watch = [e for e in CONTROL_TABLE if e[0] in args.only]
        if not watch:
            print("No matching control table entries found for --only names")
            sys.exit(1)
    else:
        watch = CONTROL_TABLE

    print(f"Monitoring {len(watch)} entries from {args.device} at {args.baud} baud. Press Ctrl-C to stop.")

    try:
        while True:
            timestamp = time.time()
            out = { 'ts': timestamp }
            for name, addr, size, signed, is_float in watch:
                try:
                    raw = read_value(packetHandler, portHandler, args.id, addr, size)
                    val = interpret_raw(raw, size, signed=signed, is_float=is_float)
                except Exception as e:
                    val = f"ERR: {e}"
                out[name] = val

            # Nicely format output: timestamp and key=value pairs
            ts_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(out['ts']))
            line = [ts_str]
            for name in out:
                if name == 'ts':
                    continue
                line.append(f"{name}={out[name]}\n")
            print(' | '.join(line))
            print("---------------------------------------------------------")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        portHandler.closePort()


if __name__ == '__main__':
    main()