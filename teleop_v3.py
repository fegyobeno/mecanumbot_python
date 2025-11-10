import serial
import struct
import msvcrt
import threading
import time
import math

# --- Constants ---
DEVICENAME = 'COM3'
BAUDRATE = 57600

MECANUMBOT_MIN_CAM_POS = 200
MECANUMBOT_MAX_CAM_POS = 860
MECANUMBOT_CLOSED_GRIPPER_POS = 350
MECANUMBOT_FRONT_GRIPPER_POS = 512
MECANUMBOT_OPEN_GRIPPER_POS = 700
SPEED = 100

# control values (7 shorts)
wheel_vel1 = 0
wheel_vel2 = 0
wheel_vel3 = 0
wheel_vel4 = 0
ax_pos1 = (MECANUMBOT_MAX_CAM_POS - MECANUMBOT_MIN_CAM_POS) // 2
ax_pos2 = MECANUMBOT_CLOSED_GRIPPER_POS
ax_pos3 = MECANUMBOT_CLOSED_GRIPPER_POS

# Packet layout (matches Arduino changes)
PAYLOAD_FMT = '<7h14f'            # 7 int16, 14 floats
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)  # should be 70
MAGIC = b'\x55\xAA'               # little-endian 0xAA55
SEQ_SIZE = 1
CRC_SIZE = 1
FULL_PACKET_SIZE = len(MAGIC) + SEQ_SIZE + PAYLOAD_SIZE + CRC_SIZE  # 74

# --- helper functions ---
def crc8_ccitt(data: bytes) -> int:
    """Match Arduino crc8_ccitt implementation (poly 0x07, MSB-first)."""
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0x07
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF

def getch():
    ch_bytes = msvcrt.getch()
    try:
        return ch_bytes.decode('utf-8')
    except UnicodeDecodeError:
        return ch_bytes

# Plausibility thresholds - tune for your robot
MAX_WHEEL_SPEED = 10000   # large enough but will filter extreme garbage
MIN_POS = -1000           # Theoretically impossible
MAX_POS = 10000           # Theoretically impossible
MAX_FLOAT_ABS = 1e7       # reject obvious garbage floats

def plausible_payload(vals):
    # vals: tuple of 7 shorts then 14 floats
    shorts = vals[:7]
    floats = vals[7:]
    # wheel velocities check
    for v in shorts[:4]:
        if abs(v) > MAX_WHEEL_SPEED:
            return False
    # positions check
    for p in shorts[4:7]:
        if p < MIN_POS or p > MAX_POS:
            return False
    # floats check
    for f in floats:
        if math.isnan(f) or math.isinf(f) or abs(f) > MAX_FLOAT_ABS:
            return False
    return True

def read_thread_fn(ser):
    buf = bytearray()
    while True:
        try:
            # read some bytes (blocking read with timeout)
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)

            # avoid buffer growing too large
            if len(buf) > 4096:
                # keep last 2048 bytes as a fallback
                buf = buf[-2048:]

            # search for magic header
            idx = buf.find(MAGIC)
            if idx == -1:
                # not found yet, continue reading
                time.sleep(0.001)
                continue

            # If found but not enough bytes for full packet yet, continue reading
            if len(buf) - idx < FULL_PACKET_SIZE:
                time.sleep(0.001)
                continue

            # we have at least one full candidate packet
            start = idx
            end = start + FULL_PACKET_SIZE
            packet = bytes(buf[start:end])

            # extract components
            seq = packet[len(MAGIC)]
            payload_bytes = packet[len(MAGIC) + SEQ_SIZE : len(MAGIC) + SEQ_SIZE + PAYLOAD_SIZE]
            recv_crc = packet[-1]

            # compute CRC over all but last byte (including magic and seq and payload)
            computed_crc = crc8_ccitt(packet[:-1])

            if computed_crc != recv_crc:
                # CRC mismatch -> drop this magic occurrence and continue searching
                # drop just the first byte of the current search window to resync
                del buf[start]
                continue

            # CRC ok -> unpack payload
            try:
                vals = struct.unpack(PAYLOAD_FMT, payload_bytes)
            except struct.error:
                # something wrong with size/format; drop header and resync
                del buf[start]
                continue

            if not plausible_payload(vals):
                # payload fails heuristic plausibility -> drop header and resync
                del buf[start]
                continue

            # Valid packet: consume bytes up to end
            del buf[:end]

            # Process/display values
            shorts = vals[:7]
            floats = vals[7:]
            print(f"\n[ROBOT] seq={seq} shorts={shorts} floats_head={floats} ...")
            # You can map them into names here
            # e.g.: vel_BL, vel_BR, vel_FL, vel_FR, pos_N, pos_GL, pos_GR = shorts

        except serial.SerialException as e:
            print(f"Serial error in read thread: {e}")
            time.sleep(0.5)
            continue
        except Exception as e:
            print(f"Unexpected error in read thread: {e}")
            time.sleep(0.1)
            continue
        finally:
            if not ser.is_open:
                print("Serial closed, exiting read thread.")
                break

# ---------------- main program ----------------
ser = None
reader = None
try:
    ser = serial.Serial(DEVICENAME, BAUDRATE, timeout=0.1)
    print("Serial port opened successfully.")
    reader = threading.Thread(target=read_thread_fn, args=(ser,), daemon=True)
    reader.start()
    print("Serial read thread started.")
    print("Press 'a','s','d','w','e','q','i','k','j','l' for control, space to stop, Esc to quit.")
    print("Press 1-10, t - TURBOOOO to controll speed")

    while True:
        key = getch()
        if key == '\x1b':  # Escape
            print("Escape pressed. Quitting...")
            wheel_vel1 = wheel_vel2 = wheel_vel3 = wheel_vel4 = 0
            fmt = '<7h'
            message_bytes = struct.pack(fmt, wheel_vel1, wheel_vel2, wheel_vel3, wheel_vel4,
                                        ax_pos1, ax_pos2, ax_pos3)
            ser.write(message_bytes)
            break
        elif key == ' ':
            wheel_vel1 = wheel_vel2 = wheel_vel3 = wheel_vel4 = 0
        elif key == 'w':
            wheel_vel1 = wheel_vel2 = wheel_vel3 = wheel_vel4 = SPEED
        elif key == 's':
            wheel_vel1 = wheel_vel2 = wheel_vel3 = wheel_vel4 = -SPEED
        elif key == 'd':
            wheel_vel1 = SPEED; wheel_vel2 = -SPEED; wheel_vel3 = -SPEED; wheel_vel4 = SPEED
        elif key == 'a':
            wheel_vel1 = -SPEED; wheel_vel2 = SPEED; wheel_vel3 = SPEED; wheel_vel4 = -SPEED
        elif key == 'q':
            wheel_vel1 = -SPEED; wheel_vel2 = SPEED; wheel_vel3 = -SPEED; wheel_vel4 = SPEED
        elif key == 'e':
            wheel_vel1 = SPEED; wheel_vel2 = -SPEED; wheel_vel3 = SPEED; wheel_vel4 = -SPEED
        elif key == 'i':
            ax_pos1 = MECANUMBOT_MAX_CAM_POS
        elif key == 'k':
            ax_pos1 = MECANUMBOT_MIN_CAM_POS
        elif key == 'j':
            ax_pos2 = MECANUMBOT_CLOSED_GRIPPER_POS
            ax_pos3 = MECANUMBOT_OPEN_GRIPPER_POS
        elif key == 'l':
            ax_pos2 = MECANUMBOT_OPEN_GRIPPER_POS
            ax_pos3 = MECANUMBOT_CLOSED_GRIPPER_POS
        elif key in '1234567890':
            num = int(key)
            if num == 0:
                SPEED = 100
            else:
                SPEED = num * 10
            print(f"Speed set to {SPEED}")
        elif key == 't':
            SPEED = 250
            print(f"Speed set to {SPEED}")

        fmt = '<7h'
        message_bytes = struct.pack(fmt,
                                    int(wheel_vel1), int(wheel_vel2), int(wheel_vel3), int(wheel_vel4),
                                    int(ax_pos1), int(ax_pos2), int(ax_pos3))
        ser.write(message_bytes)
        time.sleep(0.02)

except Exception as e:
    print(f"Error: {e}")
finally:
    if ser is not None:
        ser.close()
        print("Serial port closed.")
    if reader is not None:
        reader.join(timeout=1.0)