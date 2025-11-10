import serial
import struct
import sys
# --- Constants ---
DEVICENAME = 'COM3'
BAUDRATE = 57600

MECANUMBOT_MIN_CAM_POS = 200

MECANUMBOT_MAX_CAM_POS = 860

MECANUMBOT_CLOSED_GRIPPER_POS = 350

MECANUMBOT_FRONT_GRIPPER_POS = 512

MECANUMBOT_OPEN_GRIPPER_POS = 700

SPEED = 20

# --- Your 7 values ---
wheel_vel1 = int(sys.argv[1])  # Get from command line argument
wheel_vel2 = int(sys.argv[2])  # Get from command line argument
wheel_vel3 = int(sys.argv[3])  # Get from command line argument
wheel_vel4 = int(sys.argv[4])  # Get from command line argument
ax_pos1 = int(sys.argv[5])  # Get from command line argument
ax_pos2 = int(sys.argv[6])  # Get from command line argument
ax_pos3 = int(sys.argv[7])  # Get from command line argument
# ---------------------

try:
    ser = serial.Serial('COM3', 57600, timeout=1)
    
    # 'h' stands for a 16-bit signed integer (short)
    # We have 7 of them, so the format is 'hhhhhhh'
    # The '<' means little-endian byte order (standard for Arduino/STM32)
    format_string = '<hhhhhhh'
    
    # Pack the 7 values into a binary byte string
    # This will be 7 * 2 = 14 bytes long
    message_bytes = struct.pack(format_string, 
                                wheel_vel1, wheel_vel2, wheel_vel3, wheel_vel4,
                                ax_pos1, ax_pos2, ax_pos3)

    ser.write(message_bytes)
    
    print(f"Sent {len(message_bytes)} bytes.")
    ser.close()

except Exception as e:
    print(f"Error: {e}")