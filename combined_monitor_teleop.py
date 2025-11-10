from time import sleep
import sys
from dynamixel_sdk import *
from dynamixel_sdk import COMM_SUCCESS
import termios
import tty
import threading
from addr_info import AddrInfo

# --------------- DYNAMIXEL SETUP ----------------
# Control table address and protocol version
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
DXL_ID = 200

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

# ---------------- ADDRESS INFO ----------------------
MEM_ADDR = {
    "DEBUG_MODE": AddrInfo(14, 1),
    "XM_TORQUE_ENABLE": AddrInfo(169, 1),
    "ADDR_X_VEL": AddrInfo(170, 4),
    "ADDR_Y_VEL": AddrInfo(174, 4),
    "ADDR_Z_ANG": AddrInfo(190, 4),
    "AX_TORQUE_ENABLE": AddrInfo(210, 4),
    "ADDR_CAM": AddrInfo(214, 4),
    "ADDR_GRIPPER_L": AddrInfo(218, 4),
    "ADDR_GRIPPER_R": AddrInfo(222, 4),
}

# ------------------- CONSTANTS -------------------------
MECANUMBOT_MIN_CAM_POS = 200

MECANUMBOT_MAX_CAM_POS = 860

MECANUMBOT_CLOSED_GRIPPER_POS = 350

MECANUMBOT_FRONT_GRIPPER_POS = 512

MECANUMBOT_OPEN_GRIPPER_POS = 700

SPEED = 20


def read(id: int, addr: int, BYTE_SIZE: int) -> int:
    if BYTE_SIZE == 1:
        value, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, id, addr)
    elif BYTE_SIZE == 2:
        value, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, id, addr)
    elif BYTE_SIZE == 4:
        value, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, addr)
    if dxl_comm_result == COMM_SUCCESS:
        print(f'Read {value} from address {addr} (ID {id})')
        if dxl_error != 0:
            print(f'Device returned error: {dxl_error}')
        return value
    else:
        print(f'Read failed. Communication result: {dxl_comm_result}')
        return None

def monitor_thread(portHandler, packetHandler, DXL_ID):
    try:
        while True:
            # Read some example values
            pos_x = read(DXL_ID, MEM_ADDR["ADDR_X_VEL"].addr, MEM_ADDR["ADDR_X_VEL"].bytesize)
            pos_y = read(DXL_ID, MEM_ADDR["ADDR_Y_VEL"].addr, MEM_ADDR["ADDR_Y_VEL"].bytesize)
            ang_z = read(DXL_ID, MEM_ADDR["ADDR_Z_ANG"].addr, MEM_ADDR["ADDR_Z_ANG"].bytesize)
            print(f"Current Velocities - X: {pos_x}, Y: {pos_y}, Z: {ang_z}")
            
    except Exception as e:
        print(f"Monitoring error: {e}")

"""
writes a value to a given byte address for a specified DYNAMIXEL ID and byte size
* id:        DYNAMIXEL ID
* addr:      memory address
* value:     value to write
* BYTE_SIZE: size of the value in bytes (1,2,4)
"""
def write(id: int, addr: int, value: int, BYTE_SIZE: int) -> None:
    if BYTE_SIZE == 1:
       dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, addr, value)
    if BYTE_SIZE == 2:
       dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, addr, value)
    if BYTE_SIZE == 4:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, addr, value)
    if dxl_comm_result == COMM_SUCCESS:
        print(f'Wrote {value} to address {addr} (ID {id})')
        if dxl_error != 0:
            print(f'Device returned error: {dxl_error}')
    else:
        print(f'Write failed. Communication result: {dxl_comm_result}')

"""
Starts the DYNAMIXEL communication
Opens the port, sets the baudrate, enables debug mode and torque
!!! Debug mode must be enabled, as otherwise the robot will not accept velocity commands !!!
"""
def start():
    if portHandler.openPort():
        print('Succeeded to open the port!')
    else:
        print('Failed to open the port.')
        exit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print('Succeeded to change the baudrate!')
    else:
        print('Failed to change the baudrate.')
        exit()

    print(f'Using ID: {DXL_ID}, Address: {MEM_ADDR}')
    print("Enable debug mode")
    write(DXL_ID, MEM_ADDR["DEBUG_MODE"].addr, 1, MEM_ADDR["DEBUG_MODE"].bytesize)  # Enable debug mode

    print("set XM torque")
    write(DXL_ID, MEM_ADDR["XM_TORQUE_ENABLE"].addr, 1, MEM_ADDR["XM_TORQUE_ENABLE"].bytesize)  # Set torque enable

    print("set AX torque")
    write(DXL_ID, MEM_ADDR["AX_TORQUE_ENABLE"].addr, 1, MEM_ADDR["AX_TORQUE_ENABLE"].bytesize)  # Set torque enable

"""
Ends the DYNAMIXEL communication
Disables debug mode and torque
"""
def end():
    print("Disable debug mode")
    write(DXL_ID, MEM_ADDR["DEBUG_MODE"].addr, 0, MEM_ADDR["DEBUG_MODE"].bytesize)  # Disable debug mode
    print("Disable torque")
    write(DXL_ID, MEM_ADDR["XM_TORQUE_ENABLE"].addr, 0, MEM_ADDR["XM_TORQUE_ENABLE"].bytesize)  # Set torque disable
    print("Disable AX torque")
    write(DXL_ID, MEM_ADDR["AX_TORQUE_ENABLE"].addr, 0, MEM_ADDR["AX_TORQUE_ENABLE"].bytesize)  # Set torque disable

"""
Gets a single character from standard input without echoing.
"""
def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main():
    try:
        global portHandler, packetHandler
        # Initialize PortHandler and PacketHandler
        portHandler = PortHandler(DEVICENAME)
        packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        monitor_thread_instance = threading.Thread(target=monitor_thread, args=(portHandler, packetHandler, DXL_ID))
        monitor_thread_instance.daemon = True
        monitor_thread_instance.start()
        

        start()

        print("Press 'a', 's', 'd', 'w', 'e', 'q', 'i', 'k', 'j', 'l' for input, space to stop, Esc to quit.")
        while True:
            # limits the users ability to one command per 0.1 second. Seems to solve the stack smashing issue 
            sleep(0.1) # sleep 
            key = getch()
            if key == '\x1b':  # Escape key
                print("Escape pressed. Quitting...")
                break
            elif key == ' ':
                print("Space pressed. Stopping all motors.")
                write(DXL_ID, MEM_ADDR["ADDR_X_VEL"].addr, 0, MEM_ADDR["ADDR_X_VEL"].bytesize)
                write(DXL_ID, MEM_ADDR["ADDR_Y_VEL"].addr, 0, MEM_ADDR["ADDR_Y_VEL"].bytesize)
                write(DXL_ID, MEM_ADDR["ADDR_Z_ANG"].addr, 0, MEM_ADDR["ADDR_Z_ANG"].bytesize)
            if key == '':
                print("Quitting...")
                break
            elif key in ['a', 's', 'd', 'w', 'e', 'q', 'i', 'k', 'j', 'l']:
                print(f"Key pressed: {key}")
                if key == 'w': #forward
                    write(DXL_ID, MEM_ADDR["ADDR_X_VEL"].addr, SPEED, MEM_ADDR["ADDR_X_VEL"].bytesize)
                elif key == 's': #backward
                    write(DXL_ID, MEM_ADDR["ADDR_X_VEL"].addr, -SPEED, MEM_ADDR["ADDR_X_VEL"].bytesize)
                elif key == 'd': #right
                    write(DXL_ID, MEM_ADDR["ADDR_Y_VEL"].addr, SPEED, MEM_ADDR["ADDR_Y_VEL"].bytesize)
                elif key == 'a': #left
                    write(DXL_ID, MEM_ADDR["ADDR_Y_VEL"].addr, -SPEED, MEM_ADDR["ADDR_Y_VEL"].bytesize)
                elif key == 'q': #turn left
                    write(DXL_ID, MEM_ADDR["ADDR_Z_ANG"].addr, SPEED, MEM_ADDR["ADDR_Z_ANG"].bytesize)
                elif key == 'e': #turn right
                    write(DXL_ID, MEM_ADDR["ADDR_Z_ANG"].addr, -SPEED, MEM_ADDR["ADDR_Z_ANG"].bytesize)
                elif key == 'i': #cam_up
                    write(DXL_ID, MEM_ADDR["ADDR_CAM"].addr, MECANUMBOT_MAX_CAM_POS, MEM_ADDR["ADDR_CAM"].bytesize)
                elif key == 'k': #cam_down
                    write(DXL_ID, MEM_ADDR["ADDR_CAM"].addr, MECANUMBOT_MIN_CAM_POS, MEM_ADDR["ADDR_CAM"].bytesize)
                elif key == 'j': #open gripper
                    write(DXL_ID, MEM_ADDR["ADDR_GRIPPER_L"].addr, MECANUMBOT_OPEN_GRIPPER_POS, MEM_ADDR["ADDR_GRIPPER_L"].bytesize)
                    write(DXL_ID, MEM_ADDR["ADDR_GRIPPER_R"].addr, MECANUMBOT_OPEN_GRIPPER_POS, MEM_ADDR["ADDR_GRIPPER_R"].bytesize)
                elif key == 'l': #close gripper,
                    write(DXL_ID, MEM_ADDR["ADDR_GRIPPER_L"].addr, MECANUMBOT_CLOSED_GRIPPER_POS, MEM_ADDR["ADDR_GRIPPER_L"].bytesize)
                    write(DXL_ID, MEM_ADDR["ADDR_GRIPPER_R"].addr, MECANUMBOT_CLOSED_GRIPPER_POS, MEM_ADDR["ADDR_GRIPPER_R"].bytesize)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        end()
        portHandler.closePort()
        monitor_thread_instance.join()

if __name__ == '__main__':
    main()