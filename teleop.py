from time import sleep
import sys
from dynamixel_sdk import *
from dynamixel_sdk import COMM_SUCCESS
import termios
import tty
from addr_info import AddrInfo

# --------------- DYNAMIXEL SETUP ----------------
# Control table address and protocol version
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
DXL_ID = 200

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

if __name__ == '__main__':
    main()