from time import sleep
import sys
from dynamixel_sdk import *
from dynamixel_sdk import COMM_SUCCESS
import termios
import tty

# Control table address and protocol version
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 115200
#BAUDRATE = 1000000
#BAUDRATE=57600
#BAUDRATE=9600
DXL_ID = int(sys.argv[1]) if len(sys.argv) > 1 else 200  # Dynamixel ID

ADDR = int(sys.argv[2]) if len(sys.argv) > 2 else 169  # Change this to your desired address

WRITE = int(sys.argv[3]) if len(sys.argv) > 3 else -1

BYTE_SIZE = int(sys.argv[4]) if len(sys.argv)>4 else 1

SPEED = 20

def read(id, addr, BYTE_SIZE=BYTE_SIZE):
    if BYTE_SIZE == 1:
        value = packetHandler.read1ByteTxRx(portHandler, id, addr)
    if BYTE_SIZE == 4:
        value = packetHandler.read4ByteTxRx(portHandler, id, addr)
    result = value[0]
    dxl_comm_result = value[1]
    dxl_error = value[2]

    if dxl_comm_result == COMM_SUCCESS:
        print(f'Read from address {ADDR} (ID {id}): {result}')
        if dxl_error != 0:
            print(f'Device returned error: {dxl_error}')
    else:
        print(f'Read failed. Communication result: {dxl_comm_result}')

def write(id, addr, value, BYTE_SIZE=BYTE_SIZE):
    if BYTE_SIZE ==1:
       dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, addr, value)
    if BYTE_SIZE==4:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, addr, value)
    if dxl_comm_result == COMM_SUCCESS:
        print(f'Wrote {value} to address {addr} (ID {id})')
        if dxl_error != 0:
            print(f'Device returned error: {dxl_error}')
    else:
        print(f'Write failed. Communication result: {dxl_comm_result}')

# Open port
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

    print(f'Using ID: {DXL_ID}, Address: {ADDR}, Write: {WRITE}, Byte Size: {BYTE_SIZE}')
    print("Enable debug mode") # python3 read.py 200 14 1 1
    write(DXL_ID, 16, 1, 4)  # Enable debug mode

    print("set torque")
    write(DXL_ID, 169, 1, 1)  # Set torque enable

def end():
    print("Disable debug mode")
    write(DXL_ID, 16, 0, 4)  # Disable debug mode
    print("Disable torque")
    write(DXL_ID, 169, 0, 1)  # Set torque disable

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
        # Initialize PortHandler and PacketHandler
        portHandler = PortHandler(DEVICENAME)
        packetHandler = PacketHandler(PROTOCOL_VERSION)

        start()

        print("Press 'a', 's', 'd', 'w', 'q', 'e' for input, 'x' to quit.")
        while True:
            key = getch()
            if key == '\x1b':  # Escape key
                print("Escape pressed. Quitting...")
                break
            elif key == ' ':
                print("Space pressed. Stopping all motors.")
                write(DXL_ID, 170, 0, 4)
                write(DXL_ID, 174, 0, 4)
                write(DXL_ID, 182, 0, 4)
                write(DXL_ID, 186, 0, 4)
            if key == '':
                print("Quitting...")
                break
            elif key in ['a', 's', 'd', 'w', 'e', 'q']:
                print(f"Key pressed: {key}")
                # You can add code here to handle each key as needed

                if key == 'w':
                    write(DXL_ID, 170, SPEED, 4)
                    #pass

                elif key == 's':
                    write(DXL_ID, 170, -SPEED, 4)
                    #pass

                elif key == 'd':
                    #pass
                    write(DXL_ID, 174, SPEED, 4)

                elif key == 'a':
                    write(DXL_ID, 174, -SPEED, 4)

                elif key == 'q':
                    #pass
                    write(DXL_ID, 182, SPEED, 4)
                elif key == 'e':
                    #pass
                    write(DXL_ID, 182, -SPEED, 4)
                    #pass


    except Exception as e:
        print(f"Error: {e}")
    finally:
        end()
        portHandler.closePort()

if __name__ == '__main__':
    main()