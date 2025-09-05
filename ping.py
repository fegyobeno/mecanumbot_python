from dynamixel_sdk import *

# Control table address and protocol version
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 115200
DXL_ID = 200

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print(f'Succeeded to open the port!\n{PROTOCOL_VERSION}|{DEVICENAME}')
else:
    print('Failed to open the port.')
    exit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print('Succeeded to change the baudrate!')
else:
    print('Failed to change the baudrate.')
    exit()

dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)

print(dxl_model_number, dxl_comm_result, dxl_error)

if dxl_comm_result == COMM_SUCCESS:
    print(f'Successfully pinged ID {DXL_ID}. Model number: {dxl_model_number}')
    if dxl_error != 0:
        print(f'Device returned error: {dxl_error}')
else:
    print(f'Ping failed. Communication result: {dxl_comm_result}')

# Close port
portHandler.closePort()