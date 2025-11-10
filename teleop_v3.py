import serial
import struct
import msvcrt
import threading
import time

# --- Constants ---
DEVICENAME = 'COM3'
BAUDRATE = 57600

MECANUMBOT_MIN_CAM_POS = 200

MECANUMBOT_MAX_CAM_POS = 860

MECANUMBOT_CLOSED_GRIPPER_POS = 350

MECANUMBOT_FRONT_GRIPPER_POS = 512

MECANUMBOT_OPEN_GRIPPER_POS = 700

SPEED = 100

# --- Your 7 values ---
wheel_vel1 = 0
wheel_vel2 = 0
wheel_vel3 = 0
wheel_vel4 = 0
ax_pos1 = 0
ax_pos2 = 0
ax_pos3 = 0
# ---------------------

"""
Gets a single character from standard input without echoing.
"""
# for Linux/Mac
# def getch():
#         fd = sys.stdin.fileno()
#         old_settings = termios.tcgetattr(fd)
#         try:
#             tty.setraw(fd)
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch
# for windows
def getch():
    """
    Gets a single character from the user on Windows
    without waiting for the Enter key.
    """
    ch_bytes = msvcrt.getch()
    try:
        # Try to decode as a standard character
        return ch_bytes.decode('utf-8')
    except UnicodeDecodeError:
        # If it fails, it's a special key (e.g., arrow key)
        # Return the raw bytes or a placeholder
        return ch_bytes

def read_from_serial(ser):
    while True:
        try:
            line_bytes = ser.read(14)
            #print(f"Raw bytes received: {line_bytes}")
            format_string = '<hhhhhhh'
        
            # Pack the 7 values into a binary byte string
            # This will be 7 * 2 = 14 bytes long
            decoded_message = struct.unpack(format_string, line_bytes)
                    
            # Print the message from the robot
            # \n at the start ensures it prints on a new line
            print(f"\n[ROBOT]: {decoded_message}")
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            continue
        except Exception as e:
            print(f"Unexpected error: {e}")
            continue
        finally:
            # if ser is no longer open, exit the thread
            if not ser.is_open:
                print("Serial port closed. Exiting read thread.")
                break
        
    
        
ser = None
read_thread = None
try:
    ser = serial.Serial('COM3', 57600, timeout=1)
    print("Serial port opened successfully.")
    read_thread = threading.Thread(target=read_from_serial, args=(ser,), daemon=True)
    read_thread.start()
    print("Serial read thread started.")
    
    print("Press 'a', 's', 'd', 'w', 'e', 'q', 'i', 'k', 'j', 'l' for input, space to stop, Esc to quit.")
    
    while True:
        key = getch()
        if key == '\x1b':  # Escape key
            print("Escape pressed. Quitting...")
            wheel_vel1 = 0
            wheel_vel2 = 0
            wheel_vel3 = 0
            wheel_vel4 = 0
            format_string = '<hhhhhhh'
        
            # Pack the 7 values into a binary byte string
            # This will be 7 * 2 = 14 bytes long
            message_bytes = struct.pack(format_string, 
                                    wheel_vel1, wheel_vel2, wheel_vel3, wheel_vel4,
                                    ax_pos1, ax_pos2, ax_pos3)

            ser.write(message_bytes)
            break
        elif key == ' ':
            print("Space pressed. Stopping all motors.")
            wheel_vel1 = 0
            wheel_vel2 = 0
            wheel_vel3 = 0
            wheel_vel4 = 0
        elif key == 'w': #forward
            wheel_vel1 = SPEED
            wheel_vel2 = SPEED
            wheel_vel3 = SPEED
            wheel_vel4 = SPEED
        elif key == 's': #backward
            wheel_vel1 = -SPEED
            wheel_vel2 = -SPEED
            wheel_vel3 = -SPEED
            wheel_vel4 = -SPEED
        elif key == 'd': #right
            wheel_vel1 = SPEED
            wheel_vel2 = -SPEED
            wheel_vel3 = -SPEED
            wheel_vel4 = SPEED
        elif key == 'a': #left
            wheel_vel1 = -SPEED
            wheel_vel2 = SPEED
            wheel_vel3 = SPEED
            wheel_vel4 = -SPEED
        elif key == 'q': #turn left
            wheel_vel1 = -SPEED
            wheel_vel2 = SPEED
            wheel_vel3 = -SPEED
            wheel_vel4 = SPEED
        elif key == 'e': #turn right
            wheel_vel1 = SPEED
            wheel_vel2 = -SPEED
            wheel_vel3 = SPEED
            wheel_vel4 = -SPEED
        elif key == 'i': #cam_up
            ax_pos1 = MECANUMBOT_MAX_CAM_POS
        elif key == 'k': #cam_down
            ax_pos1 = MECANUMBOT_MIN_CAM_POS
        elif key == 'j': #open gripper
            ax_pos2 = MECANUMBOT_CLOSED_GRIPPER_POS
            ax_pos3 = MECANUMBOT_CLOSED_GRIPPER_POS
        elif key == 'l': #close gripper,
            ax_pos2 = MECANUMBOT_OPEN_GRIPPER_POS
            ax_pos3 = MECANUMBOT_OPEN_GRIPPER_POS
        # Add other key controls as needed
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
        
        time.sleep(0.02)

except Exception as e:
    print(f"Error: {e}")
    
except KeyboardInterrupt:
    print("KeyboardInterrupt received. Quitting...")
    
finally:
    if ser is not None:
        ser.close()
        print("Serial port closed.") 
        time.sleep(1)  # Give some time for the read thread to notice the port is closed
    if read_thread is not None:
        read_thread.join()