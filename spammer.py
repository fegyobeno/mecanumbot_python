import time
import msvcrt

def getch():
        # get char without termios
        ch = msvcrt.getch().decode('utf-8')
        t_time = time.time()
        
        return ch, t_time


log = []
key = ''
while key != 'q':  # Escape key
    key, t_time = getch()
    log.append(t_time)
    print(f"Key pressed: {key} at time: {t_time}")

avg = sum([log[i] - log[i-1] for i in range(1, len(log))])/(len(log)-1)
print(f"Average time between key presses: {avg*1000} milliseconds")
