import serial
import serial.tools.list_ports
import time

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

# find the correct serial port
teensyPort = ""
ports = serial.tools.list_ports.comports()
for port in ports:
    vid = port.vid
    pid = port.pid
    if vid == TEENSY_VID and pid == TEENSY_PID:
        teensyPort = port.device
        break

with serial.Serial(teensyPort, 115200, timeout=1) as ser:
    cmd = 2250

    while 1:
        ser.write((str(cmd) + '\n').encode())
        line = ser.readline()
        print(line.decode())

        cmd += 10

        if cmd > 4500:
            break

        time.sleep(0.1)
