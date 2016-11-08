import serial
import serial.tools.list_ports
import time
import sys

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

ENCODER_MSG = 0xA0
DRIVE_MSG = 0xA4

# find the correct serial port
teensyPort = ""
ports = serial.tools.list_ports.comports()
for port in ports:
    vid = port.vid
    pid = port.pid
    if vid == TEENSY_VID and pid == TEENSY_PID:
        teensyPort = port.device
        print("Interface board found on " + str(teensyPort))
        break
else:
    # exit program if board not found
    sys.exit("Interface board not found, exiting!")

while 1:
    try:
        with serial.Serial(teensyPort, 115200, timeout=1) as ser:
            while 1:
                ser.write((str(ENCODER_MSG) + '\n').encode())
                line = ser.readline()
                print(line.decode())
                time.sleep(0.01)

    # handle disconnect exceptions and attempt to reconnect
    except serial.serialutil.SerialException:
        #sys.exit("Interface board disconnected!")
        print("Interface board connection lost!")
        time.sleep(1)
        pass
