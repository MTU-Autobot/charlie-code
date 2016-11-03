import serial
import time



with serial.Serial('/dev/ttyACM1', 115200, timeout=1) as ser:
    cmd = 2250

    while 1:
        ser.write((str(cmd) + '\n').encode())
        line = ser.readline()
        print(line.decode())

        cmd += 10

        if cmd > 4500:
            break

        time.sleep(0.1)
