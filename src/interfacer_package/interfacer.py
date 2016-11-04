import rospy
import sys
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import time
import signal

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

ENCODER_MSG = 0xA0
DRIVE_MSG = 0xA4


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)

def sigint_handler(signum, frame):
    sys.exit(0)

def listener():
    signal.signal(signal.SIGINT, sigint_handler)

    # find the correct serial port
    teensyPort = ""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        vid = port.vid
        pid = port.pid
        if vid == TEENSY_VID and pid == TEENSY_PID:
            teensyPort = port.device
            print("Interface board found on " + str(teensyPort) + "\n")
            break
    else:
        # exit program if board not found
        sys.exit("Interface board not found, exiting!")

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)

    while 1:
        try:
            with serial.Serial(teensyPort, 115200, timeout=1) as ser:
                while 1:
                    ser.write((str(ENCODER_MSG) + '\n').encode())
                    line = ser.readline()
                    print(line.decode())
                    time.sleep(0.01)

        # handle disconnect exceptions and attempt to reconnect
        except serial.serialutil.SerialException as a:
            print(a)
            #sys.exit("Interface board disconnected!")
            print("Interface board connection lost!")
            time.sleep(1)
            pass


if __name__ == '__main__':
    listener()
