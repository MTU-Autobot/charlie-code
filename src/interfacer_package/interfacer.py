import rospy
import sys
import serial
import serial.tools.list_ports
import time
#import signal
from geometry_msgs.msg import Twist

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

ENCODER_MSG = 0xA0
DRIVE_MSG = 0xA4


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)


def findPort():
    # find the correct serial port
    teensyPort = ""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        vid = port.vid
        pid = port.pid
        if vid == TEENSY_VID and pid == TEENSY_PID:
            teensyPort = port.device
            print("Interface board found on " + str(teensyPort) + "\n")
            return teensyPort


def listener():
    # create node for listening to twist messages
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rate = rospy.Rate(100)

    connected = False

    while not rospy.is_shutdown():
        try:
            if not connected:
                teensyPort = findPort()
                ser = serial.Serial(teensyPort, 115200, timeout=1)
                connected = True

            ser.write((str(ENCODER_MSG) + '\n').encode())
            line = ser.readline()
            print(line.decode())
            rate.sleep()

        # handle disconnect exceptions and attempt to reconnect
        except serial.serialutil.SerialException:
            connected = False
            print("Interface board not found, check connection!")
            time.sleep(1)
            pass


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("exit pls")
        pass
