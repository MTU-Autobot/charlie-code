import rospy
import sys
import serial
import serial.tools.list_ports
import time
import math
from geometry_msgs.msg import Twist

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

ENCODER_MSG = 0xA0
DRIVE_MSG = 0xA4

# encoders are 512 counts per revolution, 4 pulses per count. 20:1 gearbox
CNTS_PER_REV_WHEEL = 512 * 4 * 20
TURN_RADIUS = 0.7239
PI = math.pi
TWOPI = math.pi * 2

# left and right wheel encoder position
leftWheel = 0
rightWheel = 0

# left and right positions in meters
leftM = 0.0
rightM = 0.0

# distance variables
distance = 0.0
distanceOld = 0.0

# variables for position from previous call
lastLeft = 0
lastRight = 0

# positions
theta = 0.0
xPos = 0.0
yPos = 0.0
positionVector = [0.0, 0.0, 0.0]


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


def calculatePosition(leftVal, rightVal):
    # calculate positions
    rightWheel = rightVal
    rightM -= (rightWheel - lastRight) / CNTS_PER_REV_WHEEL
    leftWheel = leftVal
    leftM += (leftWheel - lastLeft) / CNTS_PER_REV_WHEEL

    # update distances
    distanceOld = distance
    distance = (rightM + leftM) / 2

    # calculate angle of rotation
    theta = (rightM - leftM) / TURN_RADIUS
    while theta >= PI:
        theta -= TWOPI
    while theta <= -PI:
        theta += TWOPI

    xPos += (distance - distanceOld) * math.cos(theta)
    yPos += (distance - distanceOld) * math.sin(theta)
    positionVector = [xPos, yPos, 0]


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
            else:
                ser.write((str(ENCODER_MSG) + '\n').encode())
                line = ser.readline()
                line.decode()

            rate.sleep()

        # handle disconnect exceptions and attempt to reconnect
        except serial.serialutil.SerialException:
            connected = False
            print("Interface board not found, check connection!")
            time.sleep(1)
            pass
'''
def talker():
    pub = rospy.Publisher('odom', )
'''

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("exit pls")
        pass
