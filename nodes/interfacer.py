#!/usr/bin/env python

import rospy
import sys
import serial
import serial.tools.list_ports
import time
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry

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


def twistCallback(data):
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

    # create position vector
    xPos += (distance - distanceOld) * math.cos(theta)
    yPos += (distance - distanceOld) * math.sin(theta)
    positionVector = [xPos, yPos, 0]

    #update previous values
    lastRight = rightWheel
    lastLeft = leftWheel


def twistListener():
    # create node for listening to twist messages
    rospy.init_node("interfacer")
    rospy.Subscriber("cmd_vel", Twist, twistCallback)
    rate = rospy.Rate(100)

    connected = False

    while not rospy.is_shutdown():
        try:
            if not connected:
                # connect to the serial port that the interface board is on
                teensyPort = findPort()
                ser = serial.Serial(teensyPort, 115200, timeout=1)
                connected = True
            else:
                # send request for encoder data and process the return
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


def odomPublisher():
    pub = rospy.Publisher("/odom", Odometry)
    frame_id = "/odom"
    child_frame_id = "/base_footprint"

    # build odometry message
    # http://answers.ros.org/question/79851/python-odometry/
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position = Point(positionVector[0], positionVector[1], positionVector[2])
    #msg.pose.pose.orientation = Quaternion(*)


if __name__ == "__main__":
    try:
        twistListener()
    except rospy.ROSInterruptException:
        print("exit pls")
        pass
