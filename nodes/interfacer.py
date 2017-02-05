#!/usr/bin/env python

import rospy
import sys
import serial
import serial.tools.list_ports
import time
import math
import re
import tf
from geometry_msgs.msg import Twist, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

ENCODER_MSG = 0xA0
DRIVE_MSG = 0xA4

# encoders are 512 counts per revolution, 4 pulses per count. 20:1 gearbox
CNTS_PER_REV_WHEEL = 512 * 4 * 20
PI = math.pi
TWOPI = math.pi * 2
# distance in meters
WHEEL_SEPARATION = 0.6731
WHEEL_RADIUS = 0.371475

PreviousLeftEncoderCounts = 0
PreviousRightEncoderCounts = 0
current_time_encoder = None
last_time_encoder = None
DistancePerCount = (TWOPI * WHEEL_RADIUS) / CNTS_PER_REV_WHEEL

x = 0.0
y = 0.0

vx = 0.0
vy = 0.0
deltaLeft = 0.0
deltaRight = 0.0

linear_encoder = 0.0
angular_encoder = 0.0

heading = 0.0
heading_old = 0.0


def twistCallback(data):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(data.linear.x, data.linear.y, data.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(data.angular.x, data.angular.y, data.angular.z))

    # get linear x and angular z for drive info
    velocity = data.linear.x
    turn  = data.angular.z

    # calculate wheel velocities
    leftVelocity = (velocity - turn * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    rightVelocity = (velocity + turn * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS


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


def sendDriveMessage(leftVal, rightVal):
    # send the drive command to get interface board ready
    ser.write((str(DRIVE_MSG) + '\n').encode())


def wheel_callback(left_wheel, right_wheel):
    global current_time_encoder
    global vx
    global vy
    global PreviousLeftEncoderCounts
    global PreviousRightEncoderCounts
    global last_time_encoder
    global linear_encoder
    global angular_encoder
    global heading
    global heading_old
    global x
    global y

    current_time_encoder = rospy.get_rostime()

    delta_left = left_wheel - PreviousLeftEncoderCounts
    delta_right = right_wheel - PreviousRightEncoderCounts

    left_wheel_est_vel = delta_left * DistancePerCount
    right_wheel_est_vel = delta_right * DistancePerCount

    linear_encoder  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5
    angular_encoder = (right_wheel_est_vel - left_wheel_est_vel) / WHEEL_SEPARATION

    direction = heading + angular_encoder * 0.5;
    x += linear_encoder * math.cos(direction);
    y += linear_encoder * math.sin(direction);
    heading += angular_encoder;

    print 'X: ' + str(x) + '\tY: ' + str(y) + '\theading: ' + str(heading)

    # Update old values
    PreviousLeftEncoderCounts = left_wheel
    PreviousRightEncoderCounts = right_wheel
    last_time_encoder = current_time_encoder


def twistListener():
    global x
    global y
    global th
    global vx
    global vy
    global vth

    # create node for listening to twist messages
    rospy.init_node("interfacer")
    rospy.Subscriber("cmd_vel", Twist, twistCallback)
    rate = rospy.Rate(100)

    pub = rospy.Publisher("/odom", Odometry)
    frame_id = "/odom"
    child_frame_id = "/base_footprint"

    #odom_broadcaster = tf.TransformBroadcaster()

    connected = False

    current_time = rospy.get_rostime()
    last_time = rospy.get_rostime()

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

                if len(line) > 10:
                    current_time = rospy.get_rostime()
                    encoders = re.split(r'\t+', line)
                    print encoders
                    wheel_callback(int(encoders[1]), int(encoders[2]))

                    msg = Odometry()
                    msg.header.stamp = current_time
                    msg.header.frame_id = "odom"
                    msg.child_frame_id = "base_link"

                    msg.pose.pose.position = Point(x, y, 0.0)
                    q = tf.transformations.quaternion_from_euler(0, 0, heading)
                    msg.pose.pose.orientation = Quaternion(*q)
                    #msg.twist.twist.linear = Point(vx, vy, 0.0)
                    #msg.twist.twist.angular.x = 0.0
                    #msg.twist.twist.angular.y = 0.0
                    #msg.twist.twist.angular.z = vth

                    pub.publish(msg)

                    last_time = current_time

            rate.sleep()

        # handle disconnect exceptions and attempt to reconnect
        except serial.serialutil.SerialException:
            connected = False
            print("Interface board not found, check connection!")
            time.sleep(1)
            pass


def odomPublisher():
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
