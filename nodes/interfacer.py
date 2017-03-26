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
from tf.broadcaster import TransformBroadcaster
from math import sin, cos, pi, ceil


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

# global vars for speed
globalLeftVelocity = 2047
globalRightVelocity = 2047

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


class OdometryPub:
    def __init__(self):
        self.base_frame_id = 'base_link' # the name of the base frame of the robot
        self.odom_frame_id = 'odom' # the name of the odometry reference frame

        # internal data
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.then = 0

        self.odomPub = rospy.Publisher("/odom", Odometry,queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    def update(self):
        global x
        global y
        global vx
        global vy
        global heading
        global heading_old

        if(self.then == 0):
            self.then = rospy.Time.now()
            return

        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin( heading / 2 )
        quaternion.w = cos( heading / 2 )
        self.odomBroadcaster.sendTransform(
            (x, y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
        )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = heading - heading_old
        self.odomPub.publish(odom)


def map(value, fromLow, fromHigh, toLow, toHigh):
    # figure out width of each range
    fromSpan = fromHigh - fromLow
    toSpan = toHigh - toLow
    # convert left range into 0-1
    valueScaled = float(value - fromLow) / float(fromSpan)
    # convert 0-1 to a value in to range
    return toLow + (valueScaled * toSpan)


def twistCallback(data):
    global globalLeftVelocity
    global globalRightVelocity

    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(data.linear.x, data.linear.y, data.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(data.angular.x, data.angular.y, data.angular.z))

    # get linear x and angular z for drive info
    velocity = data.linear.x
    turn  = data.angular.z

    # calculate wheel velocities
    leftVelocity = (velocity - turn * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    rightVelocity = (velocity + turn * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    # remap values to ints for sending
    globalLeftVelocity = int(map(leftVelocity, -3.0, 3.0, 0, 4095))
    globalRightVelocity = int(map(rightVelocity, -3.0, 3.0, 0, 4095))


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

    odometry_publisher = OdometryPub()

    #pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    #frame_id = "/odom"
    #child_frame_id = "/base_link"

    ######odom_broadcaster = tf.TransformBroadcaster()

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
                    # print encoders
                    wheel_callback(int(encoders[1]), int(encoders[2]))

                    #msg = Odometry()
                    #msg.header.stamp = current_time
                    #msg.header.frame_id = "odom"
                    #msg.child_frame_id = "base_link"

                    #msg.pose.pose.position = Point(x, y, 0.0)
                    #q = tf.transformations.quaternion_from_euler(0, 0, heading)
                    #msg.pose.pose.orientation = Quaternion(*q)
                    ####msg.twist.twist.linear = Point(vx, vy, 0.0)
                    ####msg.twist.twist.angular.x = 0.0
                    ####msg.twist.twist.angular.y = 0.0
                    ###msg.twist.twist.angular.z = vth

                    # publish odometry and send drive command
                    #pub.publish(msg)
                    odometry_publisher.update()
                    ser.write((str(DRIVE_MSG) + "\n").encode())
                    ser.write((str(globalLeftVelocity) + "\t" + str(globalRightVelocity) + "\n").encode())

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
