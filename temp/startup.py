#!/usr/bin/python

import subprocess, threading, time, os, signal

##########################
# start of thread classes
##########################

# This thread runs roscore
class roscoreThread (threading.Thread):
	def __init__(self, threadID):
	        threading.Thread.__init__(self)
	        self.threadID = threadID
	        self.name = "roscore"
		self.process = 0;
		self.f = open('core.log', 'w')
	def run(self):
		self.f.truncate()
	        print "Starting " + self.name
		self.process = subprocess.Popen(["roscore"], stdout = self.f, stderr = self.f) 
	def stop(self):
		print "Attempting to exit " + self.name
		self.f.close()
		self.process.send_signal(2) #send a SIGINT to roscore process	
		self._is_running = False


# This thread will use rostopic to list all current ROS nodes
class rostopicThread (threading.Thread):
    	def __init__(self, threadID):
        	threading.Thread.__init__(self)
        	self.threadID = threadID
        	self.name = "rostopic"
		self.process = 0
		self.f = open('rostopic.log', 'w')
    	def run(self):
		self.f.truncate()
        	print "Starting " + self.name
		self.process = subprocess.Popen(["rostopic", "list"], stdout = self.f, stderr = self.f)
	def stop(self):
        	print "Attempting to exit " + self.name
		self.f.close()
		self.process.send_signal(2) #send a SIGINT to rostopic process
		self._is_running = False

# This thread will run mapping
class mappingThread (threading.Thread):
    	def __init__(self, threadID):
        	threading.Thread.__init__(self)
        	self.threadID = threadID
        	self.name = "mapping"
		self.process = 0
		self.f = open('mapping.log', 'w')
    	def run(self):
		self.f.truncate()
        	print "Starting " + self.name
		self.process = subprocess.Popen(["rosrun", "rtabmap_ros", "rtabmap"], stdout = self.f, stderr = self.f)
	def stop(self):
		self.f.close()
        	print "Attempting to exit " + self.name
		self.process.send_signal(2) #send a SIGINT to mapping process
		self._is_running = False

# This thread will run the GPS
class gpsThread (threading.Thread):
    	def __init__(self, threadID):
        	threading.Thread.__init__(self)
        	self.threadID = threadID
        	self.name = "GPS"
		self.process = 0;
		self.f = open('gps.log', 'w')
    	def run(self):
		self.f.truncate()
        	print "Starting " + self.name
		self.process = subprocess.Popen(["roslaunch", "ublox_gps", "ublox_gps.launch"], stdout = self.f, stderr = self.f)
	def stop(self):
		self.f.close()
        	print "Attempting to exit " + self.name
		self.process.send_signal(2) #send a SIGINT to GPS process
		self._is_running = False

# This thread will run the camera
class cameraThread (threading.Thread):
    	def __init__(self, threadID):
        	threading.Thread.__init__(self)
        	self.threadID = threadID
        	self.name = "camera"
		self.process = 0;
		self.f = open('camera.log', 'w')
    	def run(self):
		self.f.truncate()
        	print "Starting " + self.name
		self.process = subprocess.Popen(["roslaunch", "zed_wrapper", "zed.launch"], stdout = self.f, stderr = self.f)
	def stop(self):
		self.f.close()
        	print "Attempting to exit " + self.name
		self.process.send_signal(2) #send a SIGINT to camera process
		self._is_running = False

# This thread will run the IMU
class imuThread (threading.Thread):
    	def __init__(self, threadID):
        	threading.Thread.__init__(self)
        	self.threadID = threadID
        	self.name = "IMU"
		self.process = 0;
		self.f = open('imu.log', 'w')
    	def run(self):
		self.f.truncate()
        	print "Starting " + self.name
		self.process = subprocess.Popen(["roslaunch", "phidgets_imu", "imu.launch"], stdout = self.f, stderr = self.f)
	def stop(self):
		self.f.close()
        	print "Attempting to exit " + self.name
		self.process.send_signal(2) #send a SIGINT to camera process
		self._is_running = False

#TODO add thread classes for IMU, motor encoders, navigation, and localization
#TODO maybe a thread class for lidar?
#TODO test GPS and mapping and camera

##############################
# Beginning of startup process
##############################

# Create new threads
coreThread = roscoreThread(1)
topicList = rostopicThread(2)
camera = cameraThread(3)
gps = gpsThread(4)
mapping = mappingThread(5)
imu = imuThread(6)

# Start new Threads
coreThread.start()

time.sleep(2)

gps.start()
mapping.start()

time.sleep(5)
topicList.start()

time.sleep(2)

# terminate threads
mapping.stop()
gps.stop()
coreThread.stop()


