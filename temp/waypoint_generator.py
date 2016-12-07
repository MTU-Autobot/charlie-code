#!/usr/bin/python
import math


# **************************************************************************
# Func:    fetch_gps_points
#
# Purpose: This script extracts GPS waypoints from a file, converts them to
#          a usable form, and sends a waypoint to ROS whenever it reaches
#          its current goal until there are not more waypoints left.
# **************************************************************************





# *************************************************************************
# Func:    degreesToMeters
#
# Purpose: This method takes a number of latitude/longitude degrees and
#          converts it to meters
#          
#          NOTE: This function converts the x and y coordinates separately
#
# ARGS:    y: A pointer to a GPS latitude that will be transformed into
#             meters from the latitude 
#          x: A pointer to a GPS longitude that will be transformed into
#             meters from the longitude 
#	   latitude: the current latitude Charlie is located at. This is
#		     used to correct errors in calculations due to smaller
#                    distances between longitudes as the latitude moves
#                    away from the Equator
#
# Return:  A list containing the X and Y coordinates for the next waypoint
# *************************************************************************
def degreesToMeters(y, x, latitude):
	# Radius of the Earth in meters
	earth_radius = 6378137
	

	# Intermediate values used in calculations
	x_a = 0.0
	x_c = 0.0
	x_d = 0.0

	y_a = 0.0
	y_c = 0.0
	y_d = 0.0
	
	# Convert GPS coordinates from degrees to radians
	x = math.radians(x)
	y = math.radians(y)
	latitude = math.radians(latitude)
	
	# Convert y (latitude) to meters
	y_a = math.sin(y/2) * math.sin(y/2) + math.cos(0) * math.cos(y) * math.sin(0) * math.sin(0)    
	y_c = 2 * math.atan2(math.sqrt(y_a), math.sqrt(1 - y_a))
	y_d = earth_radius * y_c 
	
	# Convert x (longitude) to meters
	x_a = math.sin(0) * math.sin(0) + math.cos(latitude) * math.cos(latitude) * math.sin(x/2) * math.sin(x/2)
	x_c = 2 * math.atan2(math.sqrt(x_a), math.sqrt(1 - x_a))
	x_d = earth_radius * x_c
	
	ret = [y_d,x_d]
	return ret
	

# NOTE: The first waypoint will be indexed at 1 since I append 
# the coordinates to the list and the 0 index is an empty list.
coordinateList = [[]]

# NOTE: The waypoint count starts at 0 since it is 
# immediately incremented when we calculate the first waypoint
waypointCount = 0
currentOrientation = 180.0
currentOrientation = math.radians(currentOrientation)

currentCoordinates = [47.2175, -88.5528]
nextCoordinates = [47.2175, -88.5528]
currentLocation = [0.0, 0.0]
nextWaypoint = [0.0,0.0]
distanceRemaining = [0.0, 0.0]

# Open the waypoints txt file then read and store each waypoint
with open("gps_waypoints.txt") as f:
	for line in f:
		coordinateList.append(line.split(" ", 2))

print "Coordinate List:"
print str(coordinateList)

while waypointCount < len(coordinateList):
	
	# TODO: grab a GPS message to update the current coordinates
	# TODO: update current orientation

	# Calculate distance to next waypoint in meters
	distanceRemaining[0] = nextCoordinates[0] - currentCoordinates[0]
	distanceRemaining[1] = nextCoordinates[1] - currentCoordinates[1]
	distanceRemaining = degreesToMeters(distanceRemaining[0], distanceRemaining[1], currentCoordinates[0])
	
	# Get next waypoint if we are within 2 meters
	if (math.sqrt(distanceRemaining[0] * distanceRemaining[0] + distanceRemaining[1] * distanceRemaining[1]) < 2.0):
		# Increment waypoint counter and grab the next coordinates from the waypoint list
		waypointCount = waypointCount + 1
		if (waypointCount == len(coordinateList)):
			continue
		nextCoordinates[0] = float(coordinateList[waypointCount][0])
		nextCoordinates[1] = float(coordinateList[waypointCount][1])
		print "Current GPS Coordinates:"
		print str(currentCoordinates)
		print "Target GPS Coordinates:"
		print str(nextCoordinates)
		
		# Calculate next waypoint. It will be a distance from Charlie's current location
		nextWaypoint = degreesToMeters(nextCoordinates[0] - currentCoordinates[0], nextCoordinates[1] - currentCoordinates[1], nextCoordinates[0])
		
		# Rotate waypoint arount the origin so that it matches Charlie's current orientation
		temp = (nextWaypoint[1] * math.sin(currentOrientation)) + (nextWaypoint[0] * math.cos(currentOrientation))
		nextWaypoint[1] = (nextWaypoint[1] * math.cos(currentOrientation)) - (nextWaypoint[0] * math.sin(currentOrientation))
		nextWaypoint[0] = temp
		print "Next Waypoint:"
		print str(nextWaypoint)
		
		currentCoordinates[0] = nextCoordinates[0]
		currentCoordinates[1] = nextCoordinates[1]
		# TODO: send a message to global map containing the next waypoint
	
