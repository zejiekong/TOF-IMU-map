#!/usr/bin/env python3

import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

def set_obstacle(grid, orientation, tof_range):
	# set the occupied cells when detecting an obstacle
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# orientation:      quaternion, orientation of the car
	global resolution

	off_x = width  // 2
	off_y = height // 2

	euler = tf.transformations.euler_from_quaternion(orientation)
	#print(euler)

	if not tof_range == 0.0:
		rotMatrix = numpy.array([[numpy.cos(euler[2]-(math.pi/2)),  -numpy.sin(euler[2]-(math.pi/2))],
			                     [numpy.sin(euler[2]-(math.pi/2)),  numpy.cos(euler[2]-(math.pi/2))]])
		obstacle = numpy.dot(rotMatrix,numpy.array([0, (tof_range) // resolution])) + numpy.array([off_x,off_y])
		#print(tof_range)
		# set probability of occupancy to 100 and neighbour cells to 50
		grid[int(obstacle[0]), int(obstacle[1])] = int(100)
		# t = 0.5
		# i = 1
		# free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
		# #while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
		# while int(free_cell[0]) != int(obstacle[0]) and int(free_cell[1]) != int(obstacle[1]):
		# 	grid[int(free_cell[0]), int(free_cell[1])] = int(0)
		# 	free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
		# 	i = i+1;

def callback_range(msg):
	
	tof_range = msg.ranges[0]
	try:
		t = imu_pose.getLatestCommonTime("/imu", "/map")
		position, quaternion = imu_pose.lookupTransform("/imu","/map", t)
		set_obstacle(grid,quaternion,tof_range)
		# stamp current ros time to the message
		map_msg.header.stamp = rospy.Time.now()
		map_msg.data = grid.flat
		occ_pub.publish(map_msg)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass

# initialize node
rospy.init_node('later_to_occupancy_grid_node')

# listener of transforms between the car_base_link and the world frame
imu_pose = tf.TransformListener()

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = 0.1
width = 1000
height = 1000

# fill map_msg with the parameters from launchfile
map_msg.info.resolution = resolution
map_msg.info.width = width
map_msg.info.height = height
#map_msg.data = range(width*height)

# initialize grid with -1 (unknown)
grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int64),
			dtype=numpy.int64)
grid.fill(int(-1))

# set map origin [meters]
map_msg.info.origin.position.x = - width // 2 * resolution
map_msg.info.origin.position.y = - height // 2 * resolution

# Publishers
occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 1)

# Subscribers
range_sub = rospy.Subscriber("/scan", LaserScan, callback_range,queue_size=1)

rospy.spin()

	






	


