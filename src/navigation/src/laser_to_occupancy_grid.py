#!/usr/bin/env python

# OccupancyGrid publisher
# author: Riccardo Giubilato
# mail: riccardo.giubilato@gmail.com
# year: 2017
# ============================================
# subscribed topics:
#	/scan - contains LaserScan messages
# published topics:
#   /map - contains OccupancyGrid messages
# listened tf:
#   /base_link to /world
#	/hokuyo_link to /base_link
#   /map to /world
# ============================================

import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# initialize node
rospy.init_node('later_to_occupancy_grid_node')

# listener of transforms between the car_base_link and the world frame
car_pose = tf.TransformListener(cache_time = rospy.Duration(2.0))

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = 0.01
width = 500
height = 500

# Initialize car pose relative to world
x_car = 0.0
y_car = 0.0

# square size of the car footprint [m]
footprint = 0.1

# Map update rate (defaulted to 5 Hz)
rate = 5.0

# Range data
car_range = 0.0
increment = 0.0

def set_free_cells(grid, position, size):
	# set free the cells occupied by the car
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# size: 			r     radius of the footprint
	global resolution

	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2

	# set the roi to 1: known free positions
	for i in range(-size//2, size//2):
		for j in range(-size//2, size//2):
			grid[int(i + off_x), int(j + off_y)] = 1

def set_obstacle(grid, position, orientation, position_sonar, quaternion_sonar, car_range):
	# set the occupied cells when detecting an obstacle
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# orientation:      quaternion, orientation of the car
	global resolution
	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2
	euler = tf.transformations.euler_from_quaternion(orientation)
	cul_increment = 0.0
	for curr_range in car_range:

		if not (curr_range == 0.0 or math.isinf(curr_range)):

			rotMatrix = numpy.array([[numpy.cos(euler[2] + cul_increment),   numpy.sin(euler[2] + cul_increment)],
									[-numpy.sin(euler[2] + cul_increment),  numpy.cos(euler[2] + cul_increment)]])

			obstacle = numpy.dot(rotMatrix,numpy.array([0, ( curr_range + position_sonar[0]) // resolution])) + numpy.array([off_x,off_y])

			rospy.loginfo("FOUND OBSTACLE AT: x:%f y:%f", obstacle[0], obstacle[1])

			# set probability of occupancy to 100 and neighbour cells to 50
			grid[int(obstacle[0]), int(obstacle[1])] = int(100)
			if  grid[int(obstacle[0]+1), int(obstacle[1])]   < int(1):
				grid[int(obstacle[0]+1), int(obstacle[1])]   = int(50)
			if  grid[int(obstacle[0]), 	 int(obstacle[1]+1)] < int(1):
				grid[int(obstacle[0]),   int(obstacle[1]+1)] = int(50)
			if  grid[int(obstacle[0]-1), int(obstacle[1])]   < int(1):
				grid[int(obstacle[0]-1), int(obstacle[1])]   = int(50)
			if  grid[int(obstacle[0]),   int(obstacle[1]-1)] < int(1):
				grid[int(obstacle[0]),   int(obstacle[1]-1)] = int(50)

			t = 0.5
			i = 1
			free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
			while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
				grid[int(free_cell[0]), int(free_cell[1])] = int(0)
				free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
				i = i+1
			cul_increment += increment
			

def callback_range(msg):
	# callback range
	global car_range
	global increment
	car_range = msg.ranges
	increment = msg.angle_increment


# Subscribers
range_sub = rospy.Subscriber("/scan", LaserScan, callback_range)

# Publishers
occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)


# main function
if __name__ == '__main__':

	# set grid parameters
	if rospy.has_param("occupancy_rate"):
		rate = rospy.get_param("occupancy_rate")

	if rospy.has_param("grid_resolution"):
		resolution = rospy.get_param("grid_resolution")

	if rospy.has_param("grid_width"):
		width = rospy.get_param("grid_width")

	if rospy.has_param("grid_height"):
		height = rospy.get_param("grid_height")

	# fill map_msg with the parameters from launchfile
	map_msg.info.resolution = resolution
	map_msg.info.width = width
	map_msg.info.height = height
	map_msg.data = range(width*height)

	# initialize grid with -1 (unknown)
	grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int),
	         dtype=numpy.int)
	grid.fill(int(-1))

	# set map origin [meters]
	map_msg.info.origin.position.x = - width // 2 * resolution
	map_msg.info.origin.position.y = - height // 2 * resolution

	loop_rate = rospy.Rate(rate)

	t = car_pose.getLatestCommonTime("base_link", "world")
	car_pose.waitForTransform("base_link", "world", t, rospy.Duration(10.0))

	while not rospy.is_shutdown():
		
		try:
			t = car_pose.getLatestCommonTime("/base_link", "/world")
			position, quaternion = car_pose.lookupTransform("/world", "/base_link", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		try:
			t = car_pose.getLatestCommonTime("/base_link", "/hokuyo_link")
			position_sonar, quaternion_sonar = car_pose.lookupTransform("/base_link", "/hokuyo_link", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		# write 0 (null obstacle probability) to the free areas in grid
		set_free_cells(grid, position, int(footprint//resolution))

		# write p>0 (non-null obstacle probability) to the occupied areas in grid
		set_obstacle(grid, position, quaternion, position_sonar, quaternion_sonar, car_range)

		# stamp current ros time to the message
		map_msg.header.stamp = rospy.Time.now()

		# build ros map message and publish
		for i in range(width*height):
			map_msg.data[i] = grid.flat[i]
		occ_pub.publish(map_msg)

		loop_rate.sleep()
