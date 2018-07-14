#!/usr/bin/env python
import sys
import os
import datetime
import rospy

import std_msgs.msg
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

import math
import numpy as np

from datetime import datetime

all_x = []
all_y = []

my_speed = 0

count = 0
d = None
w = 0.2 #width of the car
old_dists = {}
old_data = {}
delta_t = 3.0
t = 0.0
avg = 0.0

#constants
p = 0.1
a_max = 9.1789 #accel
a_min = -8.896 #brake

scan_data = None
last_scan_time = datetime.now()

def scan_callback(data):
	global scan_data, last_scan_time
	scan_data = data
	last_scan_time = datetime.now()

def rpm_callback(data):
	
	#1 rotation = 0.11 meter
	rpm = data.data

	'''
	dist = rpm * 0.11
	speed = dist / 60.0 # remember! rpm := rotations per 'minute'
	print 'rpm = %d' % rpm
	print 'speed = %f m/s' % speed
	'''
		
	global window, cnt, start_averaging, my_speed
	window[cnt] = rpm
	cnt += 1
	if cnt == WIN_SIZE:
		start_averaging = True
	cnt %= WIN_SIZE

	if start_averaging:
		avg_rpm = np.sum(window)/float(WIN_SIZE)

		dist = avg_rpm * 0.11

		#speed = distance / time
		speed = dist / 60.0

		myspeed = speed
	
		#print 'speed = %f m/s' % speed
	

#acceleration stuff
def imu_callback(data):
	linear_accel = data.linear_acceleration
	a_x = linear_accel.x
	a_y = linear_accel.y
	#print a_x, a_y	

	all_x.append(a_x)
	all_y.append(a_y)

	#print("min x accel", min(all_x))
	#print("min y accel", min(all_y))

def lidar_stuff():

	FREQ = 20
	rate = rospy.Rate(FREQ)

	W = 30.0
	D = 35.0	#changed this value from 50.0
	theta = to_degree(math.atan(D/(W/2)))

	while scan_data is None:
		print 'waiting for LaserScan data'
		rospy.sleep(1)
	
	angle_min = to_degree(scan_data.angle_min)
	angle_max = to_degree(scan_data.angle_max)
	angle_increment = to_degree(scan_data.angle_increment)
	
	mid = len(scan_data.ranges) / 2
	num_points_oneside = int( (90 - theta)/angle_increment )
	num_points = num_points_oneside * 2
	print mid, theta, num_points_oneside
	start_idx = mid - num_points_oneside
	end_idx = mid + num_points_oneside
	print 'start = %d, end = %d' % (start_idx, end_idx)

	WIN_SIZE = 5
	window = np.zeros([WIN_SIZE, num_points])
	cnt = 0
	start_averaging = False


	if scan_data is not None:

		dist_array = np.array(scan_data.ranges[start_idx:end_idx], copy=True)
		
		window[cnt, :] = np.copy(dist_array)
		cnt += 1
		if cnt == WIN_SIZE:
			start_averaging = True
		cnt %= WIN_SIZE			

		if start_averaging:

			moving_avg = np.sum(window, axis=0)/float(WIN_SIZE)
			#print moving_avg[num_points_oneside]
			#print min(moving_avg)

			#draw lidar image 
			width = 640
			height = 480
			lidar_image = np.zeros((height,width,3), np.uint8)


			min_dist = scan_data.range_max
			for i in range(num_points):
				dist = dist_array[i]
				angle = theta + angle_increment*i
				vert = dist * math.sin(angle*math.pi/180.0)
				hori = dist * math.cos(angle*math.pi/180.0)

				if (vert<min_dist):
					min_dist = vert

			#print 'min distance = %f' % min_dist
			return min_dist, angle, 

		else:
			#scan_data is none...
			print 'scan_data is None'


def to_degree( rad):
	return rad*(180.0/math.pi)

def distance_calculator():

	global d, w, delta_t, t, avg, my_speed

	v_r = my_speed
	data_read = data.ranges
	
	a = []
	widths = []


	#what does this mean??
	W = 30.0
	D = 50.0	
	theta = to_degree(math.atan(D/(W/2)))

	angle_min = to_degree(data.angle_min)
	angle_max = to_degree(data.angle_max)
	angle_increment = to_degree(data.angle_increment)
	
	mid = len(data.ranges) / 2
	num_points_oneside = int( (90 - theta)/angle_increment )
	num_points = num_points_oneside * 2
	start_idx = mid - num_points_oneside
	end_idx = mid + num_points_oneside

	dist_array = data.ranges[start_idx:end_idx]

	for i in range(num_points):
		dist = dist_array[i]

		angle = theta + angle_increment*i

		vert = dist * math.sin(angle*math.pi/180.0)
		hori = dist * math.cos(angle*math.pi/180.0)

		a.append([round(angle), vert])
		
		#width = data_read[i]*math.sin(math.radians(i))
		#height = data_read[i]*math.cos(math.radians(i))*-1 #because y is negative in these quadrants

		# if width < 0:
		# 	width = width * -1

		# widths.append(width)

		# if width < w/2:
		# 	a.append([i, height*39.37]) #in inches


	a.sort(key=lambda x: x[1])

	
	if len(a) == 0:
		return

	#print(a)
	#print(a[0])

	min_i = a[0][0]
	min_d = a[0][1]

	'''
	total = list()
	for k in old_data:
		if t - k > avg:
			continue
		if old_data[k][min_i] != float("inf") and old_data[k][min_i] != float("-inf"):
			val = old_data[k][min_i]*math.sin(math.radians(min_i))
			if val < 0:
				val = val * -1
			total.append(val)

	if min_d < 100:
		total.append(min_d)
    
	avg_d = sum(total) / float(len(total))
	'''
	avg_d = min_d
	old_dists[t] = avg_d
	old_data[t] = dist_array


	if t > delta_t:
		del old_dists[t-delta_t-1]
		del old_data[t-delta_t-1]
		old_d = old_dists[t-delta_t] #shortest distance delta_t timesteps ago
		v_f = v_r + (avg_d - old_d) / delta_t
	
		keep_distance = (v_r*p) + 0.5*a_max*p*p + 0.5*(1/a_min)*(v_r+p*a_max)**2 - (v_f * v_f)*0.5*(1/a_min)
		print(dist)
		
		print(v_f)

	t = t + 1



def safe_distance():
	rospy.Subscriber('scan', LaserScan, scan_callback)
	rospy.Subscriber('rpm', std_msgs.msg.Int32, rpm_callback)
	rospy.Subscriber('imu', Imu, imu_callback)
	rospy.init_node('safe_distance')

	distance_calculator()

	'''
	FREQ = 20
	rate = rospy.Rate(FREQ)
	while not rospy.is_shutdown():
		rate.sleep()
	'''
	rospy.spin()
        
if __name__ == '__main__':
        print("safe_distance")
        safe_distance()
