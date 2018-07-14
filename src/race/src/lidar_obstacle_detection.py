#!/usr/bin/env python
import sys
import os
import datetime
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

import std_msgs.msg

import math
import numpy as np
import random

from datetime import datetime 

scan_data = None
last_scan_time = datetime.now()

def scan_callback(data):
	global scan_data, last_scan_time
	scan_data = data
	last_scan_time = datetime.now()

def to_degree( rad):
	return rad*(180.0/math.pi)



def lidar_obstacle_detection():

	bridge = CvBridge()
	camera1_pub = rospy.Publisher("camera_lidar", Image, queue_size=10)

	#min distance - subscribe to this
	dist_pub = rospy.Publisher("lidar_min_dist", std_msgs.msg.Float32, queue_size=10)

	rospy.Subscriber('scan', LaserScan, scan_callback)

	rospy.init_node('lidar_obstacle_detection')


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

	threshold = 0.5

        while not rospy.is_shutdown():
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
	

					
					x = width/2 + int(hori*100)
					y = height - int(vert*100)
					
					if (vert < threshold):
						cv2.circle(lidar_image, (x, y), 3, (0, 0, 255), -1)
						cv2.line(lidar_image, (x, y), (width/2, height), (0, 0, 255), 1)
					else:
						cv2.circle(lidar_image, (x, y), 2, (0, 255, 0), -1)
						cv2.line(lidar_image, (x, y), (width/2, height), (0, 255, 0), 1)


				print 'min distance = %f' % min_dist
				dist_pub.publish(min_dist)
				new_ros_image1 = bridge.cv2_to_imgmsg(lidar_image, "bgr8")
				camera1_pub.publish(new_ros_image1)

				#cv2.imshow('lidar', lidar_image)
				#cv2.waitKey(2)

		else:
			#scan_data is none...
			print 'scan_data is None'


		rate.sleep()
        


if __name__ == '__main__':
        print("lidar_obstacle_detection")
        lidar_obstacle_detection()
