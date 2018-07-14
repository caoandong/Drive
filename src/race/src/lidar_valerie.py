#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import curses
import math
from operator import itemgetter

count = 0
d = None
w = 0.3 #width of the car
old_dists = {}
old_data = {}
delta_t = 3.0
t = 0.0
avg = 0.0
#DELTA T NEEDS TO BE GREATER THAN AVG

#ignoring the infs
def callback1(data):

	global d
	global w
	global delta_t
	global t
	global avg	

	v_r = 0 #pass this in later
	
	data_read = data.ranges
	a = []

	angles1 = [i for i in range(315, 360)]
	angles2 = [i for i in range(45)]
	angles = angles1+angles2

	#print(data_read[270])

	for i in angles:
		
		if data_read[i] < data.range_min:
			continue

		if data_read[i] == float("inf") or data_read[i] == float("-inf"):
			continue
	
		width = data_read[i]*math.sin(math.radians(i))
		height = data_read[i]*math.cos(math.radians(i))
		if width < 0:
			width = width * -1
		if width < w/2:
			a.append([i, height*39.37])

	a.sort(key=lambda x: x[1])

	if len(a) == 0:
		return

#	print(a)

	min_i = a[0][0]
	min_d = a[0][1]

	#print(min_d)

	total = list()
	for k in old_data:
		if t - k > avg:
			continue
		if old_data[k][min_i] != float("inf") and old_data[k][min_i] != float("-inf"):
			val = old_data[k][min_i]*math.cos(math.radians(i))

			if val < 0:
				val = val * -1
			total.append(val)
	if min_d < 100:	
		total.append(min_d)
	#print(len(total))
	avg_d = sum(total) / float(len(total))
	#print(avg_d)

	old_dists[t] = avg_d
	old_data[t] = data_read	

	#print(len(old_data))

	if t > delta_t:
		del old_dists[t-delta_t-1]
		del old_data[t-delta_t-1]
		old_d = old_dists[t-delta_t]
		v_f = v_r + (avg_d - old_d) / delta_t	
		print(int(round(v_f)))

	t = t + 1	

def callback(data):
	
	global d
	global count
	
	data_read = data.ranges[350:359] + data.ranges[1:10]

	lst = list(data_read)
	
	for i in range(len(data_read)):
		if lst[i] == float('Inf'):
			lst[i] = 0

	if d == None:
		d = lst
	else:
	
		#fill in 0s?
		for i in range(len(data_read)):
			if d[i] == 0 and lst[i] != 0:
				d[i] = lst[i]


		if count == 10:
			count = 0
			
			rounded = []			

			for i in range(len(data_read)):
				if d[i] != 0 and lst[i] != 0:
					rounded.append('%.2f' % (d[i] - lst[i]))
				else:
					rounded.append('n/a')	
					
			#diff = np.array(d) - np.array(lst)
			#rounded = ['%.2f' % elem for elem in diff]
			print(rounded)

			d = lst

	count += 1
        #print(data.ranges[350:359]+data.ranges[1:10])

def start():
        # Initialize everything
        #global pub
        #pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
        rospy.Subscriber("scan", LaserScan, callback1)
        rospy.init_node('lidar_valerie')
        rospy.spin()



start()

