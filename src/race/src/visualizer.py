#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from marvelmind_nav.msg import hedge_pos
from marvelmind_nav.msg import beacon_pos_a
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

pub = rospy.Publisher('visualize_hedge', Marker, queue_size=1)
pts = Marker()
lines = Marker()
vecs = Marker()

def init_shapes():
	global pts
	global lines
	global vecs

	pts.header.frame_id = lines.header.frame_id = vecs.header.frame_id = 'odom'
	pts.ns = lines.ns = vecs.ns = 'pts_lines_vecs'
	pts.id = 0
	lines.id = 1
	vecs.id = 2

	vecs.action = 0 # ADD one vector. That's it.

	pts.type = Marker.POINTS
	lines.type = Marker.LINE_STRIP
	vecs.type = Marker.ARROW

	pts.scale.x = 0.1
	pts.scale.y = 0.1
	lines.scale.x = 0.05
	vecs.scale.x = 0.07 # length
	vecs.scale.y = 0.09 # width
	vecs.scale.z = 0.5
	
	pts.color.b = 1.0
	pts.color.a = 1.0
	lines.color.b = 1.0
	lines.color.a = 1.0
	vecs.color.r = 1.0
	vecs.color.a = 1.0

def quat_to_R(q):
	# 1st col
	R00 = q.x**2+q.y**2-q.z**2-q.w**2
	R10 = 2*(q.y*q.z+q.x*q.w)
	R20 = 2*(q.y*q.w-q.x*q.z)

	# 2nd col
	R01 = 2*(q.y*q.z-q.x*q.w)
	R11 = q.x**2-q.y**2+q.z**2-q.w**2
	R21 = 2*(q.z*q.w+q.x*q.y)

	# 3rd col
	R02 = 2*(q.y*q.w+q.x*q.z)
	R12 = 2*(q.z*q.w-q.x*q.y)
	R22 = q.x**2-q.y**2-q.z**2+q.w**2

	R = np.array([[R00, R01, R02],[R10, R11, R12],[R20, R21, R22]])

	return R

def callback_hedge(odom):
	global pts
	global lines
	global vecs

	pts.header.stamp = lines.header.stamp = vecs.header.stamp = rospy.get_rostime()
	
	pts.action = lines.action = 0 # ADD new ones

	# Create vertices for the pts and lines
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	z = odom.pose.pose.position.z
	# print('heard it: ', x, y, z)

	p = Point()
	p.x = x
	p.y = y
	p.z = z

	vecs.pose.position = odom.pose.pose.position
	vecs.pose.orientation = odom.pose.pose.orientation

	# Orientation vector
	# p_array = np.array([x,y,z])
	# q = odom.pose.pose.orientation
	# R = quat_to_R(q)
	# p_orient = np.matmul(R, p_array)
	# p_orient = p_array + p_orient + p_array/np.linalg.norm(p_array)
	# p2 = Point()
	# p2.x = p_orient[0]
	# p2.y = p_orient[1]
	# p2.z = p_orient[2]

	if len(pts.points) > 10:
		del pts.points[0]
	pts.points.append(p)
	if len(lines.points) > 10:
		del lines.points[0]
		del lines.points[0]
	lines.points.append(p)
	# lines.points.append(p2)
	vecs.points.append(p)
	if len(vecs.points) > 2:
		del vecs.points[0]
	# vecs.points = [p, p2]

if __name__ == '__main__':
	rospy.init_node('visualize_rviz')

	init_shapes()

	rospy.Subscriber('/hedge_odom', Odometry, callback_hedge)

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		# print 'orientation: ', vecs.pose.orientation
		# print 'heard pts: ', pts.points[0]
		pub.publish(pts)
		if len(lines.points) >= 2:
			pub.publish(lines)
		# if len(vecs.points) == 2:
		# 	p1 = np.array([vecs.points[1].x,vecs.points[1].y,vecs.points[1].z])
		# 	p0 = np.array([vecs.points[0].x,vecs.points[0].y,vecs.points[0].z])
		# 	p1 = p1+10*(p1-p0)/np.linalg.norm(p1-p0)
		# 	vecs.points[1].x = p1[0]
		# 	vecs.points[1].y = p1[1]
		# 	vecs.points[1].z = p1[2]
		# 	pub.publish(vecs)

		rate.sleep()

	# rospy.spin()