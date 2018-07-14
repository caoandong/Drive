#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from marvelmind_nav.msg import hedge_pos

pub = rospy.Publisher('hedge_odom', Odometry, queue_size=1)
rospy.init_node('hedge_tf')

msg = Odometry()

def callback_hedge(pos):
	global msg
	msg.header.stamp = rospy.get_rostime()
	msg.header.frame_id = "odom"
	msg.child_frame_id = "base_link"

	msg.pose.pose.position.x = pos.x_m
	msg.pose.pose.position.y = pos.y_m
	msg.pose.covariance =   [0.004, 0.0,  0.0,  0.0, 0.0,  0.0,
								 0.0,  0.004,  0.0,  0.0, 0.0,  0.0,
								 0.0,  0.0, 99999,  0.0,  0.0,  0.0,
								 0.0,  0.0,  0.0,  99999,  0.0,  0.0,
								 0.0,   0.0,  0.0,  0.0,  99999,  0.0,
								 0.0,  0.0,  0.0,  0.0,  0.0,  99999]

if __name__ == '__main__':
	rospy.Subscriber("/hedge_pos", hedge_pos, callback_hedge)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()
		