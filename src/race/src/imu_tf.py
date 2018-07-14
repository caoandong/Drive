#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

pub = rospy.Publisher('imu_transformed', Imu, queue_size=1)

imu_new = Imu()
rospy.init_node('imu_tf')

def callback_imu(imu):
	global imu_new
	imu_new = imu
	imu_new.header.stamp = rospy.get_rostime()
	imu_new.header.frame_id = "base_link"
	imu_new.orientation.x = imu.orientation.y
	imu_new.orientation.y = imu.orientation.x
	imu_new.orientation.z = -1.0*imu.orientation.z
	imu_new.linear_acceleration.x = imu.linear_acceleration.y
	imu_new.linear_acceleration.y = imu.linear_acceleration.x
	imu_new.linear_acceleration.z = -1.0*imu.linear_acceleration.z
	imu_new.angular_velocity.x = imu.angular_velocity.y
	imu_new.angular_velocity.y = imu.angular_velocity.x
	imu_new.angular_velocity.z = -1.0*imu.angular_velocity.z

if __name__ == '__main__':
    rospy.Subscriber("imu", Imu, callback_imu)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
    	pub.publish(imu_new)
    	rate.sleep()