#!/usr/bin/env python
import numpy as np
import rospy

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

pub = rospy.Publisher('imu_transformed', Imu, queue_size=1)
imu_topic = "imu_topic"

imu_new = Imu()
rospy.init_node('imu_tf')

def scale_linear_accel(x, y, z):
	norm = np.sqrt(x**2+y**2+z**2)
	scale = 9.818/norm
	x_sc = scale*x
	y_sc = scale*y
	z_sc = scale*z
	x_sc = x_sc - 0.1
	y_sc = y_sc + 0.1
	return x_sc, y_sc, z_sc

def scale_ang_vel(x,y,z):
	x_sc = x-0.02
	y_sc = y-0.01
	z_sc = z+0.02
	return x_sc, y_sc, z_sc

def callback_imu(imu):
	global imu_new
	imu_new = imu
	imu_new.header.stamp = rospy.get_rostime()
	imu_new.header.frame_id = "base_link"
	imu_new.orientation.x = -1.0*imu.orientation.y
	imu_new.orientation.y = -1.0*imu.orientation.x
	imu_new.orientation.z = imu.orientation.z
	x = -1.0*imu.linear_acceleration.y
	y = -1.0*imu.linear_acceleration.x
	z = imu.linear_acceleration.z
	x,y,z = scale_linear_accel(x,y,z)
	imu_new.linear_acceleration.x = x
	imu_new.linear_acceleration.y = y
	imu_new.linear_acceleration.z = z
	x = -1.0*imu.angular_velocity.y
	y = -1.0*imu.angular_velocity.x
	z = imu.angular_velocity.z
	x,y,z = scale_ang_vel(x,y,z)
	imu_new.angular_velocity.x = x
	imu_new.angular_velocity.y = y
	imu_new.angular_velocity.z = z

if __name__ == '__main__':
    rospy.Subscriber(imu_topic, Imu, callback_imu)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
    	pub.publish(imu_new)
    	rate.sleep()