#!/usr/bin/env python

import rospy

from race.msg import drive_values
from race.msg import drive_param
from race.msg import drive_angle
from race.msg import drive_speed
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

import tf
import numpy as np

pub = rospy.Publisher('drive_pwm', drive_values, queue_size=10)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
debug_pub = rospy.Publisher("talker_debug", String, queue_size=1)

#inloop_pub = rospy.Publisher('talker_in', drive_param, queue_size=10)
# outloop_pub = rospy.Publisher('talker_out', drive_param, queue_size=10)

global angle
global speed
angle = 0
speed = 0

# global param_angle
# global param_speed
param_angle = 0
param_speed = 0

def arduino_map(x, in_left, in_right, out_left, out_right):
        return (x-in_left)*(out_right-out_left) // (in_right - in_left) + out_left

def callback_angle(data):
        global angle
        angle = data.angle

def callback_speed(data):
        global speed
        speed = data.speed

def get_odom(curr_time, v, theta):
        msg = Odometry()
        # Header: seq, stamp, frame_id
        msg.header.stamp = curr_time
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # pose: pose, position, orientation, covariance
        quat = tf.transformations.quaternion_about_axis(theta,(0,0,1))
        
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.x = quat[2]
        msg.pose.pose.orientation.x = quat[3]
        msg.pose.covariance =   [99999, 0.0,  0.0,  0.0, 0.0,  0.0,
                                 0.0,  99999,  0.0,  0.0, 0.0,  0.0,
                                 0.0,  0.0, 99999,  0.0,  0.0,  0.0,
                                 0.0,  0.0,  0.0,  99999,  0.0,  0.0,
                                 0.0,   0.0,  0.0,  0.0,  99999,  0.0,
                                 0.0,  0.0,  0.0,  0.0,  0.0,  0.1]


        # twist:
        msg.twist.twist.linear.y = v*np.sin(theta)
        msg.twist.twist.linear.x = v*np.cos(theta)
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = theta
        msg.twist.covariance = [0.1, 0.0,  0.0,  0.0, 0.0,  0.0,
                                 0.0,  0.1,  0.0,  0.0, 0.0,  0.0,
                                 0.0,  0.0, 99999,  0.0,  0.0,  0.0,
                                 0.0,  0.0,  0.0,  99999,  0.0,  0.0,
                                 0.0,   0.0,  0.0,  0.0,  99999,  0.0,
                                 0.0,  0.0,  0.0,  0.0,  0.0,  0.1]

        # print 'odometry msg: ', msg
        return msg

def lin_map(val, in_left, in_right, out_left, out_right):
        k = (float(out_right) - out_left)/(in_right - in_left)
        b = out_left - k*in_left
        return k*val+b


def map_velocity(v):
        # unit: m/s
        v_map = 0
        if v > 0:
                if v >= 15.5 and v <= 20:
                        v_map = lin_map(float(v), 15.5, 20.0, 0.0, 0.304)
                        # v_map = arduino_map(float(v), 15.5, 20, 0, 0.304)
                elif v > 20:
                        v_map = 0.304
        elif v < 0:
                if v <= -15.5 and v >= -20:
                        v_map = lin_map(float(v), -20, -15.5, -0.289, 0)
                elif v < -20:
                        v_map = -0.289
        else:
                # v == 0:
                v_map = 0
        return v_map

def map_angle(angle):
        # unit: radians
        
        # In this case: counter-clockwise negative, clockwise positive
        # So correct this by inverting the sign
        ang_map = lin_map(float(angle), -100, 100, -0.431139, 0.431139)
        ang_map = -1*ang_map

        return ang_map

def callback_param(data):
        global param_angle
        global param_speed
        # print("+++++++++++++++++++++++++talker live++++++++++++++++++++++++++++++")
        param_speed = data.velocity
        param_angle = data.angle

# v_prev = map_velocity(0)

def talker():
        global v_prev
        rospy.init_node('serial_talker', anonymous=True)
        em_pub.publish(False)
        rospy.Subscriber("drive_param_fin", drive_param, callback_param)

        #v_prev = v = map_velocity(param_speed)
        curr_time = last_time = rospy.get_rostime()

        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
                # PWM
                pwm1 = arduino_map(param_speed, -100,100,6554,13108);
                pwm2 = arduino_map(param_angle,-100,100,6554,13108);
                
                msg = drive_values()
                msg.pwm_drive = pwm1
                msg.pwm_angle = pwm2
                # print '////////////////////param_speed: %f, param_angle: %f' % (param_speed, param_angle)
                # print '////////////////////pwm1: %f, pwm2: %f' % (msg.pwm_drive, msg.pwm_angle)
                pub.publish(msg)

                # Odometry
                curr_time = rospy.get_rostime()

                dt = float(curr_time.to_sec()) - float(last_time.to_sec())
                debug_pub.publish("time: %f" % dt)
                yaw = map_angle(param_angle)

                debug_pub.publish("yaw: %f" % yaw)

                v = map_velocity(param_speed)
                debug_pub.publish("velocity: %f" % v)
                # dv = v - v_prev
                # debug_pub.publish("dv: %f" % dv)
                # ds = v*dt + 0.5*dv*dt
                # debug_pub.publish("distance inc: %f" % ds)

                odom_msg = get_odom(curr_time, v, yaw)

                # print '////////////////////odom x: %f, odom y: %f' % (odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y)
                # odom_msg = Odometry()

                odom_pub.publish(odom_msg)

                # v_prev = v
                last_time = curr_time
        
                rate.sleep()

        # rate = rospy.Rate(20)

        # while not rospy.is_shutdown():
        #         pwm1 = arduino_map(param_speed, -100,100,6554,13108);
        #         pwm2 = arduino_map(param_angle,-100,100,6554,13108);
        #         #print '////////////////////pwm1: %f, pwm2: %f' % (pwm1, pwm2)
        #         msg = drive_values()
        #         msg.pwm_drive = pwm1
        #         msg.pwm_angle = pwm2
        #         pub.publish(msg)
        
        #         rate.sleep()

if __name__ == '__main__':
        print("Serial talker initialized")
        talker()
