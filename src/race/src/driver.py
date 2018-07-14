#!/usr/bin/env python

import sys
import os
import datetime
import rospy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from race.msg import drive_angle
from race.msg import drive_speed
from race.msg import drive_param
from sensor_msgs.msg import Joy

import math
import numpy as np
import random

from datetime import datetime 

import std_msgs.msg
import curses

stdscr = curses.initscr()

#from subprocess import call

#angle_pub = rospy.Publisher('drive_angle', drive_angle, queue_size=1)
#speed_pub = rospy.Publisher('drive_speed', drive_speed, queue_size=1)
pub_param = rospy.Publisher('driver_control', drive_param, queue_size=1)
pub_toggle = rospy.Publisher('toggle_selfdrive', String, queue_size=10)

is_auto_driving = False
speed = 0
toggle = 0
START = 0

speed_bound = {"lower": -22, "upper": 22, "normal": 18, "turning": 17}       # in [-100,100]
angle_bound = {"lower": -100, "upper": 100}                   # in [-100,100]

def set_speed(new_speed_value):
    global speed
    speed = new_speed_value
    if speed > speed_bound["upper"]:
            speed = speed_bound["upper"]
    elif speed < speed_bound["lower"]:
            speed = speed_bound["lower"]

    msg = drive_speed()
    msg.speed = speed
    speed_pub.publish(msg)   

def set_angle(new_angle_value):
    msg = drive_angle()
    msg.angle = new_angle_value
    if msg.angle < angle_bound["lower"]:
        msg.angle = angle_bound["lower"]
    elif msg.angle > angle_bound["upper"]:
        msg.angle = angle_bound["upper"]
    angle_pub.publish(msg)



def joy_callback(data):
        if (data.buttons[7] == 1):
            #start button
            print 'start button'
            #auto_speed_control(True)

            #change_mode(DrivingMode.lane_following)            
            #move_normal_speed()
                
        elif (data.buttons[0] == 1):
            #Manual mode
            #stop()
            #auto_speed_control(False)
            print 'back button'

        if is_auto_driving is False:
            scale = 0.25
            set_speed(data.axes[1] * 100 * scale)
            set_angle(data.axes[2] * -100)


def lidar_min_dist_callback(data):
    threshold = 0.5
    min_dist = data.data
    #do some processing


def driver():
        rospy.Subscriber("joy", Joy, joy_callback)
        rospy.Subscriber("lidar_min_dist", std_msgs.msg.Float32, lidar_min_dist_callback)
        rospy.init_node('driver')

        FREQ = 20        
        rate = rospy.Rate(FREQ)

        while not rospy.is_shutdown():

            print '**************************************************************'

            ############################################################################
            # do some processing
            ############################################################################

            rate.sleep()

def joy_callback_2(data):
    
    global START
    global toggle
    
    if (data.buttons[7] == 1):
        toggle = (toggle+1)%2
    
        pub_toggle.publish("%d" % toggle)

        if toggle == 1:
            # print("+++++++++++++++++++++++++++++++++++++++++selfdrive live+++++++++++++++++++++++++++++++++")
            # print 'selfdrive start.'
            START = 1
        elif toggle == 0:
            # print("+++++++++++++++++++++++++++++++++++++++++selfdrive stop+++++++++++++++++++++++++++++++++")
            # print 'selfdrive stop'
            START = 0

def selfdrive_control(data):
    global START
    if START == 1:
        msg_param = drive_param()
        msg_param.velocity = data.velocity
        msg_param.angle = data.angle
        pub_param.publish(msg_param)

def driver_2():
    #toggle = 0
    rospy.Subscriber("joy", Joy, joy_callback_2)
    rospy.Subscriber("selfdrive", drive_param, selfdrive_control)
    rospy.init_node('driver')
    stdscr.refresh()
    rospy.spin()


if __name__ == '__main__':
        print("driver started")
        #driver()
        driver_2()
