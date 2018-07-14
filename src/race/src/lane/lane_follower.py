#!/usr/bin/env python

import sys
import os
import datetime
import math
import numpy as np
from datetime import datetime 

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image

from lane_detector import detect

DEBUG_MSG = True


class LaneFollower:

    def __init__(self, FREQ):
        self.frame = None
        self.last_frame_time = datetime.now()
        self.bridge = CvBridge()
        rospy.Subscriber('camera', Image, self.image_callback)
        
        self.camera_pub = rospy.Publisher("camera_lane",Image,queue_size=10)

        self.Kp = 0.25
        self.Ki = 0.0
        self.Kd = 0.01

        self.FREQ = FREQ
        self.dt = 1/float(self.FREQ)
        self.rate = rospy.Rate(self.FREQ)


        self.reset()


    def image_callback(self, ros_image):            
            self.frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.last_frame_time = datetime.now()

    def get_curr_frame(self):
        return self.frame

    def reset(self):        
        self.error = 0
        self.prev_error = 0
        self.integral = 0          

    def start_following(self):
        self.reset()
        
        while self.frame is None:
            print 'Waiting for camera input'
            rospy.sleep(1)
        return True


    def update_pid(self, wider_mask = False):

            if self.frame is None:
                print 'frame is None. This should not happen'
                return None, 0, False
            
            processed_image, CTE, cnt_left, cnt_right = detect(np.copy(self.frame), wider_mask)

            if processed_image is not None:
                self.camera_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
            else:
                self.camera_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            
            if DEBUG_MSG:
                print 'CTE=%d' % CTE
                print '# valid lines=%d, %d' % (cnt_left, cnt_right)


            setpoint = 0    #always want to stay on the center line
            self.error = setpoint - CTE
            self.integral = self.integral + self.error * self.dt
            derivative = (self.error - self.prev_error) / self.dt
            output = self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative
            self.prev_error = self.error


            if DEBUG_MSG:
                print 'output=%d' % (output)

            return processed_image, output, True

    def get_error(self):
        return self.error

    def loop(self):
        while not rospy.is_shutdown():
            img, output, ret = self.update_pid(False)
            if ret:
                new_ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")                
                self.camera_pub.publish(new_ros_image)
                if DEBUG_MSG:
                    print '[GOOD] output %d' % output
            else:
                if DEBUG_MSG:
                    print '[BAD]  output %d' % output
                img = self.frame
            
            cv2.imshow('lane_follower', img)
            cv2.waitKey(1)

            self.rate.sleep()

if __name__ == '__main__':
        print("lane_follower")        
        rospy.init_node('lane_follower')
        lane_follower = LaneFollower(FREQ=20)
        lane_follower.start_following()
        lane_follower.loop()
        rospy.spin()
        
        
        
