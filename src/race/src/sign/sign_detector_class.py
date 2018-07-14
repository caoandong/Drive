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

from sign_detectorv2 import initialize_sign_detection, cleanup_sign_detection, red_classifier


DEBUG_MSG = False


class SignDetector():
    def __init__(self, FREQ):
        self.frame = None
        self.last_frame_time = datetime.now()
        self.bridge = CvBridge()
        rospy.Subscriber('camera', Image, self.image_callback)
        self.camera_pub = rospy.Publisher("camera_sign", Image, queue_size=10)
        self.classifier = red_classifier()
        self.FREQ = FREQ
        self.rate = rospy.Rate(self.FREQ)

    def image_callback(self, ros_image):
        self.frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        self.last_frame_time = datetime.now()

    def get_curr_frame(self):
        return self.frame

    def start_detecting(self):
        while self.frame is None:
            print 'Waiting for camera input'
            rospy.sleep(1)

        print 'Initializing sign detection network'
        initialize_sign_detection()
        return True

    def stop_detecting(self):
        cleanup_sign_detection()

    def find_signs(self):
        while self.frame is None:
            print 'Waiting for camera input'
            rospy.sleep(1)

        processed_image, sign, dist = self.classifier.classify(self.frame)
        return processed_image, sign, dist    

        

    def loop(self):
        while not rospy.is_shutdown():
            img, sign, distance = self.find_signs()
            print '----------------------------------'
	    if sign is not None and distance is not None:
                print "{} at distance {}".format(sign, distance)
            self.rate.sleep()


if __name__ == '__main__':
        print("sign_detector")
        rospy.init_node('sign_detector')
        sign_detector = SignDetector(FREQ=20)
        sign_detector.start_detecting()
        sign_detector.loop()
        sign_detector.stop_detecting()
        rospy.spin()
