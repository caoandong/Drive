#!/usr/bin/env python

import sys
import os
import datetime
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np
import time



DIM = (640, 480)
K = np.array(
    [[244.74679589748206, 0.0, 324.10090465649074],
     [0.0, 245.8525640393831, 218.54424425387143],
     [0.0, 0.0, 1.0]]
)
D = np.array(
    [[0.02273244776342948], [-0.3833982766138981],
     [0.7321903621661955], [-0.44154988517309635]]
)
def undistort(frame):
    h, w = frame.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_frame = cv2.remap(
        frame, map1, map2, interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT)
    return undistorted_frame






pub_topic_name = "camera"
camera1_pub = rospy.Publisher(pub_topic_name,Image,queue_size=1)
camera_info_pub = rospy.Publisher("camera_info",CameraInfo,queue_size=1)
bridge = CvBridge()

camera_index = 0
FREQ = 30

def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
 
    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)

def unpesp(img):
    pts1 = np.float32([[90,344],[591,326],[361,220],[292,222]])
    pts2 = np.float32([[216,580],[384,580],[384,72],[216,72]])

    M = cv2.getPerspectiveTransform(pts1,pts2)

    dst = cv2.warpPerspective(img,M,(600,625))

    #plt.subplot(121),plt.imshow(img),plt.title('Input')
    #plt.subplot(122),plt.imshow(dst),plt.title('Output')
    #plt.show()

    return dst

def camera_feeder():
        rospy.init_node('camera_feeder')
        rate = rospy.Rate(FREQ)

        capture = cv2.VideoCapture(camera_index)  
        #TODO: check if capture is open

        #print capture.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        #print capture.get(cv2.CAP_PROP_GAIN)
        #capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

        count = 0
        while not rospy.is_shutdown():
                ret1, frame = capture.read()

                frame = undistort(frame)
                #cv2.imshow("undistort", frame)
                #cv2.waitKey(0)
                #frame = unpesp(frame)
                new_ros_image1 = bridge.cv2_to_imgmsg(frame, "bgr8")
                camera1_pub.publish(new_ros_image1)

                camera_info = CameraInfo()
                camera_info.header.stamp = rospy.Time.from_sec(time.time())
                camera_info.height = 640
                camera_info.width = 480
                camera_info.distortion_model = "plumb_bob"
                camera_info.K = [244.74679589748206, 0.0, 324.10090465649074, 0.0, 245.8525640393831, 218.54424425387143, 0.0, 0.0, 1.0]
                camera_info.D = [0.02273244776342948, -0.3833982766138981, 0.7321903621661955, -0.44154988517309635]
                camera_info.R = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
                camera_info.P = [244.74679589748206, 0.0, 324.10090465649074, 0.0, 0.0, 245.8525640393831, 218.54424425387143, 0.0, 0.0, 0.0, 1.0, 0.0]
                camera_info_pub.publish(camera_info)
                #cv2.imshow("undistort+unpesp", frame)
                #cv2.waitKey(0)

                '''
                frame = adjust_gamma(frame, 2.0)
                name = "%s/%05d.jpg" % ('/home/mkyoon/image_record', count)
                count += 1
                cv2.imwrite(name, frame)
                '''

                rate.sleep()

        capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':

    print(len(sys.argv))
    if len(sys.argv) > 1:
        #camera_index = int(sys.argv[1])
        print("camera_feeder: /dev/video%d, topic=%s" %  (camera_index,pub_topic_name) )
        camera_feeder()
