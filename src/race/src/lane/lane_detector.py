#!/usr/bin/env python

import sys
import os
import datetime
import rospy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from race.msg import drive_param
from std_msgs.msg import String

import math
import numpy as np
import random

import datetime


DEBUG_MSG = True

frame = None
bridge = CvBridge()

turning = 0
speed = 0
angle = 0

pub_param = rospy.Publisher('selfdrive', drive_param, queue_size=1)
pub_cnt = rospy.Publisher('line_cnt', String, queue_size=1)
slope_pub = rospy.Publisher('line_cnt/slope', String, queue_size=1)

def image_callback(ros_image):
        global frame
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

def mask_edges(img, wider_mask=False):
        mask = np.zeros_like(img)
        ignore_mask_color = 255        
        imshape = img.shape
        #vertices = np.array([[(0,imshape[0]),(0, imshape[0]/2), (imshape[1], imshape[0]/2), (imshape[1],imshape[0])]], dtype=np.int32)
        #vertices = np.array([[(0,imshape[0]),(0,300),(320, 200), (320, 200), (imshape[1], 300), (imshape[1],imshape[0])]], dtype=np.int32)
        
        #vertices = np.array([[(0,imshape[0]),(0,260),(imshape[1], 260), (imshape[1],imshape[0])]], dtype=np.int32)
        #vertices = np.array([[(0,imshape[0]),(0,300),(imshape[1], 300), (imshape[1],imshape[0])]], dtype=np.int32)
        #vertices = np.array([[(0,imshape[0]),(0,280),(imshape[1], 280), (imshape[1],imshape[0])]], dtype=np.int32)

        if wider_mask:
            vertices = np.array([[(0,imshape[0]),(0,280),(imshape[1], 280), (imshape[1],imshape[0])]], dtype=np.int32)
        else:
            vertices = np.array([[(0,imshape[0]),
                (0,280), 
                (imshape[1]/2, 280), 

                #(imshape[1]/8, imshape[0]), 
                (0, imshape[0]), 

                #(imshape[1]*7/8, imshape[0]), 
                (imshape[1], imshape[0]), 

                (imshape[1]/2, 280), 
                (imshape[1], 280), 
                (imshape[1],imshape[0])]], dtype=np.int32)

        cv2.fillPoly(mask, vertices, ignore_mask_color)
        img = cv2.bitwise_and(img, mask)

        #cv2.imshow('m', mask)
        #cv2.waitKey(1)

        return img


def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)


def auto_canny(image, sigma=0.33):
    v = np.median(image)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    return edged


def detect(org_image, wider_mask=False):


        # if DEBUG_MSG:
        #     print '========================'

        #org_image = adjust_gamma(org_image, 1)
        #org_image = adjust_gamma(org_image, 2)
        org_image = adjust_gamma(org_image, 1.5)
      
        
        #convert to grayscale image
        img = cv2.cvtColor(org_image, cv2.COLOR_BGR2GRAY)

        #blurring
        kernel_size = 7
        img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

        
        #Canny edge detector
        #canny_threshold = [70, 200]
        #img = cv2.Canny(img, canny_threshold[0], canny_threshold[1])        
        img = auto_canny(img)

       
        # masked edges
        img = mask_edges(img, wider_mask)

        #cv2.imshow('masked', img)
        #cv2.waitKey(1)
       
        # Hough transform
        rho = 1
        theta = 1*np.pi/180
        #threshold = 30
        #min_line_length = 100   #The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
        #max_line_gap = 10        #The maximum gap between two points to be considered in the same line.
        threshold = 10
        min_line_length = 30   #The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
        max_line_gap = 10      #The maximum gap between two points to be considered in the same line.

        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

        line_image = np.copy(org_image)*0

        
        if lines is None:
            CTE = 0
            img = org_image
                        
            return (img, CTE, 0, 0)
        else:
            left_line_x = []
            left_line_y = []
            right_line_x = []
            right_line_y = []
            mid_line_x = []
            mid_line_y = []
            mid_line_left_x = []
            mid_line_left_y = []
            mid_line_right_x = []
            mid_line_right_y = []

            cnt_left = 0
            cnt_right = 0
            cnt_mid = 0
            cnt_mid_left = 0
            cnt_mid_right = 0

            mid_x = 640/2
            left_mid_x = int(640/3)
            right_mid_x = 2*left_mid_x

            min_x_val = 640
            max_x_val = 0

            for line in lines:
                for x1, y1, x2, y2 in line:

                    if x2 == x1:
                        continue

                    slope = (y2 - y1) / float(x2 - x1)                     

                    distance = np.sqrt( (x2-x1)**2 + (y2-y1)**2 )
                    
                    if distance < 60:
                        #exclude short lines
                        continue

                    
                    abs_slope = math.fabs(slope)                    
                    #if abs_slope < 0.3:
                    if abs_slope > 1.5:                        
                        continue
                    
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, ), 2)
                    cv2.circle(line_image, (x1, y1), 5, (0, 255, 0), -1)
                    cv2.circle(line_image, (x2, y2), 5, (0, 255, 0), -1)



                    if slope <= -0.3:
                        # pub_cnt.publish("left slope: %f" % slope)
                        left_line_x.extend([x1, x2])
                        left_line_y.extend([y1, y2])
                        cnt_left += 1
                    elif slope >= 0.3: 
                        # pub_cnt.publish("right slope: %f" % slope)
                        right_line_x.extend([x1, x2])
                        right_line_y.extend([y1, y2])
                        cnt_right += 1
                    else:
                        min_x_tmp = min(x1, x2)
                        max_x_tmp = max(x1, x2)
                        if min_x_tmp < min_x_val:
                            min_x_val = min_x_tmp
                        if max_x_tmp > max_x_val:
                            max_x_val = max_x_tmp
                        mid_line_x.extend([x1, x2])
                        mid_line_y.extend([y1, y2])
                        cnt_mid += 1

                        if max(x1,x2) < left_mid_x:
                            mid_line_left_x.extend([x1, x2])
                            mid_line_left_y.extend([y1, y2])
                            cnt_mid_left += 1
                        elif min(x1,x2) > right_mid_x:
                            mid_line_right_x.extend([x1, x2])
                            mid_line_right_y.extend([y1, y2])
                            cnt_mid_right += 1
                        

            color_edges = np.dstack((img, img, img))                            
            combo = cv2.addWeighted(org_image, 0.8, line_image, 1, 0)
            img = combo


            min_y = 280
            max_y = img.shape[0] # <-- The bottom of the image

            left_polyfit = None
            right_polyfit = None

            slope_left = slope_mid_left = slope_mid = slope_mid_right = slope_right = "0"

            if cnt_left > 0:
                #do 1d fitting
                left_polyfit = np.polyfit(left_line_y, left_line_x, deg=1)
                poly_left = np.poly1d(left_polyfit)
                left_x_start = int(poly_left(max_y))
                left_x_end = int(poly_left(min_y))

                cv2.line(img, (left_x_start, max_y), (left_x_end, min_y), (0, 0, 255), 5)

                if left_x_start < 0:
                    # find the value of y at x=0:
                    sol = poly_left.roots
                    sol = np.asscalar(sol)
                    slope_left = [[0, sol], [left_x_end, min_y]]
                else:
                    slope_left = [[left_x_start, max_y], [left_x_end, min_y]]
                
                slope_left = str(slope_left)

            if cnt_mid > 0:
                #do 1d fitting
                mid_polyfit = np.polyfit(mid_line_y, mid_line_x, deg=1)
                poly_mid = np.poly1d(mid_polyfit)

                sol_1 = (poly_mid - min_x_val).roots
                sol_1 = np.asscalar(sol_1)
                sol_2 = (poly_mid - max_x_val).roots
                sol_2 = np.asscalar(sol_2)

                slope_mid = [[min_x_val, sol_1],[max_x_val, sol_2]]
                slope_mid = str(slope_mid)

                try:
                    cv2.line(img, (min_x_val, int(sol_1)), (max_x_val, int(sol_2)), (0, 0, 255), 5)
                except:
                    pass

            if cnt_mid_left > 0:
                #do 1d fitting
                mid_left_polyfit = np.polyfit(mid_line_left_y, mid_line_left_x, deg=1)
                poly_left_mid = np.poly1d(mid_left_polyfit)
                mid_left_x_start = int(poly_left_mid(max_y))
                mid_left_x_end = int(poly_left_mid(min_y))

                sol_1 = poly_left_mid.roots
                sol_1 = np.asscalar(sol_1)
                sol_2 = (poly_left_mid - left_mid_x).roots
                sol_2 = np.asscalar(sol_2)

                slope_mid_left = [[0, sol_1],[left_mid_x, sol_2]]
                slope_mid_left = str(slope_mid_left)

                try:
                    if int(sol_1) > max_y:
                        cv2.line(img, (mid_left_x_end, min_y), (left_mid_x, int(sol_2)), (123, 0, 232), 5)
                    else:
                        cv2.line(img, (0, int(sol_1)), (left_mid_x, int(sol_2)), (123, 0, 232), 5)
                except:
                    pass

            if cnt_mid_right > 0:
                #do 1d fitting
                mid_right_polyfit = np.polyfit(mid_line_right_y, mid_line_right_x, deg=1)
                poly_right_mid = np.poly1d(mid_right_polyfit)
                mid_right_x_start = int(poly_right_mid(max_y))
                mid_right_x_end = int(poly_right_mid(min_y))

                sol_1 = (poly_right_mid - right_mid_x).roots
                sol_1 = np.asscalar(sol_1)
                sol_2 = (poly_right_mid - 640).roots
                sol_2 = np.asscalar(sol_2)

                slope_mid_right = [[right_mid_x, sol_1],[640, sol_2]]
                slope_mid_right = str(slope_mid_right)

                try:
                    if int(sol_2) > max_y:
                        cv2.line(img, (right_mid_x, int(sol_1)), (mid_right_x_start, max_y), (232, 0, 143), 5)
                    else:
                        cv2.line(img, (right_mid_x, int(sol_1)), (640, int(sol_2)), (232, 0, 143), 5)
                except:
                    pass

            if cnt_right > 0:
                #do 1d fitting                
                right_polyfit = np.polyfit(right_line_y, right_line_x, deg=1)
                poly_right = np.poly1d(right_polyfit)
                right_x_start = int(poly_right(max_y))
                right_x_end = int(poly_right(min_y))

                cv2.line(img, (right_x_start, max_y), (right_x_end, min_y), (0, 0, 255), 5)

                if right_x_start > 640:
                    # find the value of y at x=640:
                    sol = (poly_right - 640).roots
                    sol = np.asscalar(sol)
                    slope_right = [[right_x_end, min_y],[640, sol]]
                else:
                    slope_right = [[right_x_start, max_y],[right_x_end, min_y]]
                slope_right = str(slope_right)
                
            pub_cnt.publish("mid count: %d" % cnt_mid)
            slope_pub.publish("[%s,%s,%s,%s,%s]" % (slope_left, slope_mid_left, slope_mid, slope_mid_right, slope_right))

            MID_X = img.shape[0]/2
            CTE = 0

            global turning
            global speed
            global angle

            if turning == 0:

                if cnt_left>0 and cnt_right>0:

                    mid_point = (right_x_start+left_x_start)/2
                    error = MID_X - mid_point

                    if DEBUG_MSG:
                        print error

                    CTE = error

                    cv2.line(img, (mid_point, 640), ( (right_x_end+left_x_end)/2, min_y), (180, 180, 0), 5)

                    mask = np.zeros_like(img)                
                    imshape = img.shape                
                    vertices = np.array([[(left_x_start,max_y),(left_x_end, min_y), (right_x_end, min_y), (right_x_start, max_y)]], dtype=np.int32)
                    cv2.fillPoly(mask, vertices, [0,255,0])
                    combo = cv2.addWeighted(img, 0.8, mask, 0.2, 0)
                    img = combo

                    speed = 20
                    angle = 0

                elif cnt_left+cnt_right == 0:
                    if DEBUG_MSG:
                        print 'cannot find any lanes'
                    CTE = 0   #or some special number?
                    
                    if speed > 2:
                        speed -= 2
                                   
                else:
                    if cnt_left == 0:                    
                        if DEBUG_MSG:
                            print 'cannot find left lane'
                        CTE = 400   #just some number

                        speed = 20 
                        angle = -50
                        turning = 1
                    else:
                        
                        if DEBUG_MSG:
                            print 'cannot find right lane'
                        CTE = -400  #just some number

                        speed = 20 
                        angle = 50
                        turning = 1

                cv2.line(img, (MID_X, 640), (MID_X, min_y), (180, 0, 0), 5)

            else:
                pub_cnt.publish("turning")
                if angle > 0:
                    angle += 5
                elif angle < 0:
                    angle -= 5
                if speed > 20:
                    speed -= 2
                elif speed < 15:
                    speed += 2 
                if cnt_left>0 and cnt_right>0:
                    turning = 0

                    mid_point = (right_x_start+left_x_start)/2
                    error = MID_X - mid_point

                    if DEBUG_MSG:
                        print error

                    CTE = error

                    cv2.line(img, (mid_point, 640), ( (right_x_end+left_x_end)/2, min_y), (180, 180, 0), 5)

                    mask = np.zeros_like(img)                
                    imshape = img.shape                
                    vertices = np.array([[(left_x_start,max_y),(left_x_end, min_y), (right_x_end, min_y), (right_x_start, max_y)]], dtype=np.int32)
                    cv2.fillPoly(mask, vertices, [0,255,0])
                    combo = cv2.addWeighted(img, 0.8, mask, 0.2, 0)
                    img = combo

                    speed = 20
                    angle = 0

            # print("+++++++++++++++++++++++++++++++lane_detect live+++++++++++++++++++++++++++++ ")

            msg_param = drive_param()
            msg_param.velocity = speed
            msg_param.angle = angle

            

            pub_param.publish(msg_param)
            
            # if DEBUG_MSG:
                # print '# left  = ', cnt_left
                # print '# right = ', cnt_right

            #camera1_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        return (img, CTE, cnt_left, cnt_right)





def lane_detector():
        rospy.init_node('lane_detector')
        rospy.Subscriber('camera', Image, image_callback)


        rate = rospy.Rate(20)
        while not rospy.is_shutdown():                
                if frame is not None:
                        global speed
                        global angle
                        processed_image, CTE, cnt_left, cnt_right = detect(np.copy(frame), wider_mask=True)
                        cv2.imshow('lane_detector', processed_image)
                        cv2.waitKey(1)

                	#new_ros_image1 = bridge.cv2_to_imgmsg(processed_image, "bgr8")                
                	#camera1_pub.publish(new_ros_image1)

                rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
        print("lane_detector")
        lane_detector()
