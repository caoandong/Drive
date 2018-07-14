#!/usr/bin/env python

import sys
import os
import datetime
import rospy
from sensor_msgs.msg import Image, Joy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from race.msg import drive_angle, drive_speed

import std_msgs.msg

import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

pub_topic_name = "camera_filtered"
camera1_pub = rospy.Publisher(pub_topic_name,Image,queue_size=1)
bridge = CvBridge()

pub_speed = rospy.Publisher('drive_speed', drive_speed, queue_size=10)
pub_angle = rospy.Publisher('drive_angle', drive_angle, queue_size=10)

stop = False


def get_min_dist(dist):

    global stop, frame_count

    print(dist.data)

    if dist.data < .75:
        stop = True
    elif stop == True:
        frame_count = 0
        stop = False


def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    # return mask

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def get_slope(x1, y1, x2, y2):
    if x1 == x2:
        return float("inf")
    return (y2-y1)/float(x2-x1)

def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    """
    NOTE: this is the function you might want to use as a starting point once you want to
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).
    Think about things like separating line segments by their
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of
    the lines and extrapolate to the top and bottom of the lane.
    This function draws `lines` with `color` and `thickness`.
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    if lines == None:
        return

    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    Compute the hough lines for the given image, returns array of [x1, y1, x2, y2] segments
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    return lines
 
def filter_lines(prev_segments, curr_segments):
    """
    Takes an array of line segments and returns a filtered list of line segments
    """
    pass

def lines_on_image(poslines, img):
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, poslines)
    return line_img

def points_on_image(points, img, size=2):
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    for point in points:
        cv2.circle(line_img, (point[0], point[1]), size, (0, 255, 0), -1)
    return line_img

def weighted_img(img, initial_img, alpha=0.8, beta=1., l=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    `initial_img` should be the image before any processing.
    The result image is computed as follows:
    initial_img * alpha + img * beta + lambda
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, alpha, img, beta, l)

def nn_walk(points):
    """
    Returns an array of (x, y) points in the order they should be connected
    """

    points = [(float(x[0]), float(x[1])) for x in points]
    points.sort(key=lambda x: x[1])

    visited = set()
    retval = list()
    retval.append(points[-1])
    
    curr = points[-1]
    while curr not in visited:
        visited.add(curr)

        if len(retval) == len(points):
            break

        min_dist = 9999999
        min_point = None
        for p in points:
            if p in visited:
                continue

            dist = math.sqrt((curr[0] - p[0])**2 + (curr[1] - p[1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_point = p

        curr = min_point
        if min_dist > 500:
            return retval
        retval.append(curr)

    return retval

def publish_drive(speed, angle):
    print('publishing ' + str(speed) + ', ' + str(angle))
    msg_speed = drive_speed()
    msg_speed.speed = speed
    pub_speed.publish(msg_speed)

    msg_angle = drive_angle()
    msg_angle.angle = angle
    pub_angle.publish(msg_angle)


def on_joy(data):
    global stop

    if data.buttons[1] == 1:
        stop = True

points_agg = list()
frame_count = 0
prev_angle = 0
def on_frame(frame):
    global points_agg, frame_count, stop, prev_angle
    cv_img = bridge.imgmsg_to_cv2(frame, 'bgr8')

    frame_count += 1
    if frame_count > 5 or stop:
       print(frame_count)
       print('stopped')
       publish_drive(0, 0)
       #cv2.imshow('cv_img', cv_img)
       #cv2.waitKey(1)
       return

    # publish_drive(15, -80)

    #msg_speed = drive_speed()
    #msg_speed.speed = 10
    #pub_speed.publish(msg_speed)
    
    #USE THIS
    #publish_drive(15,0)

    # msg_angle = drive_angle()
    # msg_angle.angle = 0
    # msg_angle.publish(msg_angle)

    

    hsv_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    gray_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

    #ORIGINALLY 25-30

    lower_yellow = np.array([30, 100, 100], dtype = 'uint8')
    upper_yellow = np.array([35, 255, 255], dtype='uint8')

    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    mask_white = cv2.inRange(gray_image, 200, 255)
    mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
    mask_yw_image = cv2.bitwise_and(gray_image, mask_yellow)

    #cv2.imshow('cv_img', mask_yw_image)
    #cv2.waitKey(1)
    #mask_yw_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)


    ros_image = bridge.cv2_to_imgmsg(mask_yw_image, 'mono8')
    #camera1_pub.publish(ros_image)
    # frame_count = 0
    # return

    kernel_size = (5,5)
    gauss_gray = cv2.GaussianBlur(mask_yw_image,kernel_size, 0)

    low_threshold = 50
    high_threshold = 150
    canny_edges = cv2.Canny(gauss_gray,low_threshold,high_threshold)

    #cv2.imshow('cv_img', canny_edges)
    #cv2.waitKey(1)

    imshape = canny_edges.shape
    lower_left = [0,imshape[0]]
    lower_right = [imshape[1],imshape[0]]
    top_left = [0,imshape[0]/2]
    top_right = [imshape[1],imshape[0]/2]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    roi_image = region_of_interest(canny_edges, vertices)

    #rho and theta are the distance and angular resolution of the grid in Hough space
    #same values as quiz
    rho = 2
    theta = np.pi/180
    #threshold is minimum number of intersections in a grid for candidate line to go to output
    threshold = 20
    min_line_len = 50
    max_line_gap = 200

    lines = hough_lines(roi_image, rho, theta, threshold, min_line_len, max_line_gap)
    line_image = lines_on_image(lines, roi_image)

    #cv2.imshow('cv_img', line_image)
    #cv2.waitKey(1)

    points = list()
    if lines == None:
        print('no lines; skipping')
        return
    for line in lines:
        for x1, y1, x2, y2 in line:
            points.append([x1, y1])
            points.append([x2, y2])

    points_agg += points

    if frame_count > 3:
        line_image = points_on_image(points_agg, roi_image)
        kmeans = KMeans(n_clusters=min(8, len(points_agg))).fit(points_agg)
        line_image = points_on_image(kmeans.cluster_centers_.astype(int), line_image, size=15)
        # print(kmeans.cluster_centers_.astype(int)[:,0])

        result = weighted_img(line_image, cv_img, alpha=0.8, beta=1., l=0.)

        bounding_path = nn_walk(kmeans.cluster_centers_.astype(int))
        line_segments = [[[int(bounding_path[i][0]), int(bounding_path[i][1]), int(bounding_path[i+1][0]), int(bounding_path[i+1][1])]] for i in range(len(bounding_path)-1)]
        
        draw_lines(result, line_segments)

        # cv2.imshow('cv_img', result)
        # cv2.waitKey(1)

        # Pathfinder algorithm
        LINE_Y = result.shape[0] - 150.0

        # find candidate line segments that hit that y
        candidates = []
        for i in range(len(bounding_path) - 1):
            if (bounding_path[i][1] <= LINE_Y and bounding_path[i + 1][1] >= LINE_Y) or (bounding_path[i][1] >= LINE_Y and bounding_path[i + 1][1] <= LINE_Y):
                candidates.append([bounding_path[i], bounding_path[i + 1]])
                cv2.line(result, (int(bounding_path[i][0]), int(bounding_path[i][1])), (int(bounding_path[i+1][0]), int(bounding_path[i+1][1])), [0, 0, 255], 5)

        slopes = [get_slope(p[0][0], p[0][1], p[1][0], p[1][1]) for p in candidates]

        candidates = [candidates[i] for i in range(len(candidates)) if (not math.isnan(slopes[i])) and slopes[i] != 0 and (not math.isinf(slopes[i]))]
        slopes = [slopes[i] for i in range(len(slopes)) if (not math.isnan(slopes[i])) and slopes[i] != 0 and (not math.isinf(slopes[i]))]
        
        x = list()
        for i in range(len(candidates)):
            x.append(int((LINE_Y - candidates[i][0][1] + slopes[i] * candidates[i][0][0]) / slopes[i]))
        # for a in x:
        #     cv2.line(result, (a, 0), (a, 480), [0, 0, 255], 2)

        #original values
        #left_cap = -100
        #right_cap = result.shape[1] + 100

        left_cap = -1000
        right_cap = result.shape[1] + 1000



        for a in x:
            if a < result.shape[1] / 2:
                if a > left_cap:
                    left_cap = a
            else:
                if a < right_cap:
                    right_cap = a

        #print("left" + str(left_cap))
        #print("right" + str(right_cap))

        # cv2.line(result, (left_cap, 0), (left_cap, 480), [0, 0, 255], 2)
        # cv2.line(result, (right_cap, 0), (right_cap, 480), [0, 0, 255], 2)

        midpoint = (left_cap + right_cap) / 2
        # print (midpoint, int(LINE_Y))
        tri_base = midpoint - 320

        angle = 0
        speed = 15

        #how to make it slowdown.. 

        if len(x) == 0:
            angle = prev_angle
            speed = 17
            print("no lines")
        elif len(x) == 1:
            single_lane = -1 * np.polyfit(kmeans.cluster_centers_.astype(int)[:,0], kmeans.cluster_centers_.astype(int)[:,1], 1)
            angle = np.degrees(np.arctan(single_lane[0]))
            if angle > 0:
                angle = 90 - angle
            elif angle < 0:
                angle = -90 - angle
            speed = 17
            print("one line - angle: " + str(angle))
        elif tri_base != 0:
            angle = np.degrees(np.arctan((480 - LINE_Y) / tri_base))
            if angle > 0:
                angle = 90 - angle
            elif angle < 0:
                angle = -90 - angle
            print("two lines?")
        
        if len(x) != 0:
            prev_angle = angle

        cv2.circle(result, (midpoint, int(LINE_Y)), 10, (0, 0, 255), -1)

        #print(midpoint, int(LINE_Y))

        #print(angle)
        publish_drive(speed, angle)
        #publish_drive(0,0)

        #cv2.imshow('cv_img', result)
        #cv2.waitKey(20)


        ros_image = bridge.cv2_to_imgmsg(result, 'bgr8')
        camera1_pub.publish(ros_image)

        frame_count = 0
        points_agg = list()
    
    return

    slopes = list()
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = get_slope(x1, y1, x2, y2)
            if not math.isinf(slope) and not math.isnan(slope):
                slopes.append(abs(get_slope(x1, y1, x2, y2)))

    # plt.hist(slopes)
    # plt.show()

    result = weighted_img(line_image, cv_img, alpha=0.8, beta=1., l=0.)

    # cv2.arrowedLine(result, (0, 0), (100, 100), (0,0,255), 5)

    ros_image = bridge.cv2_to_imgmsg(result, 'bgr8')
    camera1_pub.publish(ros_image)

def camera_feeder():
    rospy.init_node('camera_filterer', anonymous=True)
    rospy.Subscriber('camera', Image, on_frame)
    rospy.Subscriber('joy', Joy, on_joy)
    rospy.Subscriber("lidar_min_dist", std_msgs.msg.Float32, get_min_dist)
    rospy.spin()

if __name__ == '__main__':
    camera_feeder()
