#!/usr/bin/env python

import rospy

from race.msg import drive_values
from race.msg import drive_param
from race.msg import drive_angle
from race.msg import drive_speed
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

import tf
import numpy as np
import shapely
from shapely.geometry import Point, LineString, Polygon
from shapely import affinity
import map_gen
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.animation as anim

# Input:
# pose: position and orientation and speed
# output: drive parameters

# choose an action
# if col_free(drive_param) == True:
# publish drive_param
# else:
# pick another drive param


drive_pub = rospy.Publisher('lane_driver', drive_param, queue_size=1)

# A list of things I wish to visualize:
# the car, the orientation, the predicted movement, the FOV, the map

visual_pub_car = rospy.Publisher('lane_driver/car', String, queue_size=1)
visual_pub_FOV = rospy.Publisher('lane_driver/FOV', String, queue_size=1)
visual_pub_pred = rospy.Publisher('lane_driver/pred_car', String, queue_size=1)

#[car_pos, car_ang, orient_x, orient_y, show_FOV, show_pred_car, pred_pos, pred_ang, p0, p1]

max_speed = 20
min_speed = 17
max_angle = 0.431139 # in radian

forward = [max_speed, 0]
back = [-1*max_speed, 0]
turn_speed = min_speed
forward_right_1 = [turn_speed, -1*max_angle/2]
forward_right_2 = [turn_speed, -1*max_angle]
forward_left_1 = [turn_speed, max_angle/2]
forward_left_2 = [turn_speed, max_angle]
back_right_1 = [-1*turn_speed, -1*max_angle/2]
back_right_2 = [-1*turn_speed, -1*max_angle]
back_left_1 = [-1*turn_speed, max_angle/2]
back_left_2 = [-1*turn_speed, max_angle]

drive_param_list = [forward, forward_right_1, forward_right_2, forward_left_1, forward_left_2, 
                    back, back_right_1, back_right_2, back_left_1, back_left_2]

# car = Point([0,0,0])
car_length = 0.5
car_width = 0.3
# distance from mobile beacon to the COM of the car
offset = 0.13 # meters
car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])

driver = drive_param()
driver.velocity = 0
driver.angle = 0

line_slope = [12345.6789,12345.6789,12345.6789]
show_FOV = 0
show_pred_car = 0

if check_intersection() == 1:
        # Near an intersection
        # dot product check
        pt = find_front_pt_in_line()
    elif check_intersection() == 0:
        # Not near an intersection
        # distance check
        for p in right_lane_pts:
            dist = np.array(p) - np.array(car_pos)
            dist = np.linalg.norm(dist)
            if dist < dist_min:
                dist_min = dist
                pt = p
        if dist_min > 0.6:
            update_debug.publish("pred point too far, try dot product")
            pt = find_front_pt_in_line()


    if left_lane_turn == 1:
        # Rotate the car_orient counter-clockwsie by 90 degrees
        orient = [-1*car_orient[1], car_orient[0]]
        test_probe = np.array(left_lane) + np.array(orient)
        test_probe = test_probe/np.linalg.norm(test_probe)
        test_probe = np.array(pt) + test_probe
        test_probe = tuple(test_probe.tolist())
        test_probe = LineString([(pt[0], pt[1]), test_probe])
        update_debug.publish('left test_probe: %s' % str(test_probe))
    elif left_lane_turn == 0:
        # Rotate the car_orient clockwsie by 90 degrees
        orient = [car_orient[1], -1*car_orient[0]]
        orient = np.array(orient)/np.linalg.norm(orient)
        pt1 = np.array(pt) + orient
        orient = -1*np.array(car_orient)
        orient = np.array(orient)/np.linalg.norm(orient)
        pt2 = pt1 + orient
        pt1 = tuple(pt1.tolist())
        pt2 = tuple(pt2.tolist())
        test_probe = LineString([(pt[0], pt[1]), pt1, pt2])
        update_debug.publish('left test_probe: %s' % str(test_probe))

    # Find the left and right lines
    pred_left_lane = 0
    pred_right_lane = 0
    pt = Point(tuple(pt))
    # TODO: expand env_lines to contain all lines within a sphere of the car
    for l in line_map:
        x0, x1 = l.xy[0]
        y0, y1 = l.xy[1]
        if set(((x0, y0),(x1, y1))) == set((tuple(left_lane_pts[0]), tuple(left_lane_pts[1]))):
            continue
        if l.contains(pt):
            pred_left_lane = l
            update_debug.publish('pred_left_lane: %s' % str(pred_left_lane))
        elif (not l.contains(pt)) and (test_probe.intersects(l)):
            pred_right_lane = l
            update_debug.publish('pred_right_lane: %s' % str(pred_right_lane))
    if type(pred_left_lane) == int or type(pred_right_lane) == int:
        update_debug.publish("can't find left/right lines")
        return 0
    left_lane_predict = []
    left_lane_predict.append(pred_left_lane)
    left_lane_predict.append(pred_right_lane)
    update_debug.publish('left_lane_predict: %s' % str(left_lane_predict))

def predict_left_turn(pt):
    global env_lines, left_lane_pts, right_lane_pts, car_pos, car_orient
    ball = Point(pt[0], pt[1]).buffer(1)
    line_list = []
    left_right_set = set([(tuple(left_lane_pts[0]), tuple(left_lane_pts[1])), (tuple(right_lane_pts[0]), tuple(right_lane_pts[1]))])
    p = Point(pt[0],pt[1])
    left_lane = 0
    right_lane = 0
    for l in env_lines:
        if ball.intersects(l):
            x0, x1 = l.xy[0]
            y0, y1 = l.xy[1]
            if bool(set([((x0, y0),(x1, y1))]).intersection(left_right_set)):
                continue
            else:
                line_list.append(l)
    num_line = len(line_list)
    right_line_list = []
    left_vec = []
    if num_line <= 1:
        return [0,0]
    for l in line_list:
        x0, x1 = l.xy[0]
        y0, y1 = l.xy[1]
        p0 = [x0, y0]
        p1 = [x1, y1]
        if pt == p0 or pt == p1:
            left_lane = l
            left_vec = np.array(p1) - np.array(p0)
            line_list.remove(l)
            continue
        dist0 = np.array(p0) - np.array(car_pos)
        dist1 = np.array(p1) - np.array(car_pos)
        check0 = check_cross(np.array(car_orient), dist0)
        check1 = check_cross(np.array(car_orient), dist1)
        if check0 > 0 or check1 > 0:
            right_line_list.append(l)
            continue
    num_line = len(right_line_list)
    if num_line <= 0:
        return [left_lane, 0]
    elif num_line == 1:
        right_lane = right_line_list[0]
    elif num_line > 1:
        for l in right_line_list:
            x0, x1 = l.xy[0]
            y0, y1 = l.xy[1]
            p0 = [x0, y0]
            p1 = [x1, y1]
            vec = np.array(p1) - np.array(p0)
            dot = np.dot(left_vec, vec)
            if abs(abs(dot)-1) < 0.01:
                right_lane = l
                break
    return [left_lane, right_lane]

def check_right_turn():
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global right_lane_turn
    global car_pos
    pt = [0,0]
    dist_min = 99999
    # Check left
    # Find the point closest to the car_pos
    # Make sure to align the left and right lanes before turning

    # Depending on whether or not you are in an intersection
    # Use either distance check or dot product check
    pt = find_front_pt_in_line(right_lane_pts)
    
    # Rotate the car_orient clockwsie by 90 degrees
    orient = [car_orient[1], -1*car_orient[0]]
    test_probe = np.array(right_lane) + np.array(orient)
    test_probe = test_probe/np.linalg.norm(test_probe)
    test_probe = np.array(pt) + test_probe
    test_probe = tuple(test_probe)
    test_probe = LineString([(pt[0], pt[1]), test_probe])
    update_debug.publish('right test_probe: %s' % str(test_probe))

    # Find the left and right lines
    pred_left_lane = 0
    pred_right_lane = 0
    pt = Point(tuple(pt))
    # TODO: expand env_lines to contain all lines within a sphere of the car
    for l in line_map:
        x0, x1 = l.xy[0]
        y0, y1 = l.xy[1]
        if set(((x0, y0),(x1, y1))) == set((tuple(left_lane_pts[0]), tuple(left_lane_pts[1]))):
            continue
        if l.contains(pt):
            pred_left_lane = l
        elif (not l.contains(pt)) and (test_probe.intersects(l)):
            pred_right_lane = l
    if type(pred_left_lane) == int or type(pred_right_lane) == int:
        update_debug.publish("can't find left/right lines")
        return 0
    right_lane_predict = []
    right_lane_predict.append(pred_left_lane)
    right_lane_predict.append(pred_right_lane)
    return right_lane_predict

def check_left_turn():
    global left_lane, left_lane_pts
    global left_lane_turn
    global car_pos
    pt = [0,0]
    dist_min = 99999
    # Check left
    # Find the point closest to the car_pos
    # Make sure to align the left and right lanes before turning

    # Depending on whether or not you are in an intersection
    # Use either distance check or dot product check
    pt = find_front_pt_in_line(left_lane_pts)
    left_lane_predict = predict_left_turn()
    return left_lane_predict

                if (odom_pos == odom_pos_prev) and (imu_accel > 0.05):
                    ell, odom_pos_tmp, odom_orient_tmp = predict_pose_distrib(odom_pos_tmp, odom_orient_tmp)
                    ell_list.append(ell)
                elif odom_pos != odom_pos_prev:
                    contain = 0
                    if len(ell_list) >= 1:
                        localize_debug.publish('number of ell distributions: %d' % len(ell_list))
                        for ell in ell_list:
                            if ell.contains(pt):
                                contain = 1
                                break
                    if (contain == 1) or (len(ell_list) == 0):
                        orient = np.array(car_orient)/np.linalg.norm(car_orient)
                        w0 = 1 # weight of offset
                        w1 = 1 # weight of cam_pos_shift
                        localize_debug.publish('cam_pos_shift: %s' % str(cam_pos_shift))
                        car_pos = np.array(odom_pos) + w0*offset*orient + w1*np.array(cam_pos_shift)
                        car_pos = car_pos.tolist()
                        localize_debug.publish('car_pos updated: %s' % str(car_pos))
                        update_pose_prev_time = update_pose_now_time = rospy.get_time()
                    car_orient = predict_orient(car_orient)
                    odom_pos_prev = odom_pos
                    odom_pos_tmp = car_pos
                    # How to update car_orient
                    odom_orient_tmp = car_orient
                    localize_debug.publish('Revert back to present - pos: %s | orient: %s ' % (str(odom_pos_tmp), str(odom_orient_tmp)))
                    ell_list = []

# Predict what you are going to see after a 90 degree turn
def predict_left_right(left_lane_predict, right_lane_predict):
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global car_pos, car_orient
    global line_map, lane_predict
    global turning, turning_complete, update_toggle

    if turning == 0 and update_toggle[3] == 1:
        if type(left_lane) == list and type(left_lane_pts) == list:
            update_debug.publish("start predicting for left")
            left_lane_predict = check_left_turn()
            if type(left_lane_predict) != int:
                left_check = check_left_right_pred(left_lane_predict[0], left_lane_predict[1])
                update_debug.publish('predict left check: %d' % left_check)
                if left_check <= 1:
                    update_toggle[3] = 1
                else:
                    update_toggle[3] = 0
            else:
                lane_predict[2] = 1
        else:
            lane_predict[2] = 1
        update_debug.publish("right_lane type: %s | right_lane_pts type: %s" % (str(type(right_lane)), str(type(right_lane_pts))))
        if type(right_lane) == list and type(right_lane_pts) == list:
            update_debug.publish("start predicting for right")
            if lane_predict[3] == 1:
                right_lane_predict = check_right_turn()
                if type(right_lane_predict) != int:
                    right_check = check_left_right_pred(right_lane_predict[0], right_lane_predict[1])
                    update_debug.publish('predict right check: %d' % right_check)
                    if right_check <= 1:
                        lane_predict[3] = 1
                    else:
                        lane_predict[1] = right_lane_predict
                        lane_predict[3] = 0
                else:
                    lane_predict[3] = 1
        else:
            lane_predict[3] = 1
    if lane_predict[2] == 0 and lane_predict[3] == 0:
        update_toggle[3] = 0

    update_debug.publish('lane_predict: %s' % str(lane_predict))


            if right_lane_turn == 1:
                    if print_main == 1:
                        print 'Left turn?'
                        print 'check left line: ', cam_left_line
                    if type(cam_left_line) == int:
                        turn_left()
                        return
                elif right_lane_turn == 0:
                    if print_main == 1:
                        print 'T-turn or cross road?'
                    if left_cnt == 0:
                        turn_left()
                        return
                    if right_cnt == 0:
                        turn_right()
                        return
                    if type(cam_left_line) == int:
                        print 'do not see left line'
                        turn_right()
                        return
                    if type(cam_right_line) == int:
                        print 'do not see right line'
                        turn_right()
                        return
                    if type(cam_left_line) != int:
                        print 'can see left line'
                        if type(cam_mid_right_line) != int:
                            print 'can see mid right line'
                            turn_right()
                            return
                    if type(cam_right_line) != int:
                        print 'can see right line'
                        if type(cam_mid_left_line) != int:
                            print 'can see mid left line'
                            turn_left()
                            return
            elif left_lane_turn == 0:
                print 'Right turn?'
                print 'check right line: ', cam_right_line
                if type(cam_right_line) == int:
                    turn_right()
                    return

# Predict the distribution of pose (position and orientation)
# dt later from driver and IMU data
def predict_pose_distrib():
    # Use driver and IMU data to find position reletive to the center line
    # Predict the motion time dt later
    # Make a normal distribution in the predicted odom frame
    # Transform the predicted odom frame to the world frame
    # Given a new coordinate from GPS, find the reliability of that coord

    global car_pos, car_orient
    global driver, turning, pred_distrib, offset
    global update_pose_prev_time, update_pose_now_time, print_local

    dt = 0.002 # Set this manually for now
    localize_debug.publish("turning: %d" % turning)
    localize_debug.publish("dt: %f" % dt)
    if print_local == 1:
        print 'dt: ', dt
        print 'Start predicting the pose to find the distribution'

    speed = fuse_speed()
    ang = fuse_angle()
    if print_local == 1:
        print 'predicted speed: ', speed
        print 'predicted angle: ', ang

    x = car_pos[0]
    y = car_pos[1]
    p = np.array([x,y])
    speed = abs(speed)

    car_ang = find_ang([1,0], car_orient)
    if print_local == 1:
        print 'car angle wrt +x-axis: ', np.degrees(car_ang)

    orient = np.array(car_orient)/np.linalg.norm(car_orient)

    # localize_debug.publish("check turning: %d | angle: %f" % (turning, np.degrees(ang)))

    if (turning != 0) and (abs(ang) > 0.001):
        p = p.reshape([2,1])
        orient = orient.reshape([2,1])

        r = car_length/(2*np.tan(ang))
        r = abs(r)
        if print_local == 1:
            print 'dist to O: ', r

        O_x = x+r*np.sin(car_ang)
        O_y = y-r*np.cos(car_ang)
        O = [O_x, O_y]
        if print_local == 1:
            print 'O: ', O

        O = np.array(O)
        O = O.reshape([2,1])

        p = p - O

        if print_local == 1:
            print 'speed: ', speed

        # ang_vec = np.array([1, np.tan(ang)])
        # cross = check_cross(np.array(car_orient), ang_vec)
        # if cross >= 0:
        if turning == 2:
            localize_debug.publish("Anti-clockwise")
            if print_local == 1:
                print 'Anti-clockwise'
            phi = speed*dt/r
        elif turning == 3:
            localize_debug.publish("clockwise")
            if print_local == 1:
                print 'Clockwise'
            phi = -1*speed*dt/r

        localize_debug.publish("Rotation phi: %f" % np.degrees(phi))
        
        if print_local == 1:
            print 'rotation: ', np.degrees(phi)

        R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
        p = np.matmul(R, p)
        p = p + O
        p = p.reshape([2])
        p = p.tolist()

        orient = np.matmul(R, orient)
        orient = orient.reshape([2])
        car_orient = orient.tolist()
        localize_debug.publish('updated orientation: %s' % str(car_orient))
        
    else:
        p += speed*orient*dt + offset*orient
        p = p.tolist()
        orient = orient.tolist()

    if print_local == 1:
        print 'predicted position: ', p

    dist = np.linalg.norm(np.array(p)-np.array(car_pos))
    
    if print_local == 1:
        print 'distance to original position: ', dist
        
    # x distribution is the horizontal distribution of angles
    x0 = 0.3
    x1 = 0.3*(1-x0)
    x2 = (1-x0)*(1+abs(speed))
    x3 = 5*(1-x0)
    var_x = x0*dist + x1*abs(speed) + x2*abs(ang) + x3*dt 

    # y distribution is the verticle distribution of distances
    y0 = 0.3
    y1 = 1
    y2 = 0.7
    y3 = 5
    var_y = y0*dist + y1*abs(speed) + y2*abs(ang) + y3*dt
    if print_local == 1:
        print 'var_x: ', var_x
        print 'var_y: ', var_y

    ang_off = find_ang([0,1], orient)
    pred_distrib = [p, var_x, var_y, ang_off]

    ell = Point(p[0], p[1]).buffer(1)
    ell = affinity.scale(ell, var_x, var_y)
    ell = affinity.rotate(ell, ang_off, origin='center', use_radians=True)

    localize_debug.publish('speed: %s | angle: %s | pred_distribution_pos: %s | pred_orient_wrt_y: %s | var_x: %s | var_y: %s' 
                            % (str(speed), str(np.degrees(ang)), str(p), str(np.degrees(ang_off)), str(var_x), str(var_y)))

    global imu_vel, imu_ang_vel
    pred_speed_0 = imu_vel
    pred_speed_1 = car_length*imu_ang_vel/np.tan(ang)    
    localize_debug.publish('imu integrated speed: %f | imu predicted speed: %f' % (pred_speed_0, pred_speed_1))
    return ell

def undistort_slope(slope):
    print 'input slope: ', slope
    M = np.array([[-0.24825365335095023, -1.3835463057733923, 360.9934788630403],
                  [-0.09005486236297656, -3.1198655572818654, 712.6915509275204], 
                  [-0.00013159536138092456, -0.0047202178254111115, 1.0]])
    
    p1 = slope[0]
    p1.append(1)
    p1 = np.array(p1).reshape([-1,1])
    print 'p1 vector: ', p1
    p1 = np.matmul(M, p1).reshape([-1]).tolist()
    p1 = [p1[0]/p1[2], -1*p1[1]/p1[2]]

    p2 = slope[1]
    p2.append(1)
    p2 = np.array(p2).reshape([-1,1])
    p2 = np.matmul(M, p2).reshape([-1]).tolist()
    p2 = [p2[0]/p2[2], -1*p2[1]/p2[2]]

    slope_vec = p1 - p0
    print 'slope_vec inverted: ', slope_vec
    slope_new = slope_vec[1]/slope_vec[0]
    print 'slope_new: ', slope_new
    return slope_new

# def get_left_right():
#     global car_orient
#     global car_pos

#     global left_probe
#     global right_probe

#     left_lines = left_line = []
#     right_lines = right_line = []
#     left_lines_pts = []
#     right_lines_pts = []
#     left_line_pts = []
#     right_line_pts = []

#     for line in env_lines:
#         x0, x1 = line.xy[0]
#         y0, y1 = line.xy[1]
#         vec = np.array([x1, y1]) - np.array([x0, y0])
#         vec = vec/(np.linalg.norm(vec))

#         if left_probe.intersects(line):
#             left_lines.append(vec)
#             left_lines_pts.append([[x0,y0],[x1,y1]])
#             if print_main == 1:
#                 print 'found a left line: ', line

#         elif right_probe.intersects(line):
#             right_lines.append(vec)
#             right_lines_pts.append([[x0,y0],[x1,y1]])
#             if print_main == 1:
#                 print 'found a right line: ', line

#     dot_max = -10
#     num_left = len(left_lines)
#     num_right = len(right_lines)

#     if print_main == 1:
#         print 'right: ', right_lines_pts

#     if num_left > 1:
#         for i in range(num_left):
#             line_vec = left_lines[i]
#             dot_orient = np.dot(line_vec, car_orient)/(float(np.linalg.norm(car_orient)))
#             if dot_orient > dot_max:
#                 if print_main == 1:
#                     print 'found a better left line: ', line_vec
#                 dot_max = dot_orient
#                 left_line = line_vec
#                 left_line_pts = left_lines_pts[i]
#     elif num_left == 1:
#         left_line = left_lines[0]
#         left_line_pts = left_lines_pts[0]
#     else:
#         left_line = left_line_pts = [0,0]

#     if num_right > 1:
#         for i in range(num_right):
#             line_vec = right_lines[i]
#             dot_orient = np.dot(line_vec, car_orient)/(float(np.linalg.norm(car_orient)))
#             if dot_orient > dot_max:
#                 if print_main == 1:
#                     print 'found a better right line: ', line_vec
#                 dot_max = dot_orient
#                 right_line = line_vec
#                 right_line_pts = right_lines_pts[i]
#     elif num_right == 1:
#         right_line = right_lines[0]
#         right_line_pts = right_lines_pts[0]
#     else:
#         if num_left != 0:
#             right_line = left_line
#             right_line_pts = left_line_pts
#         else:
#             right_line = right_line_pts = [0,0]

#     if num_left == 0 and num_right != 0:
#         left_line = right_line
#         left_line_pts = right_line_pts

#     if print_main == 1:
#         print 'left_lane_pts: ', left_line_pts
#         print 'right_lane_pts: ', right_line_pts

#     return left_line, right_line, left_line_pts, right_line_pts

def find_orient():
    global car_pos
    global car_ang
    global line_map
    # global line_slope
    global orient
    global show_FOV

    elif left_cnt <= right_cnt:
        # Turn left
        print 'Turning left?'
        turning = 2 # 2 means left
        # Check camera to verify that there's no obstacle on the left
        if cam_left_line == 0:
            print 'Turning left'
            
            driver.velocity = 16
            driver.angle = -100
            drive_pub.publish(driver)
            # reset timer
            prev_time = now_time = rospy.get_time()
            return
        else:
            # Turn right
            print 'Left is no good, how about right?'
            turn_right = 1

    elif right_cnt <= left_cnt or turn_right == 1:
        # Turn right
        print 'Turning right?'
        turning = 3 # 3 means right
        if cam_right_line == 0:
            print 'Turning right'
            
            driver.velocity = 16
            driver.angle = 100
            drive_pub.publish(driver)
            # reset timer
            prev_time = now_time = rospy.get_time()
            return

    # M = np.array([[-0.24825365335095023, -1.3835463057733923, 360.9934788630403],
    #               [-0.09005486236297656, -3.1198655572818654, 712.6915509275204], 
    #               [-0.00013159536138092456, -0.0047202178254111115, 1.0]])

    # # Find the slope of all the lines on the image
    # # Transform the slopes to the map coordinate (? do I have to ?)
    # list_slope = []
    # for slope in line_slope:
    #     slope_vec = np.array([[1], [slope],[1]])
    #     slope_vec = np.matmul(M, slope_vec).tolist()
    #     # Invert the y-coord because the y_axis is pointing downward in an image
    #     slope_vec = [slope_vec[0]/float(slope_vec[2]), -1*slope_vec[1]/float(slope_vec[2])]
    #     slope_new = slope_vec[1]/slope_vec[0]
    #     list_slope.append(slope_new)

    # Based on the location of the car, find the nearest edges in the FOV
    orient_list = []
    orient_list_tmp = []

    print 'initialize FOV'
    FOV = Polygon([(0,0),(0.75,0.59),(0.75,-0.59)])
    FOV = affinity.rotate(FOV, car_ang, origin=(0,0), use_radians=True)
    FOV = affinity.translate(FOV, xoff=car_pos[0], yoff=car_pos[1])
    print FOV

    show_FOV = 1
    visual_pub_FOV.publish("[%d, %f, %f, %f]" % (show_FOV, car_pos[0], car_pos[1], float(car_ang)))

    for line in line_map:
        if FOV.intersects(line):
            print 'FOV intersects: ', line
            orient_list_tmp.append(line)

    ang_min = 10
    diff_min = 10
    num_orient_tmp = len(orient_list_tmp)
    orient_ret = orient

    print 'guessed orient: ', orient

    if num_orient_tmp > 1:
        print 'loop through all nearby lines'
        for line in orient_list_tmp:
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            slope_vec_tmp = np.array([x1-x0,y1-y0])
            # orient_tmp = np.array([np.cos(car_ang), np.sin(car_ang)])
            
            ang_cos_tmp = np.dot(slope_vec_tmp, orient)/(np.linalg.norm(slope_vec_tmp)*np.linalg.norm(orient))
            ang_tmp = np.arccos(ang_cos_tmp)
            if ang_tmp > np.pi/2:
                ang_tmp = ang_tmp - np.pi/2
            print 'angle between this line and guessed orientation: ', np.degrees(ang_tmp)
            if abs(ang_tmp) < diff_min:
                diff_min = abs(ang_tmp)
                print 'found a smaller angle difference: ', ang_tmp, ' the difference is: ', diff_min
                ang_min = ang_tmp
                if ang_cos_tmp < 0:
                    print 'slope_vec is opposite: ', slope_vec_tmp
                    slope_vec_tmp = -1*slope_vec_tmp
                orient_ret = slope_vec_tmp
    elif num_orient_tmp == 1:
        orient_ret = orient_list_tmp[0]

    # ang_offset is the angle between the
    # nearest edges and the car's orientation
    # ang_offset = 0

    
    # if (line_slope[0] != 12345.6789) or (line_slope[1] != 12345.6789) or (line_slope[2] != 12345.6789):
    #     # Find the angle between the y-axis and the verticle lines

    #     # Case 1: at least one verticle line:
    #     if line_slope[0] != 12345.6789:
    #         print 'there exists a left line'
    #         ang_offset_0 = undistort_slope(line_slope[0])
    #         print 'slope of the left_line: ', ang_offset_0
    #         ang_offset_0 = np.arctan2(1, ang_offset_0)
    #         ang_offset = ang_offset_0
    #         print 'angle between left line and the y-axis: ', np.degrees(ang_offset)

    #     if line_slope[2] != 12345.6789:
    #         print 'there exists a right line'
    #         ang_offset_2 = undistort_slope(line_slope[2])
    #         ang_offset_2 = np.arctan2(1, ang_offset_2)
    #         ang_offset = ang_offset_2
    #         print 'ang_offset: ', ang_offset

    #     if line_slope[0] != 12345.6789 and line_slope[2] != 12345.6789:
    #         print 'both left and right lines exist (parallel lines)'
    #         # take the average
    #         ang_offset = (ang_offset_0 + ang_offset_0)/2
    #         print 'ang_offset: ', ang_offset

    #     # Case 2: one horizontal line
    #     elif line_slope[1] != 12345.6789:
    #         print 'Only one horizontal line'
    #         ang_offset = undistort_slope(line_slope[1])
    #         ang_offset = np.arctan2(1, ang_offset)
    #         print 'ang_offset: ', ang_offset

    #     if num_orient_tmp > 1:
    #         print 'loop through all nearby lines'
    #         for line in orient_list_tmp:
    #             x0, x1 = line.xy[0]
    #             y0, y1 = line.xy[1]
    #             slope_vec_tmp = np.array([x1-x0,y1-y0])
    #             # orient_tmp = np.array([np.cos(car_ang), np.sin(car_ang)])
                
    #             ang_cos_tmp = np.dot(slope_vec_tmp, orient)/(np.linalg.norm(slope_vec_tmp)*np.linalg.norm(orient))
    #             ang_tmp = np.arccos(ang_cos_tmp)
    #             print 'angle between this line and guessed orientation: ', ang_tmp
    #             if abs(ang_tmp - ang_offset) < diff_min:
    #                 print 'found a smaller angle difference: ', ang_tmp, ' the difference is: ', diff_min
    #                 diff_min = abs(ang_tmp - ang_offset)
    #                 ang_min = ang_tmp
    #                 if ang_cos_tmp < 0:
    #                     print 'slope_vec is opposite: ', slope_vec_tmp
    #                     slope_vec_tmp = -1*slope_vec_tmp
    #                 orient_ret = slope_vec_tmp
    #     elif num_orient_tmp == 1:
    #         orient_ret = orient_list_tmp[0]

    #     orient_ret = [orient_ret[0]*np.cos(ang_offset)-orient_ret[1]*np.sin(ang_offset), 
    #                   orient_ret[0]*np.sin(ang_offset)+orient_ret[1]*np.cos(ang_offset)]
    #     orient_ret = np.array(orient_ret)
        
        orient_ret = orient_ret/np.linalg.norm(orient_ret)
        print 'orient_ret: ', orient_ret

    # Reset car
    car_ang = np.arctan2(orient_ret[1], orient_ret[0])

    car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                   (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
    # print 'polygon car: ', car
    car = affinity.rotate(car, car_ang, use_radians=True)
    # print 'rotate car: ', car
    car = affinity.translate(car, xoff=car_pos[0], yoff=car_pos[1])

    return orient_ret

line_slope = []

def dist_pt_line(pt, line_pt, line_slope):
    a = np.array(line_pt)
    n = np.array(line_slope)
    p = np.array(pt)
    print 'a: ', a
    print 'n: ', n
    print 'p: ', p
    dist = (a-p)-np.dot(n, (a-p))*n
    return dist.tolist()

def dist_mid_lane():
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global car_pos, car_orient

    mid_pt = (np.array(left_lane_pts[0])+np.array(right_lane_pts[0]))/2.0
    print 'midpoint between left and right: ', mid_pt
    # slope = (np.array(left_lane)+np.array(right_lane))/2.0
    print 'left_lane does not work: ', left_lane, right_lane
    slope = left_lane
    # slope = np.array(left_lane_pts[1])-np.array(left_lane_pts[0])
    # slope = slope/np.linalg.norm(slope)
    # slope = align_vec(slope, car_orient)
    print 'slope between left and right: ', slope

    dist = dist_pt_line(car_pos, mid_pt, slope)
    return dist

def navigate():
    global now_time
    global prev_time
    global car_pos, car_orient, prev_pos
    global left_probe, right_probe, front_probe, turn_left_probe, turn_right_probe
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line
    global env_lines
    global driver, driver_pub
    global turning, turning_complete
    global prev_time, now_time

    print 'Updating probes'
    update_probes()

    if turning == 0:
        print 'number of env_lines: ', len(env_lines)
        for line in env_lines:
            print 'car pose: ', car_pos, car_orient
            print 'front_probe: ', front_probe
            print 'line: ', line
            if not front_probe.intersects(line):
                # Keep moving forward
                driver.velocity = 18
                driver.angle = 0
                drive_pub.publish(driver)
            else:
                # Slow down
                print 'There is a line, slowing down'
                driver.velocity = 15.6
                driver.angle = 0
                drive_pub.publish(driver)
                turning = 1
                break
        if turning == 1:
            left_cnt = 0
            right_cnt = 0
            for line in env_lines:
                if turn_left_probe.intersects(line):
                    left_cnt += 1
                if turn_right_probe.intersects(line):
                    right_cnt += 1
            print 'left intersection count: ', left_cnt
            print 'right intersection count: ', right_cnt
            print 'check mid left line: ', cam_mid_left_line
            print 'check mid right line: ', cam_mid_right_line
            if left_cnt == 0:
                turn_left()
                return
            elif right_cnt == 0:
                turn_right()
                return
            elif type(cam_mid_left_line) != int:
                print 'check mid left line: ', cam_mid_left_line
                turn_left()
                return
            elif type(cam_mid_right_line) != int:
                print 'check mid right line: ', cam_mid_right_line
                turn_right()
                return
            # elif type(cam_mid_line) != int:
            #     print 'Check mid line: ', cam_mid_line
            #     p0 = cam_left_line[0]
            #     p1 = cam_left_line[1]
            #     x0 = p0[0]
            #     x1 = p1[0]
            #     dist_0 = 320 - x0
            #     dist_1 = x1 - 320
            #     # TODO: make a count of the distance (because the slope varies a lot)
            #     print 'distance to middle of the frame: ', dist_0, dist_1
            #     if dist_0 < dist_1:
            #         turn_right()
            #         return
            #     else:
            #         turn_left()
            #         return
            else:
                print 'Cannot decide, slowly moving forward.'
                turning = 0
                driver.velocity = 16
                driver.angle = 0
                drive_pub.publish(driver)
    else:
        print 'Update position and orientation'
        now_time = rospy.get_time()
        dt = now_time - prev_time
        prev_time = now_time
        print 'dt: ', dt
        update_turning(dt)
        print 'Updating probes'
        update_probes()

        if turning != 0:
            # print 'Are we sure about this direction?'
            # if (type(cam_left_line) != int) and (type(cam_mid_right_line) != int) and (type(cam_mid_left_line) == int):
            #     if turning != 3:
            #         global FOV
            #         update_FOV()
            #         FOV_lines = []
            #         for line in env_lines:
            #             FOV_lines.append(line)
            #         if len(FOV_lines) > 1:
            #             for line in FOV_lines:
            #         turn_right()
            #         return
            # elif (type(cam_right_line) != int) and (type(cam_mid_left_line) != int) and (type(cam_mid_right_line) == int):
            #     if turning != 2:
            #         turn_left()
            #         return
            if turning == 2:
                print 'Keep turning left'
                driver.velocity = 20
                driver.angle = -100
                drive_pub.publish(driver)
            elif turning == 3:
                print 'Keep turning right'
                driver.velocity = 20
                driver.angle = 100
                drive_pub.publish(driver)
        # Check whether or not to stop turning
        if type(cam_left_line) != int and type(cam_right_line) != int:
            print 'Going forward ?'
            print 'The camera says: ', cam_left_line, cam_right_line
            # check the slope of the undistorted left and right line

            p0 = cam_left_line[0]
            p1 = cam_left_line[1]
            p0 = undist_cam(p0)
            p1 = undist_cam(p1)
            vec = np.array(p1) - np.array(p0)
            dot = np.dot(vec, np.array([0,1]))/(np.linalg.norm(vec))
            print 'dot of left vec: ', dot
            if abs(dot) > 0.85:
                print 'Go ahead'
                driver.velocity = 16
                driver.angle = 0
                drive_pub.publish(driver)
                turning = 0
                turning_complete = 1
                return

            p0 = cam_right_line[0]
            p1 = cam_right_line[1]
            p0 = undist_cam(p0)
            p1 = undist_cam(p1)
            vec = np.array(p1) - np.array(p0)
            dot = np.dot(vec, np.array([0,1]))/(np.linalg.norm(vec))
            print 'dot of right vec: ', dot
            if abs(dot) > 0.85:
                print 'Go ahead'
                driver.velocity = 16
                driver.angle = 0
                drive_pub.publish(driver)
                turning = 0
                turning_complete = 1
                return

def callback_orient(data):
    global line_slope
    line_slope = eval(data.data)


def build_map(map_path):
    map = open(map_path, 'r')
    counter = 0
    map_lines = []
    for line in map:
        if counter%2 == 0:
            map_pts = eval(line)
        elif counter%2 == 1:
            map_lines = eval(line)
        counter += 1
    lines = []
    for pair in map_lines:
        line_pt = LineString([tuple(map_pts[pair[0]]), tuple(map_pts[pair[1]])])
        lines.append(line_pt)

    return lines


def init_car_1():
    global car
    global init
    global prev_time
    global now_time
    global prev_pos
    global driver
    global car_pos
    global car_ang
    global pos

    # TODO: find the median of the position
    if init == 1:
        # prev_time = now_time = rospy.get_time()

        while now_time - prev_time <= 1:
            # print 'move forward'
            # print 'time diff: ', now_time - prev_time
            driver.velocity = 20
            driver.angle = 0
            drive_pub.publish(driver)

            # print 'moving ... '
            now_time = rospy.get_time()

        print  'stop'
        driver.velocity = 0
        driver.angle = 0
        drive_pub.publish(driver)
        init = 2

def init_car_2():
    global car
    global init
    global prev_time
    global now_time
    global prev_pos
    global driver
    global car_pos
    global car_ang
    global pos

    new_pos = np.array([pos[0],pos[1]])
    print 'poses: ', new_pos, prev_pos
    orient = new_pos - prev_pos
    if np.linalg.norm(orient) <= 0.001:
        print 'Not moving'
        init = 1
        return
    orient = orient/np.linalg.norm(orient)
    print 'init orientation: ', orient
    car_ang = np.arctan(orient[1]/float(orient[0]))
    print 'init angle: ', car_ang

    center = new_pos + offset*orient
    car_pos = center
    angle = car_ang
    car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                   (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
    # print 'polygon car: ', car
    car = affinity.rotate(car, angle, use_radians=True)
    # print 'rotate car: ', car
    car = affinity.translate(car, xoff=center[0], yoff=center[1])
    # print 'translate car: ', car
    print 'car initialized: ', car

    init += 1

def plot_poly(poly):
    global ax
    global line_map
    # global axes

    x,y = poly.exterior.xy
    ax.clear()
    ax.plot(x, y, color='#6699cc', alpha=0.7,
    linewidth=3, solid_capstyle='round', zorder=2)
    plot_line(line_map)
    plt.pause(0.0001)

def predict_car(speed, ang, dt):
    global car
    global car_pos
    global car_ang

    print '-----------------START PREDICTING-----------------------'
    
    x = car_pos[0]
    y = car_pos[1]
    p = np.array([x,y])
    print 'car pos: ', p
    p = p.reshape([2,1])
    speed = abs(speed)

    print 'ang: ', ang
    
    if abs(ang) > 0.001:

        r = car_length/(2*np.tan(ang))
        r = abs(r)
        print 'dist to O: ', r

        O_x = x+r*np.sin(car_ang)
        O_y = y-r*np.cos(car_ang)
        O = [O_x, O_y]
        print 'O: ', O

        O = np.array(O)
        O = O.reshape([2,1])

        p = p - O

        print 'speed: ', speed
        print 'dt: ', dt

    
        phi = -speed*dt/r
        print 'rotation: ', phi

        R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
        p = np.matmul(R, p)
        p = p + O
        p = p.reshape([2])

        print 'pos after rotation: ', p

        sin_val = (O_x-p[0])/r
        cos_val = (p[1]-O_y)/r

        car_ang_f = inv_angle(sin_val, cos_val)
        print 'changed angle: ', car_ang_f

        d_ang = car_ang_f - car_ang
        print 'change of angle: ', d_ang

        x_off = p[0]-x
        y_off = p[1]-y
    else:
        car_ang_f = car_ang
        d_ang = 0

        x_off = speed*dt*np.cos(car_ang)
        y_off = speed*dt*np.sin(car_ang)
    
    print 'offset: ', x_off, y_off
    print 'offset amount: ', np.sqrt(x_off**2 + y_off**2)

    print 'dist_travel: ', speed*dt
    print '-----------------END PREDICTING-----------------------'

    return d_ang, x_off, y_off, p, car_ang_f
    

def col_free(param):
    global car
    global car_pos
    global car_ang
    global pos
    global orient
    global line_map
    global show_pred_car

    front_cnt = 0
    left_cnt = 0
    right_cnt = 0
    for line in env_lines:
        if front_probe.intersects(line):
            front_cnt += 1
        if left_probe.intersects(line):
            left_cnt += 1
        if right_probe.intersects(line):
            right_cnt += 1
    if front_cnt == 0:
        print 'Go ahead'
        driver.velocity = 18
        driver.angle = 0
        drive_pub.publish(driver)
        turning = 0
        turning_complete = 1
    elif left_cnt > 0 or right_cnt > 0:
        print 'Keep turning a bit'
        for num in range(50):
            if turning == 2:
                print 'left'
                driver.velocity = 20
                driver.angle = -100
                drive_pub.publish(driver)
            elif turning == 3:
                print 'right'
                driver.velocity = 20
                driver.angle = 100
                drive_pub.publish(driver)
        print 'Go ahead but be careful'
        driver.velocity = 16
        driver.angle = 0
        drive_pub.publish(driver)
        turning = 0
        turning_complete = 1

    print '----------------start checking collision----------------'

    car_prev = car

    pos = pos + offset*orient

    # for every 0.2 second, check if there is collision
    dt = 0
    x = pos[0]
    y = pos[1]
    speed = param[0]
    show_pred_car = 1

    for t in range(5):
        dt = 1/5.0
        
        speed = map_velocity(param[0])
        ang = map_angle(param[1])
        
        d_ang, x_off, y_off, p, car_ang_f = predict_car(speed, ang, dt)
        print 'predicted result: ', x_off, y_off

        car = affinity.rotate(car, d_ang, origin='centroid', use_radians=True)
        car = affinity.translate(car, xoff=x_off, yoff=y_off)

        visual_pub_pred.publish('[%d,%f,%f,%f]' % (show_pred_car, d_ang, x_off, y_off))

        car_pos = np.array([p[0], p[1]])
        car_ang = car_ang_f

        # plot_poly(car)
        show_pred_car = 0
        visual_pub_pred.publish('[%d,%f,%f,%f]' % (show_pred_car, d_ang, x_off, y_off))

        for line in line_map:
            if car.intersects(line):
                print 'INTERSECT ', line.boundary
                # Revert to previous position
                car = car_prev
                print '----------------end checking collision----------------'
                return 0
    
    # Revert to previous position
    car = car_prev
    print '----------------end checking collision----------------'
    return 1

def update_car():
    global car
    global car_pos
    global car_ang
    global pos
    global orient

    # print '------------update car----------------'

    orient = orient/float(np.linalg.norm(orient))
    center = np.array(pos) + offset*orient
    print 'center: ', center
    print 'car_pos: ', car_pos
    x_off = np.asscalar(center[0] - car_pos[0])
    y_off = np.asscalar(center[1] - car_pos[1])
    print 'offset: ', x_off, y_off
    car_pos = center

    # print 'new car_pos: ', car_pos
    orient_tmp = orient.reshape([-1]).tolist()

    new_ang = np.arctan2(float(orient_tmp[1]), float(orient_tmp[0]))
    d_ang = new_ang - car_ang
    car_ang = new_ang

    # print 'new car_ang: ', car_ang

    car = affinity.rotate(car, d_ang, origin='centroid', use_radians=True)
    car = affinity.translate(car, xoff=x_off, yoff=y_off)

    # plot_poly(car)

    # print '-------------end of update---------------'

def random_walk():
    global driver
    global car
    global drive_param_list
    global orient
    global line_map
    
    driver.velocity = 0
    driver.angle = 0

    for param in drive_param_list:
        if col_free(param) == 1:
            driver.velocity = param[0]
            driver.angle = param[1]
            return


def average_pos(pos):
    num_pos = len(pos)
    x_sum = y_sum = z_sum = 0
    for i in range(num_pos):
        x_sum += pos[i][0]
        y_sum += pos[i][1]
        z_sum += pos[i][2]
    x_avg = x_sum/float(num_pos)
    y_avg = y_sum/float(num_pos)
    z_avg = z_sum/float(num_pos)
    return [x_avg, y_avg, z_avg]

def callback(odom):
    global START
    global init
    global line_map
    global orient
    global prev_pos
    global pos
    global START_WALKING

    # position of the beacon
    # NOT the center of the car

    pos = [odom.pose.pose.position.x,odom.pose.pose.position.y]

    if START == 1:

        if init == 1:
            if prev_pos[0] == 12345.6789 or prev_pos[1] == 12345.6789 :
                prev_pos = np.array([pos[0],pos[1]])
                print '////////////////record position: ', prev_pos
                print 'INIT CAR 1'
                init_car_1()
                
        # elif init == 2:
        #     print 'INIT CAR 2'
        #     pos = [odom.pose.pose.position.x,odom.pose.pose.position.y]
        #     init_car_2()
            
        else:
            print 'Start driving'
            print 'current position: ', pos
            q1 = odom.pose.pose.orientation.x
            q2 = odom.pose.pose.orientation.y
            q3 = odom.pose.pose.orientation.z
            q0 = odom.pose.pose.orientation.w
            q_ang = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))

            orient_new = [orient[0]*np.cos(q_ang)-orient[1]*np.sin(q_ang), 
                      orient[0]*np.sin(q_ang)+orient[1]*np.cos(q_ang)]
            orient_new = np.array(orient_new) - np.array(orient)
            orient += 0.3*orient_new
            print 'current orientation: ', orient

            START_WALKING = 1

def plot_line(lines):
    global ax
    # global axes
    line_pts = []
    for line in lines:
        pts = []
        for p in line.boundary:
            pts.append((p.x,p.y))
        line_pts.append(pts)
    lc = mc.LineCollection(line_pts, linewidths=1)
    ax.add_collection(lc)
    # plt.pause(0.0001)
    # axes.add_collection(lc)

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
        # ang_map = -1*ang_map

        return ang_map

def inv_angle(sin_val, cos_val):
    s1 = np.arcsin(sin_val)
    s3 = np.arccos(cos_val)

    # if sin_val > 0 and cos_val > 0:
    #     assert s1 - s3 < 0.0001
    #     return s1
    # elif sin_val > 0 and cos_val < 0:
    #     return np.pi - s1
    # elif sin_val < 0 and cos_val > 0:
    #     return -s3
    # elif sin_val < 0 and cos_val < 0:
    #     return -s3
    # elif sin_val == 0:
    #     if cos_val == 1:
    #         return 0
    #     elif cos_val == -1:

    s2 = s1
    if s1 > 0:
        s2 = np.pi - s1
    elif s1 < 0:
        s2 = -np.pi + s1
    else:
        s2 = s1
    
    s4 = -s3
    sol = [s1, s2, s3, s4]
    for s in sol:
        if np.sin(s) - sin_val <= 0.00001 and np.cos(s) - cos_val <= 0.00001:
            return s
    print 'NO SOLUTION FOR: ', sin_val, cos_val
    print 'TRIED: ', sol
    return s3

def joy_callback(data):
    global START
    global toggle
    global init
    global prev_time
    global now_time
    
    # RB
    if (data.buttons[5] == 1):
        toggle = (toggle+1)%2

        if toggle == 1:
            START = 1
            init = 1
            prev_pos = np.array([12345.6789,12345.6789])
            print 'turn on'
            prev_time = now_time = rospy.get_time()
        elif toggle == 0:
            START = 0
            START_WALKING = 0
            init = 1
            prev_pos = np.array([12345.6789,12345.6789])
            print 'turn off'

# def update_anim(num):
#     global ax
#     global car
#     global poly

#     x,y = car.exterior.xy
#     print 'update anim: ', x, y
#     poly.set_data(x,y)
#     return poly,

# Extract the camera data to update the orientation
def callback_camera(string):
    global START
    global init
    global car_pos, car_orient
    global left_lane, right_lane
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line
    global turning, turning_complete
    global cam_pos_shift

    # The camera's position on the undistorted image frame, i.e. the bottom
    O = [300, -625]

    if START == 1 and init > 3 and turning == 0:

        print 'left_lane: ', left_lane
        print 'right_lane: ', right_lane

        data = eval(string.data)
        cam_left_line = data[0]
        cam_mid_left_line = data[1]
        cam_mid_line = data[2]
        cam_mid_right_line = data[3]
        cam_right_line = data[4]
        left_ang = mid_ang = right_ang = ang_off = 0
        left_dist = right_dist = 0
        counter = 0

        print 'left line: ', cam_left_line
        print 'right line: ', cam_right_line
        print 'mid left line: ', cam_mid_left_line
        print 'mid right line: ', cam_mid_right_line
        print 'mid line: ', cam_mid_line

        if turning_complete == 1:
            turning_complete = 0
            driver.velocity = 15.6
            driver.angle = 0
            drive_pub.publish(driver)
            print 'slow down, updating the environment'
            update_env()
            update_probes()
            left_lane, right_lane, left_lane_pts, right_lane_pts = get_left_right()
            print 'update the left and right lane: ', left_lane, right_lane
            driver.velocity = 18
            driver.angle = 0
            drive_pub.publish(driver)

        if type(cam_left_line) != int and np.array(left_lane).tolist() != [0,0]:
            p0 = cam_left_line[0]
            p1 = cam_left_line[1]
            p0 = undist_cam(p0)
            print 'p0 undistorted: ', p0
            p1 = undist_cam(p1)
            vec = np.array(p1) - np.array(p0)
            vec = align_vec(vec, np.array([0,1]))
            left_lane = align_vec(np.array(left_lane), np.array(car_orient))
            left_ang = find_ang(vec, left_lane)
            print 'left angle: ', np.degrees(left_ang)

            # Find the distance to the center

            left_dist = abs(300 - p0[0])
            left_dist = float(left_dist)*0.6/50
            print 'left_dist: ', left_dist

            counter += 1
        if type(cam_right_line) != int and np.array(right_lane).tolist() != [0,0]:
            p0 = cam_right_line[0]
            p1 = cam_right_line[1]
            p0 = undist_cam(p0)
            print 'p0 undistorted: ', p0
            p1 = undist_cam(p1)
            vec = np.array(p0) - np.array(p1)
            vec = align_vec(vec, np.array([0,1]))
            right_lane = align_vec(np.array(right_lane), np.array(car_orient))
            right_ang = find_ang(vec, right_lane)
            print 'right_ang: ', np.degrees(right_ang)

            # Find the distance to the center
            right_dist = abs(300 - p1[0])
            right_dist = float(right_dist)*0.6/50
            print 'right_dist: ', right_dist

            counter += 1
        if counter != 0:
            print 'counter: ', counter
            ang_off = (left_ang + right_ang)/float(counter)
            print '-------------------final ang offset: ', np.degrees(ang_off)

            # Update the car's orientation
            orient = [-np.sin(ang_off), np.cos(ang_off)]
            print 'previous orientation: ', car_orient
            car_orient = orient
            print '-------------------camera updated orientation: ', car_orient

            # Update the car's position shift
            if counter == 2:
                print 'predicted car width: ', left_dist + right_dist
                dist = (right_dist - left_dist)/2.0
                print 'distance from middle: ', dist
            elif left_dist != 0:
                dist = car_width/2.0 - left_dist
            elif right_dist != 0:
                dist = right_dist - car_width/2.0
            
            # find the midpoint of the lanes
            dist_vec = dist_mid_lane()
            print 'current position: ', car_pos
            print 'vector to middle of the lane: ', dist_vec
            # Find the vector orthogonal to the car's orientation
            orient = np.array([car_orient[1],-car_orient[0]])
            orient = dist*orient/np.linalg.norm(orient)
            print 'orthogonal direction: ', orient

            print '==========> proposed car_pos: ', np.array(car_pos) + dist_vec + orient
            if abs(left_dist + right_dist) < car_width:
                cam_pos_shift = dist_vec + orient
            
            # TODO: tune the exact coefficients of each vector  

    elif turning != 0:
        data = eval(string.data)
        cam_left_line = data[0]
        cam_mid_left_line = data[1]
        cam_mid_line = data[2]
        cam_mid_right_line = data[3]
        cam_right_line = data[4]


# Obtaining the car's pose
def callback(odom):
    global START
    global init
    global turning, turning_complete

    global odom_pos
    global car_pos, prev_pos
    global car_orient

    if START == 1:
        odom_pos = [odom.pose.pose.position.x,odom.pose.pose.position.y]
        # Initializing stage:
        if init < 2:
            car_pos = odom_pos
        elif init <= 3:
            global prev_pos
            # Updating stage
            vec = np.array(odom_pos) - np.array(prev_pos)
            dist = np.linalg.norm(vec)
            if dist <= 0.75:
                car_pos = odom_pos
        elif turning == 0:
            # Moving straight stage

            global cam_pos_shift
            odom_pos = np.array(odom_pos) + offset*np.array(car_orient)
            print '===========>hedge position: ', odom_pos
            vec = odom_pos - np.array(car_pos)
            dist = np.linalg.norm(vec)
            if dist > 0.001:
                print 'callback odom point distance: ', dist
                vec = vec/float(dist)
                # Ellipse or gaussian?

                if dist <= 1:
                    if turning_complete == 0:
                        try:
                            print 'camera shift: ', cam_pos_shift
                            car_pos = odom_pos + cam_pos_shift
                            car_pos = car_pos.tolist()
                        except:
                            car_pos = odom_pos.tolist()
                    else:
                        car_pos = odom_pos.tolist()
                    print '===========>car_pos updated: ', car_pos
        # Turning stage
        else:
            odom_pos = np.array(odom_pos) + offset*np.array(car_orient)
            dist = odom_pos - np.array(prev_pos)
            dist = np.linalg.norm(dist)
            print 'Turning, restrict update distance: ', dist
            if dist < 1:
                dist_2 = odom_pos - np.array(car_pos)
                dist_2 = np.linalg.norm(dist_2)
                if dist_2 > 0.001:
                    scale = 1/(dist_2+1.0)
                    print 'scaling factor for new position: ', scale
                    car_pos = (1-scale)*np.array(car_pos)+scale*odom_pos
                    car_pos = car_pos.tolist()
                    print 'Updated car_pos: ', car_pos

rospy.init_node('lane_driver', anonymous=True)

START = 0
toggle = 0
START_WALKING = 0

map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
p0 = np.array([2.29, 1.20])
p1 = np.array([3.98, 3.75])
line_map, line_map_plot = map_gen.map_gen(map_path, p0, p1)

print 'map generated: ', line_map


# fig = plt.figure()
# ax = plt.axes(xlim=(-3,5), ylim=(-1,8))
# poly, = ax.plot([], [], color='#6699cc',
#     linewidth=3, solid_capstyle='round')
# plot_line(line_map)

# Question: how to determine the orientation?
orient = np.array([1,0])
car_pos = np.array([0,0])
car_ang = np.arctan(orient[1]/float(orient[0]))

init = 1
prev_time = now_time = rospy.get_time()
prev_pos = np.array([12345.6789,12345.6789])
pos = prev_pos

if __name__ == '__main__':

    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber('odometry/filtered', Odometry, callback)
    rospy.Subscriber('line_cnt/slope', String, callback_orient)

    # print 'START ANIMATION'
    # animation = anim.FuncAnimation(fig, update_anim, frames=25,
    #                            blit=True)
    # plt.pause(0.0001)
    # if init == 1:
    #     init_car(car_pos)
    #     init += 1

    num_cycles = 10
    rate = rospy.Rate(num_cycles)

    while not rospy.is_shutdown():
        visual_pub_car.publish("[%s, %f, %s]" % (str(car_pos.tolist()), car_ang, pred_distrib))
        # plt.pause(0.01)
        # print 'init: ', init, ' START: ', START, ' START_WALKING: ', START_WALKING
        if START == 1:
            # update the car first
            update_car()
            
            if init > 2:
                print 'published driver: ', driver
                drive_pub.publish(driver)
                
                # plt.draw()

            if START_WALKING == 1:
                if init == 2:
                    print 'INIT CAR 2'
                    init_car_2()
                    orient = find_orient()

                    prev_time = rospy.get_time()-1
                

                now_time = rospy.get_time()
                if now_time - prev_time >= 1:
                    print 'Hold on, predicting movement ...'
                    driver.velocity = 0
                    driver.angle = 0
                    drive_pub.publish(driver)
                    random_walk()
                    prev_time = now_time = rospy.get_time()

        # test script:
        # if driver.velocity < 15.5:
        #     driver.velocity = 15.5
        # elif driver.velocity >= 20:
        #     driver.velocity = 20
        # else:
        #     driver.velocity += 1
        # driver.angle = -100
        # driver.angle = driver.angle%(100)

        # drive_pub.publish(driver)

        # vel = map_velocity(driver.velocity)
        # ang = map_angle(driver.angle)

        # print 'velocity: ', vel
        # print 'angle: ', ang

        # now_time = rospy.get_time()
        # dt = now_time - prev_time
        # print 'time diff: ', dt

        # # Updating ---------------------------------------------
        
        # d_ang, x_off, y_off, p, car_ang_f = predict_car(vel, ang, dt)

        # car = affinity.rotate(car, d_ang, origin='centroid', use_radians=True)
        # car = affinity.translate(car, xoff=x_off, yoff=y_off)

        # car_pos = np.array([p[0], p[1]])
        # car_ang = car_ang_f

        # # End of update ----------------------------------------

        # prev_time = rospy.get_time()

        # print 'car pos: ', car_pos

        # if init > 2:
        #     plot_poly(car)

        rate.sleep()
    