#!/usr/bin/env python

# Game Plan:
# Phase 1:
# Init the car, update the car's location in real time
# Phase 2:
# Init the FOV, and update the FOV and plot the guessed orientation
# Phase 3:
# While the car is stationary, produce a line that is the optimal path forward
# Phase 4:
# Follow that path
# Phase 5:
# For every N steps, produce a new path based on current position and velocity
# Phase 6:
# For every M steps, check the orientation of the car (ideally in real-time, possibly in another node)

import rospy
import threading

from race.msg import drive_values
from race.msg import drive_param
from race.msg import drive_angle
from race.msg import drive_speed
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Imu

import tf
import numpy as np
import shapely
from shapely.geometry import Point, LineString, Polygon
from shapely import affinity
import map_gen
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.animation as anim
from itertools import combinations
import parse_map

drive_pub = rospy.Publisher('lane_driver', drive_param, queue_size=1)
visual_pub_car = rospy.Publisher('lane_driver/car', String, queue_size=1)

path_pub = rospy.Publisher('lane_driver/path', String, queue_size=1)
update_debug = rospy.Publisher('lane_driver/update_debug', String, queue_size=1)
localize_debug = rospy.Publisher('lane_driver/localize_debug', String, queue_size=1)
orient_debug = rospy.Publisher('lane_driver/orient_debug', String, queue_size=1)
pid_debug = rospy.Publisher('lane_driver/pid_debug', String, queue_size=1)
off_lane_debug = rospy.Publisher('lane_driver/offlane_debug', String, queue_size=1)
# visual_pub_FOV = rospy.Publisher('lane_driver/FOV', String, queue_size=1)

# Parameters of the car, unit in meters
# Car represented as a shapely polygon
car_length = 0.34
car_width = 0.2
offset = 0.13 # meters
max_angle = 0.431139 # in radian
lane_width = 0.6

car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
car_pos = [0,0]
car_orient = [1,0]
car_orient_prev = [1,0]
imu_orient = [1,0] # Inital direction is along the x-axis
car_x_off = car_y_off = car_ang_off = 0.0
ell_list = []

odom_pos = [0,0] # odom_pos is the odometry data, NOT the exact position
odom_pos_prev = [0,0]
odom_pos_tmp = [0,0] 
odom_orient_tmp = [1,0]
odom_pos_1 = [0,0] # odom_pos is the odometry data, NOT the exact position
odom_pos_prev_1 = [0,0]
odom_pos_tmp_1 = [0,0] 
odom_orient_tmp_1 = [1,0]
odom_pos_2 = [0,0] # odom_pos is the odometry data, NOT the exact position
odom_pos_prev_2 = [0,0]
odom_pos_tmp_2 = [0,0] 
odom_orient_tmp_2 = [1,0]
odom_orient_prev = 1j

# Initialize FOV
FOV = Polygon([(0,0),(0.75,0.59),(0.75,-0.59)])
show_FOV = 0
FOV_pos = FOV_pos_new = [0,0]
FOV_orient = FOV_orient_new = [1,0]

# Initialize probes
side_probe_len = 0.7
front_probe_len = 1
shift = 0.25
left_probe = LineString([(0.0, 0.0), (side_probe_len/2.0, np.sqrt(3)/2*side_probe_len), (0, side_probe_len), (-0.5*side_probe_len, np.sqrt(3)/2*side_probe_len), (0.0, 0.0)])
right_probe = LineString([(0.0, 0.0), (side_probe_len/2.0, -1*np.sqrt(3)/2*side_probe_len), (0, -1*side_probe_len), (-0.5*side_probe_len, -1*np.sqrt(3)/2*side_probe_len), (0.0, 0.0)])
front_probe = LineString([(0.0, 0.0), (front_probe_len+shift, 0.0)])
turn_left_probe = LineString([(0.0, 0.0), (0.1+shift, 0.0101), (0.2+shift, 0.0417), (0.3+shift, 0.1), (0.4+shift, 0.2), (0.5+shift, 0.5)])
turn_right_probe = LineString([(0.0, 0.0), (0.1+shift, -0.0101), (0.2+shift, -0.0417), (0.3+shift, -0.1), (0.4+shift, -0.2), (0.5+shift, -0.5)])

# Drive parameters
driver = drive_param()
driver.velocity = 0
driver.angle = 0
P = 300
I = 0
D = 150
#I = -0.15
#D = -100

# Map parameters that determine the size and orientation of the map
# p0 = [3.75, 4.66]
# p1 = [1.26, 2.88]
p0 = [2.41, 0.63]
p1 = [6.59, 1.67]

# Printing options
print_main = 1
print_update = 0
print_local = 0

# Initialize position
prev_pos = [1j,1j]
now_pos = car_pos
init_orient_list = []
init_pos_list = []
cam_pos_shift = [0,0]
car_pos_pred = []

# Initialize turning parameter
left_lane_turn = right_lane_turn = 0

# Initialize control parameter
driver_vel = 0
driver_ang = 0
imu_lin_accel = [0,0]
imu_ang_vel = 0
imu_vel = 0
imu_now_time = 0
imu_prev_time = 0
imu_time_lapse = 0
imu_vel = [0,0]
imu_ang = 0
pred_distrib = [[0,0],0,0,0]

# Initialize PID control errors
e_p = 0 # Cross track error
e_p_prev = 0
e_d = 0 # Cross track error rate
e_i = 0 # Cross track error integral
dist_error = 0
dist_error_prev = 0
ang_error = 0


# Threadings
class Localize (threading.Thread):
    def __init__(self):
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()

    def run(self):
        global START, init
        global lane_width, car_pos, car_orient
        update_debug.publish('Start localize')

        while True:
            if START == 1:
                global imu_lin_accel, odom_pos, odom_pos_prev, odom_pos_tmp, odom_orient_tmp, odom_orient_prev, imu_ang_vel, dt_pred, car_pos_pred, turning
                global odom_pos_1, odom_pos_prev_1, odom_pos_tmp_1, odom_orient_tmp_1, odom_pos_2, odom_pos_prev_2, odom_pos_tmp_2, odom_orient_tmp_2, imu_orient
                #odom_pos = (np.array(odom_pos_1) + np.array(odom_pos_2))/2.0
                odom_pos = 0.8*np.array(odom_pos_1) + 0.2*np.array(odom_pos_2)
                try:
                    if type(odom_pos) == np.ndarray:
                        odom_pos = odom_pos.tolist()
                except:
                    pass
                odom_orient_tmp = np.array(odom_pos_1) - np.array(odom_pos_2)
                odom_orient_tmp_norm = np.linalg.norm(odom_orient_tmp)
                if odom_orient_tmp_norm != 0:
                    odom_orient_tmp = odom_orient_tmp/odom_orient_tmp_norm
                odom_orient_tmp = odom_orient_tmp.tolist()
                localize_debug.publish("odom_orient_tmp: %s" % str(odom_orient_tmp))
                if init < 2:
                    car_pos = odom_pos
                elif init <= 3:
                    global prev_pos
                    # Updating stage
                    vec = np.array(odom_pos) - np.array(prev_pos)
                    dist = np.linalg.norm(vec)
                    if dist <= lane_width:
                        car_pos = odom_pos
                elif init > 3:
                    
                    localize_debug.publish('start localizing ? odom_pos: %s | odom_pos_prev: %s' % (str(odom_pos), str(odom_pos_prev)))
                    if odom_pos != odom_pos_prev:
                        odom_pos_prev = odom_pos
                        orient = np.array(car_orient)/np.linalg.norm(car_orient)
                        w0 = 1 # weight of center offset
                        car_pos_pred = predict_pos(car_pos, car_orient)
                        localize_debug.publish('car_pos_pred: %s | odom_pos: %s' % (str(car_pos_pred), str(odom_pos)))
                        pos_diff = np.array(car_pos_pred) - np.array(odom_pos)
                        pos_diff_norm = np.linalg.norm(pos_diff)
                        pos_weight = gauss_distrib(pos_diff_norm, 0, 1/(2*np.pi), 3)
                        localize_debug.publish('pos_diff_norm: %f | pos_weight: %f' % (pos_diff_norm, pos_weight))
                        car_pos = np.array(odom_pos) + pos_weight*np.array(pos_diff)
                        print 'imu_ang_vel: ', imu_ang_vel
                        car_orient_pred = []
                        if abs(imu_ang_vel) > 0.02:
                            car_orient_pred = predict_orient(car_orient)
                            car_orient_pred = check_orient_pred(car_orient_pred)
                            car_orient = 0.8*np.array(odom_orient_tmp) + 0.2*np.array(car_orient_pred)
                            car_orient = car_orient.tolist()
                            print 'car_orient: ', car_orient, ' car_orient_pred: ', car_orient_pred, ' odom_orient_tmp: ', odom_orient_tmp

                        if type(odom_orient_tmp) == np.ndarray:
                            odom_orient_tmp = odom_orient_tmp.tolist()
                        if type(car_orient_pred) == np.ndarray:
                            car_orient_pred = car_orient_pred.tolist()
                        if type(imu_orient) == np.ndarray:
                            imu_orient = imu_orient.tolist()
                        if type(car_orient) == np.ndarray:
                            car_orient = car_orient.tolist()
                        localize_debug.publish('car_orient_pred: %s | odom_orient_tmp: %s' % (str(car_orient_pred), str(odom_orient_tmp)))
                        orient_debug.publish('[%s, %s, %s]'%(str(odom_orient_tmp),str(car_orient_pred),str(imu_orient)))

class Update (threading.Thread):
    def __init__(self):
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()

    def run(self):
        global START, init
        localize_debug.publish('Start update')
        while True:
            if START == 1:
                update_drive_error()
                check_destination()

# Basic helper functions:
def quat_to_ang(quat):
    # Convert quaternion to angle in radian
    q1 = quat.x
    q2 = quat.y
    q3 = quat.z
    q0 = quat.w
    ang = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))

    return ang

def rotate_by_rad(vec, ang):
    vec_ret = [vec[0]*np.cos(ang)-vec[1]*np.sin(ang), vec[0]*np.sin(ang)+vec[1]*np.cos(ang)]
    return vec_ret

def avg_list(vec_list):
    x_sum = 0
    y_sum = 0
    num = len(vec_list)
    for vec in vec_list:
        x_sum += vec[0]
        y_sum += vec[1]
    x_avg = x_sum/float(num)
    y_avg = y_sum/float(num)
    return [x_avg, y_avg]

# Undistort a point in camera frame
# Frame size after undistortion: (600,625)
def undist_cam(p):
    p_ret = p
    if len(p_ret) > 2:
        p_ret = [p_ret[0], p_ret[1]]

    M = np.array([[-0.24825365335095023, -1.3835463057733923, 360.9934788630403],
                  [-0.09005486236297656, -3.1198655572818654, 712.6915509275204], 
                  [-0.00013159536138092456, -0.0047202178254111115, 1.0]])
    p_ret.append(1)
    p_ret = np.array(p_ret).reshape([-1,1])
    p_ret = np.matmul(M, p_ret).reshape([-1]).tolist()
    p_ret = [p_ret[0]/p_ret[2], -1*p_ret[1]/p_ret[2]]
    # print 'undistorted point: ', p_ret

    return p_ret

# Find the left and the right lane to the car
# TODO: redce the global variable left_lane and right_lane

def list_to_line(l):
    global print_update
    if print_update == 1:
        print 'input to list to line: ', l
    p0 = l[0]
    p1 = l[1]
    line = LineString([(p0[0],p0[1]), (p1[0],p1[1])])
    return line

def cmp_line_list(line_list):
    global car_orient
    line_ret = 0
    dot_max = -99999
    for line in line_list:
        p0 = line[0]
        p1 = line[1]
        vec = np.array(p1) - np.array(p0)
        vec = vec/(np.linalg.norm(vec))
        car_orient = np.array(car_orient)/np.linalg.norm(car_orient)
        dot = np.dot(vec, car_orient)
        if abs(dot) > dot_max:
            dot_max = abs(dot)
            if dot < 0:
                line_ret = [p1, p0]
            line_ret = line
    return line_ret

def check_type(var_list):
    ret_list = []
    for var in var_list:
        if type(var) == np.ndarray:
            var = var.tolist()
        ret_list.append(var)
    return ret_list

def check_lane_distance(p0,p1,direction):
    global car_pos, car_orient
    v0 = np.array(p0) - np.array(car_pos)
    v1 = np.array(p1) - np.array(car_pos)
    dist0 = np.linalg.norm(v0)
    dist1 = np.linalg.norm(v1)
    if dist0 < 0.3:
        if direction == 2 and np.cross(np.array(car_orient), v0) > 0:
            off_lane_debug.publish('pt %s too close to left probe' % str(p0))
            return 0
        elif direction == 3 and np.cross(np.array(car_orient), v0) < 0:
            off_lane_debug.publish('pt %s too close to right probe' % str(p0))
            return 0
    if dist1 < 0.3:
        if direction == 2 and np.cross(np.array(car_orient), v1) > 0:
            off_lane_debug.publish('pt %s too close to left probe' % str(p1))
            return 0
        elif direction == 3 and np.cross(np.array(car_orient), v1) < 0:
            off_lane_debug.publish('pt %s too close to right probe' % str(p1))
            return 0
    return 1

def check_map_poly(poly):
    global car_pos, car_orient
    global turn_right_probe, turn_left_probe, left_lane, right_lane, left_lane_pts, right_lane_pts
    global off_lane, off_lane_toggle
    # check if probes intersect any edge of the polygon
    coords = list(poly.exterior.coords)
    num_pts = len(coords)
    left_cnt = 0
    right_cnt = 0
    if off_lane_toggle == 1:
        center = list(poly.centroid.coords[0])
        pos_vec = np.array(car_pos) - np.array(center)
        cross = np.cross(pos_vec, np.array(car_orient))
        if cross > 0:
            # counter-clockwise
            off_lane_debug.publish('turn right because counter-clockwise and first inside')
            return 3
        else:
            # clockwise
            off_lane_debug.publish('turn left because clockwise and first inside')
            return 2

    for i in range(num_pts):
        p0 = coords[i]
        p1 = coords[(i+1)%(num_pts)]
        line_tmp = LineString([p0,p1])
        left_cnt = 0
        right_cnt = 0
        if turn_left_probe.intersects(line_tmp):
            off_lane_debug.publish('left_probe intersects line: %s' % str(line_tmp))
            left_cnt = 1
        if turn_right_probe.intersects(line_tmp):
            off_lane_debug.publish('right_probe intersects line: %s' % str(line_tmp))
            right_cnt = 1
        if left_cnt == 1 and right_cnt == 1:
            off_lane_debug.publish('not sure')
            break
        if left_cnt == 1 and right_cnt == 0:
            # check distance of the lane
            off_lane_debug.publish('turn left?')
            p0 = list(p0)
            p1 = list(p1)
            check = check_lane_distance(p0,p1,2)
            if check == 1:
                off_lane_debug.publish('turn left becaue left intersects but not right')
                # set left_lane and right_lane to be the same
                vec = np.array(p1) - np.array(p0)
                vec = vec/np.linalg.norm(vec)
                left_lane = vec
                left_lane_pts = [p0, p1]
                right_lane = vec
                right_lane_pts = [p0, p1]
                return 2
        if right_cnt == 1 and left_cnt == 0:
            # check distance of the lane
            off_lane_debug.publish('turn right?')
            p0 = list(p0)
            p1 = list(p1)
            check = check_lane_distance(p0,p1,3)
            if check == 1:
                off_lane_debug.publish('turn right because right intersects but not left')
                # set left_lane and right_lane to be the same
                vec = np.array(p1) - np.array(p0)
                vec = vec/np.linalg.norm(vec)
                left_lane = vec
                left_lane_pts = [p0, p1]
                right_lane = vec
                right_lane_pts = [p0, p1]
                return 3

    # check the car's orientation with
    # respect to the polygon's center
    center = list(poly.centroid.coords[0])
    pos_vec = np.array(car_pos) - np.array(center)
    cross = np.cross(pos_vec, np.array(car_orient))
    if cross > 0:
        # counter-clockwise
        off_lane_debug.publish('turn left because counter-clockwise')
        return 2
        # if left_cnt == 1 and right_cnt == 1:
        #     off_lane_debug.publish('turn left')
        #     return 2
        # else:
        #     off_lane_debug.publish('turn right')
        #     return 3
    else:
        # clockwise
        off_lane_debug.publish('turn right because clockwise')
        return 3
        # if left_cnt == 1 and right_cnt == 1:
        #     off_lane_debug.publish('turn right')
        #     return 3
        # else:
        #     off_lane_debug.publish('turn left')
        #     return 2

def check_off_lane():
    global car_pos, car_orient, lane_predict, env_lines, line_map, map_poly, driver, drive_pub
    global off_lane, off_lane_toggle
    global left_lane,right_lane, left_lines_pts, right_lane_pts

    ball = Point(car_pos[0], car_pos[1]).buffer(0.05)
    poly = map_poly[0]
    if poly.intersects(ball):
        off_lane_debug.publish('car is inside the map')
        for i in range(1, len(map_poly)):
            poly_tmp = affinity.scale(map_poly[i], 0.7, 0.7)
            if poly_tmp.intersects(ball):
                off_lane_debug.publish('car is too far inside: %s' % str(poly_tmp.exterior.coords.xy))
                off_lane_debug.publish('slow down')
                driver.velocity = 15.6
                driver.angle = 0
                drive_pub.publish(driver)
                if off_lane == 0:
                    off_lane_debug.publish('turn on update toggle because first inside')
                    off_lane_toggle = 1
                else:
                    off_lane_debug.publish('keep update toggle off because car still inside')
                    off_lane_toggle = 0
                if left_lane == right_lane:
                    off_lane_debug.publish('continue turning')
                    return
                off_lane = check_map_poly(poly_tmp)
                return
        if off_lane != 0:
            off_lane_debug.publish('back on track, turn off off_lane')
            off_lane = 0
            off_lane_toggle = -1
            return
            # check the car's orientation
            # dot = np.dot(car_orient, left_lane)
            # off_lane_debug.publish('check orientation with lane: %f' % abs(dot))
            # if abs(abs(dot)-1) < 0.8:
            #     off_lane_debug.publish('back on track, turn off off_lane')
            #     off_lane = 0
            #     off_lane_toggle = -1
            #     return
            # else:
            #     off_lane_debug.publish('keep turning')
            #     return
        else:
            off_lane_debug.publish('keep update toggle off because car is on lane')
            off_lane = 0
            off_lane_toggle = 0
            return
    else:
        off_lane_debug.publish('car is outside the map')
        off_lane_debug.publish('slow down')
        driver.velocity = 15.6
        driver.angle = 0
        drive_pub.publish(driver)
        if off_lane == 0:
            off_lane_debug.publish('turn on update toggle because first outside')
            off_lane_toggle = 1
        else:
            off_lane_debug.publish('keep update toggle off because still outside')
            off_lane_toggle = 0
        off_lane = check_map_poly(map_poly[0])
        return


# Check if the car is in the middle of an intersection
def check_intersection():
    global car_pos

    check = 0
    counter = 0
    pt = Point(pt[0], pt[1]).buffer(0.5)
    for line in env_lines:
        if pt.intersects(line):
            counter += 1
    if counter > 2:
        check = 1
        update_debug.publish('Near an intersection')
    if check == 0:
        update_debug.publish('Not near an intersection')
    return check

def find_line_vec(line):
    x0, x1 = line.xy[0]
    y0, y1 = line.xy[1]
    vec = np.array([x1,y1]) - np.array([x0,y0])
    vec = vec/np.linalg.norm(vec)
    return vec

def get_left_right():
    global car_orient
    global car_pos
    global line_map
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global print_update, update_toggle
    global left_probe, right_probe

    if update_toggle[1] == 1 or type(left_lane) == int or type(right_lane) == int:

        left_lines = left_line = []
        right_lines = right_line = []
        left_lines_pts = []
        right_lines_pts = []
        left_line_pts = []
        right_line_pts = []

        update_debug.publish('left_lane before: %s | right_lane before: %s | left_probe: %s | right_probe: %s' 
                      % (str(left_lane), str(right_lane), str(left_probe), str(right_probe)))

        for line in line_map:
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            line_tmp = LineString([(x0, y0), (x1, y1)])
            if left_probe.intersects(line_tmp):
                left_lines_pts.append([[x0, y0], [x1, y1]])
                update_debug.publish('left intersect: %s' % (str(left_lines_pts[-1])))
            elif right_probe.intersects(line_tmp):
                right_lines_pts.append([[x0, y0], [x1, y1]])
                update_debug.publish('right intersect: %s' % (str(right_lines_pts[-1])))

        num_left = len(left_lines_pts)
        num_right = len(right_lines_pts)
        update_debug.publish('num_left: %d | num_right: %d ' % (num_left, num_right))

        if num_left != 0 and num_right != 0:

            if num_left > 1:
                left_line_pts = cmp_line_list(left_lines_pts)
            elif len(left_lines_pts) == 1:
                left_line_pts = left_lines_pts[0]
            left_line = np.array(left_line_pts[1]) - np.array(left_line_pts[0])
            left_line = align_vec(left_line, np.array(car_orient))
            left_line = left_line/np.linalg.norm(left_line)

            if num_right > 1:
                right_line_pts = cmp_line_list(right_lines_pts)
            elif len(right_lines_pts) == 1:
                right_line_pts = right_lines_pts[0]
            right_line = np.array(right_line_pts[1]) - np.array(right_line_pts[0])
            right_line = align_vec(right_line, np.array(car_orient))
            right_line = right_line/np.linalg.norm(right_line)

            update_debug.publish('left_line_pts: %s | right_line_pts: %s' % (str(left_line_pts), str(right_line_pts)))

            # Make sure that the left and right points are different
            if (left_line_pts[0] == right_line_pts[0]) and (left_line_pts[1] == right_line_pts[1]):
                # Reset
                update_debug.publish('left and right pts are the same')
                left_lines = left_line = []
                left_line_pts = []
                right_line_pts = []
                update_toggle[1] = 1
                return
            # Make sure that their slope is correct
            elif (1 - abs(np.dot(left_line, right_line))) > 0.01:
                update_debug.publish('left and right slopes are not the same')
                update_toggle[1] = 1
                return
            else:
                left_lane = left_line
                left_lane_pts = left_line_pts
                if type(left_lane) == np.ndarray:
                    left_lane = left_lane.tolist()
                for i in range(len(left_lane_pts)):
                    pt = left_lane_pts[i]
                    if type(pt) == np.ndarray:
                        left_lane_pts[i] = pt.tolist()
                right_lane = right_line
                right_lane_pts = right_line_pts
                if type(right_lane) == np.ndarray:
                    right_lane = right_lane.tolist()
                for i in range(len(right_lane_pts)):
                    pt = right_lane_pts[i]
                    if type(pt) == np.ndarray:
                        right_lane_pts[i] = pt.tolist()
                update_toggle[1] = 0
                # Issue: not printing left_lane
                update_debug.publish("left_lane: %s "%str(left_lane))
                update_debug.publish('left_lane: %s | left_lane_pts: %s' % (str(left_lane), str(left_lane_pts)))

                update_debug.publish('right_lane: %s | right_lane_pts: %s' % (str(right_lane), str(right_lane_pts)))

                update_debug.publish("left_lane: %s "%str(left_lane))
# Check cross product from vec1 to vec2
def check_cross(vec1, vec2):
    cross = np.cross(np.array(vec1), np.array(vec2))
    return np.asscalar(cross)

# Find the angle from vec1 to vec2
def find_ang(vec1, vec2):
    ang_ret = 0
    if (np.array(vec1).tolist() != 0) and (np.array(vec2).tolist() != 0):
        vec1 = np.array(vec1)/float(np.linalg.norm(vec1))
        vec2 = np.array(vec2)/float(np.linalg.norm(vec2))
        # print 'vec1: ', vec1
        # print 'vec2: ', vec2    
        dot = np.dot(vec1, vec2)

        cross = check_cross(vec1, vec2)

        # print 'cross: ', cross
        if cross >= 0:
            # rotate vec1 counter_clockwise
            # print 'dot: ', dot
            ang_ret = np.arccos(np.clip(dot, -1.0, 1.0))
        else:
            # rotate vec1 clockwise
            # print 'dot: ', dot
            ang_ret = -1*np.arccos(np.clip(dot, -1.0, 1.0))

    return ang_ret

# Find the angle between line_vec and slope_vec
def find_slope_ang(slope_vec):
    global env_lines

    print 'slope vec: ', slope_vec

    dot_max = -10
    ang_ret = 0

    for line in env_lines:
        x0, x1 = line.xy[0]
        y0, y1 = line.xy[1]
        vec = np.array([x1, y1]) - np.array([x0, y0])
        vec = vec/(np.linalg.norm(vec))
        print 'line vec: ', vec
        dot = np.dot(vec, np.array(slope_vec))
        print 'dot: ', dot
        dot = dot/(float(np.linalg.norm(slope_vec)))
        print 'cos_ang: ', dot

        if abs(dot) > dot_max:
            dot_max = dot
            print 'find better: ', dot_max
            if dot < 0:
                # flip the direction of line_vec
                dot = -1*dot
            ang_min = np.arccos(dot)

            slope_vec_3d = slope_vec.tolist()
            slope_vec_3d.append(0)
            slope_vec_3d = np.array(slope_vec_3d)

            line_vec_3d = line_vec.tolist()
            line_vec_3d.append(0)
            line_vec_3d = np.array(line_vec_3d)

            cross = np.cross(slope_vec_3d, line_vec)
            if cross >= 0:
                # rotate slope_vec counter-clockwise
                ang_ret = abs(ang_min)
                print 'ang_ret: ', ang_ret
            else:
                # rotate slope_vec clockwise
                ang_ret = -1*abs(ang_min)
                print 'ang_ret: ', ang_ret

    return ang_ret

# Align vec to target, both in numpy array
def align_vec(vec, target):
    vec = vec/np.linalg.norm(vec)
    target = target/np.linalg.norm(target)
    dot = np.dot(vec, target)
    if dot < 0:
        # Not aligned
        vec = -1*vec
    return vec

# Linear map function
def lin_map(val, in_left, in_right, out_left, out_right):
    k = (float(out_right) - out_left)/(in_right - in_left)
    b = out_left - k*in_left
    return k*val+b

# Map drive_param to velocity
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

# Map drive_param to angle
def map_angle(angle):
        # unit: radians
        
        # In this case: counter-clockwise negative, clockwise positive
        ang_map = lin_map(float(angle), -100, 100, -0.431139, 0.431139)
        # ang_map = -1*ang_map

        return ang_map

# Turn left
def turn_left():
    global turning, driver, drive_pub, tunring_start
    global prev_time, now_time
    global car_pos, car_orient, prev_pos

    print '------------------Turn left!'
    turning = 2 # 2 means left
    turning_start = 1

    driver.velocity = 20
    driver.angle = -100
    drive_pub.publish(driver)
    # reset timer
    prev_time = now_time = rospy.get_time()
    # reset position
    prev_pos = car_pos

# Turn right
def turn_right():
    global turning, driver, drive_pub, tunring_start
    global prev_time, now_time
    global car_pos, car_orient, prev_pos

    print '--------------------Turn right!'
    turning = 3 # 3 means right
    turning_start = 1
    driver.velocity = 20
    driver.angle = 100
    drive_pub.publish(driver)
    # reset timer
    prev_time = now_time = rospy.get_time()
    # reset position
    prev_pos = car_pos

# Find the point in front of the car
def find_front_pt_in_line(line):
    global car_pos, car_orient, print_update
    
    p0 = np.array(line[0])
    p1 = np.array(line[1])
    dist_0 = p0-np.array(car_pos)
    dist_1 = p1-np.array(car_pos)
    # Compare the orientations
    orient = np.array(car_orient)
    dot0 = np.dot(dist_0, orient)
    dot1 = np.dot(dist_1, orient)
    pt = p0
    if dot1 > dot0:
        pt = p1
    update_debug.publish("find front point: %s for line: %s" % (str(pt), str(line)))
    return pt

def find_lane_turn(line):
    global car_pos, car_orient, print_update
    # Find the point in front of the car
    pt = find_front_pt_in_line(line)
    vec = np.array(pt) - np.array(car_pos)
    vec = vec/np.linalg.norm(vec)
    cross = check_cross(car_orient, pt)
    turn = 0 # 0 means turn right
    if cross > 0:
        turn = 1 # 1 means turn left
    return pt, turn

def init_left_probe(pt, line_vec):
    left_vec = np.array([-1*line_vec[1], line_vec[0]])
    left_vec = left_vec/np.linalg.norm(left_vec)
    line_vec = np.array(line_vec)
    line_vec = line_vec/np.linalg.norm(line_vec)
    p0 = tuple(pt)
    p1 = np.array(pt)+line_vec+left_vec
    p1 = tuple(p1.tolist())
    probe = LineString([p0, p1])
    return probe

def init_right_probe(pt, line_vec):
    right_vec = np.array([line_vec[1], -1*line_vec[0]])
    right_vec = right_vec/np.linalg.norm(right_vec)
    line_vec = np.array(line_vec)
    line_vec = line_vec/np.linalg.norm(line_vec)
    p0 = tuple(pt)
    p1 = np.array(pt)+line_vec+right_vec
    p1 = tuple(p1.tolist())
    probe = LineString([p0, p1])
    return probe

def predict_left_turn(pt):
    global left_lane, left_lane_pts, right_lane, right_lane_pts, left_lane_turn, right_lane_turn
    global car_pos, car_orient
    global line_map

    left_right_set = set([(tuple(left_lane_pts[0]), tuple(left_lane_pts[1])), (tuple(right_lane_pts[0]), tuple(right_lane_pts[1]))])
    probe = 0
    left_vec = 0
    pred_left_lane = 0
    pred_right_lane = 0
    right_line_list =[]
    ball = Point(pt[0], pt[1]).buffer(1)
    probe = 0
    for l in line_map:
        x0, x1 = l.xy[0]
        y0, y1 = l.xy[1]
        p0 = [x0, y0]
        p1 = [x1, y1]
        if bool(set([(tuple(p0),tuple(p1))]).intersection(left_right_set)):
            continue
        if tuple(pt) == tuple(p0) or tuple(pt) == tuple(p1):
            left_vec = np.array(p1) - np.array(p0)
            left_vec = left_vec/np.linalg.norm(left_vec)
            pred_left_lane = [p0, p1]
            update_debug.publish('found pred_left_lane: %s' % str(pred_left_lane))
            # pt2 is the point in the line that is not pt
            pt2 = set([tuple(p0), tuple(p1)]) - set([tuple(pt)])
            pt2 = list(list(pt2)[0])
            vec = np.array(pt2) - np.array(car_pos)
            vec = vec/np.linalg.norm(vec)
            cross = np.cross(np.array(car_orient), vec)
            if cross > 0:
                update_debug.publish("left lane: left")
                left_lane_turn = 1
                probe = init_left_probe(pt, left_lane)
            else:
                update_debug.publish("left lane: right")
                left_lane_turn = 0
                probe = init_right_probe(pt, left_lane)
            continue
        if ball.intersects(l):
            right_line_list.append([[x0, y0], [x1, y1]])
    update_debug.publish('right_line_list: %s' % str(right_line_list))
    num_line = len(right_line_list)
    if num_line <= 0:
        return [pred_left_lane, 0]
    elif num_line == 1:
        pred_right_lane = right_line_list[0]
        update_debug.publish('found pred_right_lane: %s' % str(pred_right_lane))
    elif num_line > 1 and type(probe) != int:
        for l in right_line_list:
            if not probe.intersects(l):
                continue
            p0 = l[0]
            p1 = l[1]
            vec = np.array(p1) - np.array(p0)
            vec = vec/np.linalg.norm(vec)
            dot = np.dot(left_vec, vec)
            if abs(abs(dot)-1) < 0.01:
                pred_right_lane = [p0, p1]
                update_debug.publish('found pred_right_lane: %s' % str(pred_right_lane))
                break
    return [pred_left_lane, pred_right_lane]

def predict_right_turn(pt):
    global left_lane, left_lane_pts, right_lane, right_lane_pts, left_lane_turn, right_lane_turn
    global car_pos, car_orient
    global line_map

    left_right_set = set([(tuple(left_lane_pts[0]), tuple(left_lane_pts[1])), (tuple(right_lane_pts[0]), tuple(right_lane_pts[1]))])
    probe = 0
    right_vec = 0
    pred_left_lane = 0
    pred_right_lane = 0
    left_line_list =[]
    ball = Point(pt[0], pt[1]).buffer(1)
    probe = 0
    for l in line_map:
        x0, x1 = l.xy[0]
        y0, y1 = l.xy[1]
        p0 = [x0, y0]
        p1 = [x1, y1]
        if bool(set([(tuple(p0),tuple(p1))]).intersection(left_right_set)):
            continue
        if tuple(pt) == tuple(p0) or tuple(pt) == tuple(p1):
            right_vec = np.array(p1) - np.array(p0)
            right_vec = right_vec/np.linalg.norm(right_vec)
            pred_right_lane = [p0, p1]
            update_debug.publish('found pred_right_lane: %s' % str(pred_right_lane))
            # pt2 is the point in the line that is not pt
            pt2 = set([tuple(p0), tuple(p1)]) - set([tuple(pt)])
            pt2 = list(list(pt2)[0])
            vec = np.array(pt2) - np.array(car_pos)
            vec = vec/np.linalg.norm(vec)
            cross = np.cross(np.array(car_orient), vec)
            if cross > 0:
                update_debug.publish("right lane: left")
                right_lane_turn = 1
                probe = init_left_probe(pt, right_lane)
            else:
                update_debug.publish("right lane: right")
                right_lane_turn = 0
                probe = init_right_probe(pt, right_lane)
            continue
        if ball.intersects(l):
            left_line_list.append([[x0, y0], [x1, y1]])
    update_debug.publish('left_line_list: %s' % str(left_line_list))
    num_line = len(left_line_list)
    if num_line <= 0:
        return [pred_right_lane, 0]
    elif num_line == 1:
        pred_left_lane = left_line_list[0]
        update_debug.publish('found pred_left_lane: %s' % str(pred_left_lane))
    elif num_line > 1 and type(probe) != int:
        for l in left_line_list:
            if not probe.intersects(l):
                continue
            p0 = l[0]
            p1 = l[1]
            vec = np.array(p1) - np.array(p0)
            vec = vec/np.linalg.norm(vec)
            dot = np.dot(right_vec, vec)
            if abs(abs(dot)-1) < 0.01:
                pred_left_lane = [p0, p1]
                update_debug.publish('found pred_left_lane: %s' % str(pred_left_lane))
                break
    return [pred_left_lane, pred_right_lane]

# Check the left and right lines are aligned and that pred_left_lane and left_lane are perp
def check_left_right_pred(line1, line2):
    global car_pos, car_orient
    global left_lane, right_lane
    check = 0
    p0 = line1[0]
    p1 = line1[1]
    vec1 = np.array(p1) - np.array(p0)
    vec1 = vec1/np.linalg.norm(vec1)
    p0 = line2[0]
    p1 = line2[1]
    vec2 = np.array(p1) - np.array(p0)
    vec2 = vec2/np.linalg.norm(vec2)
    dot = np.dot(vec1, np.array(vec2))
    if abs(abs(dot)-1) < 0.01:
        check += 1
    dot = np.dot(np.array(vec1), np.array(left_lane))
    if abs(dot) < 0.1:
        check += 1
    dot = np.dot(np.array(vec2), np.array(right_lane))
    if abs(dot) < 0.1:
        check += 1
    return check

def find_intersection():
    global left_lane_pts, right_lane_pts
    global env_pts, print_update, update_toggle, lane_predict

    if update_toggle[1] == 0 and update_toggle[2] == 1:
        if type(left_lane_pts) != int and type(right_lane_pts) != int:
            # left_pt, left_lane_turn = find_lane_turn(left_lane_pts)
            # right_pt, right_lane_turn = find_lane_turn(right_lane_pts)
            # update_debug.publish('find intersection left: %d | find intersection right: %d' % (left_lane_turn, right_lane_turn))
            left_pt = find_front_pt_in_line(left_lane_pts)
            right_pt = find_front_pt_in_line(right_lane_pts)
            left_lane_predict = predict_left_turn(left_pt)
            right_lane_predict = predict_right_turn(right_pt)
            update_debug.publish('left_lane_predict: %s | right_lane_predict: %s' % (str(left_lane_predict), str(right_lane_predict)))
            if type(left_lane_predict[0]) == list and type(left_lane_predict[1]) == list and type(right_lane_predict[0]) == list and type(right_lane_predict[1]) == list:
                lane_predict = [left_lane_predict, right_lane_predict]
                left_check = check_left_right_pred(left_lane_predict[0], left_lane_predict[1])
                right_check = check_left_right_pred(right_lane_predict[0], right_lane_predict[1])
                update_debug.publish("left_check: %d | right_check: %d" % (left_check, right_check))
                if left_check > 1 and right_check > 1:
                    update_toggle[2] = 0

def exist_front_corner():
    global car_pos, car_orient
    global front_probe
    global env_lines


    orient = np.array(car_orient)
    orient = orient/np.linalg.norm(orient)
    pt = np.array(car_pos) + orient
    pt = pt.tolist()
    if print_main == 1:
        print 'front corner test point: ', pt

    intersection = []

    pt = Point(pt[0], pt[1]).buffer(0.3)
    for line in env_lines:
        if pt.intersects(line):
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            vec = np.array([x1, y1]) - np.array([x0, y0])
            vec = vec.tolist()
            intersection.append(vec)

    for pair in combinations(intersection, 2):
        dot = np.dot(np.array(pair[0]), np.array(pair[1]))
        if abs(dot) < 0.0001:
            return 1

    return 0

def dist_pt_line(pt, line_pt, line_slope):
    a = np.array(line_pt)
    n = np.array(line_slope)
    p = np.array(pt)
    if print_update == 1:
        print 'a: ', a
        print 'n: ', n
        print 'p: ', p
    dist = (a-p)-np.dot(n, (a-p))*n
    return dist.tolist()

def dist_mid_lane():
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global car_pos, car_orient
    if type(left_lane_pts) == list and type(right_lane_pts) == list:
        mid_pt = (np.array(left_lane_pts[0])+np.array(right_lane_pts[0]))/2.0
        pid_debug.publish('midpoint between left and right: %s' % str(mid_pt))
        slope = left_lane
        pid_debug.publish('slope between left and right: %s' % str(slope))
        dist = dist_pt_line(car_pos, mid_pt, slope)
        return dist
    return

def find_line_eq(line_pts):
    p0 = line_pts[0]
    p1 = line_pts[1]
    if p1[0] == p0[0]:
        return 1j,1j
    m = (float(p1[1]) - p0[1])/(p1[0] - p0[0])
    b = p0[1] - m*p0[0]
    return m,b

def find_dist_lines(line_pts1, line_pts2):
    m1,b1 = find_line_eq(line_pts1)
    m2,b2 = find_line_eq(line_pts2)
    if m1 == 1j or m2 == 1j:
        dist = abs(line_pts1[0][0] - line_pts2[0][0])
        return dist
    dist = abs(b2-b1)/np.sqrt(m1**2+1)
    return abs(dist)

# Make sure that the left and right lane makes sense
def check_lane_predict(left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp):
    global car_pos
    global turning, update_toggle

    if turning == 0:
        # Check if lane_predict is correct:
        # return 1 if everything checks out
        # return 0 if one of the checks failed

        # check if two lanes are parallel
        dot = np.dot(left_lane_tmp, right_lane_tmp)
        update_debug.publish('dot between left and right: %f' % dot)
        if abs(abs(dot) - 1) > 0.01: 
            return 0

        # check if two lanes are close to each other
        dist = find_dist_lines(left_lane_pts_tmp, right_lane_pts_tmp)
        update_debug.publish('distance between left and right lane: %f' % dist)
        if dist > 1:
            return 0

        # check if two lanes are close to the car's position
        left_line = LineString([tuple(left_lane_pts_tmp[0]), tuple(left_lane_pts_tmp[1])])
        right_line = LineString([tuple(right_lane_pts_tmp[0]), tuple(right_lane_pts_tmp[1])])
        ball = Point(car_pos[0], car_pos[1]).buffer(0.6)
        if ball.intersects(left_line) or ball.intersects(right_line):
            # Don't have to update left right until next turning
            update_toggle[1] == 0
            return 1
        else:
            return 0
    return 0

def get_left_right_vec(lanes):
    p0 = lanes[0][0]
    p1 = lanes[0][1]
    left_vec = np.array(p1) - np.array(p0)
    left_lane_tmp = left_vec/np.linalg.norm(left_vec)
    left_lane_pts_tmp = [p0, p1]
    p0 = lanes[1][0]
    p1 = lanes[1][1]
    right_vec = np.array(p1) - np.array(p0)
    right_lane_tmp = right_vec/np.linalg.norm(right_vec)
    right_lane_pts_tmp = [p0, p1]
    return left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp

def update_left_right(turn):
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global lane_predict, update_toggle

    if len(lane_predict[0][0]) == 2 and len(lane_predict[0][1]) == 2 and len(lane_predict[1][0]) == 2 and len(lane_predict[1][1]) == 2:
        if turn == 2:
            # Turn left:
            lanes = lane_predict[0]
            left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp = get_left_right_vec(lanes)
            left_check = check_lane_predict(left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp)
            update_debug.publish('left_check: %d' % left_check)
            print 'left_check: ', left_check
            if left_check == 1:
                left_lane = left_lane_tmp
                left_lane_pts = left_lane_pts_tmp
                right_lane = right_lane_tmp
                right_lane_pts = right_lane_pts_tmp
            else:
                lanes = lane_predict[1]
                left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp = get_left_right_vec(lanes)
                right_check = check_lane_predict(left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp)
                update_debug.publish('right_check: %d' % right_check)
                print 'right_check: ', right_check
                if right_check == 1:
                    left_lane = left_lane_tmp
                    left_lane_pts = left_lane_pts_tmp
                    right_lane = right_lane_tmp
                    right_lane_pts = right_lane_pts_tmp
        elif turn == 3:
            # Turn right:
            lanes = lane_predict[1]
            left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp = get_left_right_vec(lanes)
            right_check = check_lane_predict(left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp)
            update_debug.publish('right_check: %d' % right_check)
            print 'right_check: ', right_check
            if right_check == 1:
                left_lane = left_lane_tmp
                left_lane_pts = left_lane_pts_tmp
                right_lane = right_lane_tmp
                right_lane_pts = right_lane_pts_tmp
            else:
                lanes = lane_predict[0]
                left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp = get_left_right_vec(lanes)
                left_check = check_lane_predict(left_lane_tmp, left_lane_pts_tmp, right_lane_tmp, right_lane_pts_tmp)
                update_debug.publish('left_check: %d' % left_check)
                print 'left_check: ', left_check
                if left_check == 1:
                    left_lane = left_lane_tmp
                    left_lane_pts = left_lane_pts_tmp
                    right_lane = right_lane_tmp
                    right_lane_pts = right_lane_pts_tmp

# Update functions
def update_drive_error():
    global car_pos, car_orient, car_path, car_path_string, dist_error, dist_error_prev, ang_error, drive_pub
    if type(car_path_string) == list:
        return
    car_radius = 0.1
    car_ball = Point(car_pos[0], car_pos[1]).buffer(car_radius)
    while not car_ball.intersects(car_path_string):
        car_radius = 1.1*car_radius
        update_debug.publish("Keep growing the ball radius: %f" % car_radius)
        car_ball = affinity.scale(car_ball, 1.1, 1.1)
    intersection = car_ball.intersection(car_path_string)
    x_list, y_list = intersection.coords.xy
    inter_list = [[x_list[i], y_list[i]] for i in range(len(x_list))]
    lane_orient = []
    pt_next = []
    if len(inter_list) > 1:
        dot_max = -10
        for pt in inter_list:
            lane_orient_tmp = np.array(pt)-np.array(car_pos)
            lane_orient_tmp = lane_orient_tmp/np.linalg.norm(lane_orient_tmp)
            dot = np.dot(lane_orient_tmp, np.array(car_orient))
            if dot > dot_max:
                dot_max = dot
                lane_orient = lane_orient_tmp.tolist()
                pt_next = pt
    elif len(inter_list) == 1:
        lane_orient = np.array(inter_list[0])-np.array(car_pos)
        lane_orient = lane_orient/np.linalg.norm(lane_orient)
        pt_next = inter_list[0]
    if len(pt_next) == 2 and len(lane_orient) == 2:
        ang_error = find_ang(np.array(car_orient), lane_orient)
        dist_error_prev = dist_error
        dist_error = np.linalg.norm(np.array(pt_next) - np.array(car_pos))
    update_debug.publish("ang_error: %f | dist_error: %f" % (ang_error, dist_error))
    print "ang_error: ", ang_error, " dist_error: ", dist_error
    if driver.velocity > 17:
        dist_weight = (-1.0/(10*abs(dist_error)+1.0)+1.0)*0.5
        ang_weight = (abs(ang_error)/(np.pi))*0.5
        weight = dist_weight + ang_weight
        driver.velocity -= (driver.velocity - 17)*weight
        pid_debug.publish('PID velocity: %f | ang_weight: %f | dist_weight: %f' % (driver.velocity, ang_weight, dist_weight))
        drive_pub.publish(driver)
        print 'PID velocity: ', driver.velocity,' ang_weight: ', ang_weight,' dist_weight: ', dist_weight

def check_destination():
    global car_pos, car_orient, car_path, car_path_string, lane_width, destination, destination_reached, update_debug, driver, drive_pub
    if len(destination) == 2 and len(car_path) >= 2:
        dist = np.array(destination) - np.array(car_pos)
        dist = np.linalg.norm(dist)
        if dist <= lane_width/2.0:
            update_debug.publish('Stop, destination reached.')
            print 'Stop, destination reached.'
            driver.velocity = 0
            driver.angle = 0
            drive_pub.publish(driver)
            destination_reached = 1
            destination = []
            car_path = []
            car_path_string = []
        else:
            destination_reached = 0

def update_pid_error():
    global car_pos, car_orient, e_p, e_p_prev, e_i, e_d, left_lane, right_lane, left_lane_pts, right_lane_pts, drive_pub, imu_ang_vel, driver_vel
    global ang_error, dist_error, dist_error_prev
    pid_debug.publish('updated pid error:')
    if ang_error != 0 or dist_error != 0:
        error = abs(dist_error)
        if ang_error > 0:
            # turn left
            error = -1*abs(dist_error)

        e_p = error

        # cross track integrated:
        e_i += e_p

        # cross track rate:
        speed = driver_vel
        ang = find_ang(np.array(left_lane), np.array(car_orient))
        e_d = driver_vel*np.sin(ang)
    
    pid_debug.publish('PID errors e_p: %f | e_d: %f | e_i: %f' % (e_p, e_d, e_i))

def PID():
    global car_pos, car_orient, e_p, e_i, e_d, P, I, D, turning, drive_pub
    if turning == 0:
        #driver.velocity = 19.5
        driver.angle = P*e_p + D*e_d + I*e_i
        pid_debug.publish('current position: %s' % str(car_pos))
        pid_debug.publish('update drive param:')
        pid_debug.publish('Contribution of P*e_p: %f | D*e_d: %f | I*e_i: %f' % (P*e_p, D*e_d, I*e_i))
        print 'Current postion: ', car_pos, ' current orientation: ', car_orient
        print 'Contribution of P*e_p: ', P*e_p, ' D*e_d: ', D*e_d, ' I*e_i: ', I*e_i
        if driver.angle > 100:
            driver.angle = 100
        elif driver.angle < -100:
            driver.angle = -100
        driver.velocity = 0.02*abs(driver.angle)+18
        pid_debug.publish('PID output angle: %f | verlocity: %f' % (driver.angle, driver.velocity))
        drive_pub.publish(driver)
        print 'PID output angle: ', driver.angle, ' velocity: ', driver.velocity

def turning_complete_update():
    global turning, turning_complete, print_update
    global driver, drive_pub
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global left_lane_turn, right_lane_turn, lane_predict, update_toggle
    global imu_ang, odom_orient_prev

    turn = turning_complete
    update_toggle = [1,1,1]
    # Update
    driver.velocity = 16
    driver.angle = 0
    drive_pub.publish(driver)

    update_left_right(turn)
    update_debug.publish('left right updated: left_lane: %s | right_lane: %s' % (str(left_lane), str(right_lane)))
    update_probes()
    update_debug.publish('probes updated: left_probe: %s | right_probe: %s' % (str(left_probe), str(right_probe)))
    update_env()
    cam_correct_pose()
    find_intersection()
    driver.velocity = 18
    driver.angle = 0
    drive_pub.publish(driver)

# update the pose of the FOV
def update_FOV():
    global car_orient
    global car_pos

    global FOV
    global show_FOV
    global FOV_pos
    global FOV_pos_new
    global FOV_orient
    global FOV_orient_new

    FOV_pos_new = car_pos
    FOV_orient_new = car_orient

    show_FOV = 1
    visual_pub_FOV.publish("%d" % show_FOV)

    if print_main == 1:
        print 'old FOV_orient: ', FOV_orient
        print 'new FOV_orient: ', FOV_orient_new
    if abs(FOV_orient[0] - FOV_orient_new[0]) >= 0.00001 and abs(FOV_orient[1] - FOV_orient_new[1]) >= 0.00001:
        FOV_ang_off = find_ang(FOV_orient, FOV_orient_new)
        FOV_orient = FOV_orient_new
        print 'angle offset of FOV: ', FOV_ang_off
        FOV = affinity.rotate(FOV, FOV_ang_off, origin=(FOV_pos[0], FOV_pos[1]), use_radians=True)

    FOV_x_off = FOV_pos_new[0] - FOV_pos[0]
    FOV_y_off = FOV_pos_new[1] - FOV_pos[1]
    FOV_pos = FOV_pos_new
    print 'FOV position offset: ', FOV_x_off, FOV_y_off
    FOV = affinity.translate(FOV, xoff=FOV_x_off, yoff=FOV_y_off)

# update surrounding environment
def update_env():
    global car_pos
    global line_map
    global env_lines
    global env_pts
    global init, update_toggle
    global left_probe, right_probe
    # env_pts are the vertices of the nearest lines
    # env_lines are the nearest lines + their neighbors

    if update_toggle[0] == 1:
        env_lines_new = []
        env_pts_new = []

        pos_pt = Point(car_pos[0], car_pos[1]).buffer(0.5)
        for line in line_map:
            if left_probe.intersects(line) or right_probe.intersects(line) or pos_pt.intersects(line) or pos_pt.intersects(line):
                env_lines_new.append(line)
                x0, x1 = line.xy[0]
                y0, y1 = line.xy[1]
                env_pts_new.append((x0, y0))
                env_pts_new.append((x1, y1))

        update_debug.publish('number of env_pts: %d' % len(env_pts_new))
        if len(env_pts_new) < 4:
            update_debug.publish("issue: not enought env_pts: env_pts: %s | left_probe: %s | right_probe: %s")

        if init <= 3:
            env_lines = env_lines_new
            env_pts = env_pts_new

        else:
            for line in line_map:
                if line in env_lines_new:
                    continue
                x0, x1 = line.xy[0]
                y0, y1 = line.xy[1]
                if bool(set(((x0, y0),(x1, y1))).intersection(env_pts_new)):
                    env_lines_new.append(line)

            env_lines = env_lines_new
            env_pts = env_pts_new
            if len(env_lines) >= 6 and len(env_pts) >= 4:
                update_toggle[0] = 0

# update the pose change of the car
def update_pose_diff():
    global car
    global car_pos, front_probe
    global car_orient
    global car_x_off
    global car_y_off
    global car_ang_off, print_update

    x0,x1 = front_probe.coords.xy[0]
    y0,y1 = front_probe.coords.xy[1]
    car_pos_prev = np.array([x0,y0])
    car_orient_prev = np.array([x1,y1]) - np.array([x0,y0])

    if abs(car_orient[0] - car_orient_prev[0]) >= 0.001 and abs(car_orient[1] - car_orient_prev[1]) >= 0.00001:
        car_ang_off = find_ang(car_orient_prev, np.array(car_orient))
        # car_ang_off = np.dot(np.array(car_orient_prev), np.array(car_orient))/(np.linalg.norm(car_orient)*np.linalg.norm(car_orient_prev))   
        
        # car_ang_off = np.arccos(car_ang_off)
        # cross = check_cross(car_orient_prev, car_orient)
        # if cross < 0:
        #     car_ang_off = -1*car_ang_off

        car_orient_prev = car_orient
        
    else:
        car_ang_off = 0

    car_x_off = car_pos[0] - car_pos_prev[0]
    car_y_off = car_pos[1] - car_pos_prev[1]
    update_debug.publish('car x_off: %s' % str(car_x_off))
    update_debug.publish('car y_off: %s' % str(car_y_off))
    update_debug.publish('car ang_off: %s' % str(car_ang_off))
    car_pos_prev = car_pos

# Update the probes for the environment
def update_probes():
    global left_probe, right_probe
    global front_probe
    global turn_left_probe, turn_right_probe
    global car_pos, car_orient
    global car_x_off, car_y_off
    global car_ang_off

    update_pose_diff()

    left_probe = affinity.translate(left_probe, xoff=car_x_off, yoff=car_y_off)
    left_probe = affinity.rotate(left_probe, car_ang_off, origin=(car_pos[0], car_pos[1]), use_radians=True)
    right_probe = affinity.translate(right_probe, xoff=car_x_off, yoff=car_y_off)
    right_probe = affinity.rotate(right_probe, car_ang_off, origin=(car_pos[0], car_pos[1]), use_radians=True)
    front_probe = affinity.translate(front_probe, xoff=car_x_off, yoff=car_y_off)
    front_probe = affinity.rotate(front_probe, car_ang_off, origin=(car_pos[0], car_pos[1]), use_radians=True)
    turn_left_probe = affinity.translate(turn_left_probe, xoff=car_x_off, yoff=car_y_off)
    turn_left_probe = affinity.rotate(turn_left_probe, car_ang_off, origin=(car_pos[0], car_pos[1]), use_radians=True)
    turn_right_probe = affinity.translate(turn_right_probe, xoff=car_x_off, yoff=car_y_off)
    turn_right_probe = affinity.rotate(turn_right_probe, car_ang_off, origin=(car_pos[0], car_pos[1]), use_radians=True)

    update_debug.publish('car_pos reference: %s' % str(car_pos))
    update_debug.publish('front probe updated: %s' % str(front_probe))
    update_debug.publish('left probe reference: %s' % str(left_probe))
    update_debug.publish('right probe reference: %s' % str(right_probe))

# Correct the predicted orientation using IMU information
def check_orient_pred(orient_pred):
    global car_pos, car_orient, turning, imu_orient
    global imu_ang, imu_ang_vel, dt_pred, dt_imu

    localize_debug.publish('-------check_orient_pred-------')
    pred_ang_diff = find_ang(np.array(car_orient), np.array(orient_pred))
    #pred_ang_vel = pred_ang_diff/(dt_pred)
    imu_orient = rotate_by_rad(car_orient, imu_ang)
    #check = abs(imu_ang_vel - pred_ang_vel)
    check = abs(imu_ang_vel*dt_imu - pred_ang_diff)
    check = gauss_distrib(check, 0, 1/(2*np.pi), 5)
    orient_ret = check*np.array(orient_pred) + (1-check)*np.array(imu_orient)
    localize_debug.publish('orient_ret: %s | orient_pred: %s | imu_orient: %s | pred_ang_diff: %f| imu_ang_vel: %f| dt_imu: %f | odom_ang_diff: %f | check: %f' 
        % ( str(orient_ret), str(orient_pred), str(imu_orient), pred_ang_diff, imu_ang_vel, dt_imu, imu_ang_vel*dt_imu, check))
    return orient_ret

# Update the car's position and orientation checkbased on camera images
def cam_correct_pose():
    global START
    global init
    global car_pos, car_orient
    global left_lane, right_lane
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line
    global turning, turning_complete, print_local
    global cam_pos_shift, update_toggle
    global e_p

    if START == 1 and init > 3 and turning == 0:

        # localize_debug.publish('cam_correct_pose: left_lane %s, right_lane %s, left_line %s, right_line %s, mid_left_line %s, mid_right_line %s, mid_line: %s' 
        #                         % (str(left_lane), str(right_lane), str(left_lane), str(right_lane), str(cam_mid_left_line), str(cam_mid_right_line), str(cam_mid_line)))

        left_ang = mid_ang = right_ang = ang_off = 0
        left_dist = right_dist = 0
        counter = 0

        if type(cam_left_line) != int and np.array(left_lane).tolist() != [0,0]:
            p0 = cam_left_line[0]
            p1 = cam_left_line[1]
            p0 = undist_cam(p0)
            p1 = undist_cam(p1)
            vec = np.array(p1) - np.array(p0)
            vec = align_vec(vec, np.array([0,1]))
            print 'left_lane: ', left_lane
            print 'car_orient: ', car_orient
            left_lane = align_vec(np.array(left_lane), np.array(car_orient))
            left_ang = find_ang(vec, left_lane)
            left_dist = abs(300 - p0[0])
            left_dist = float(left_dist)*0.6/185
            localize_debug.publish('left angle: %f' % left_ang)
            localize_debug.publish('p0 undistorted: %s' % str(p0))
            localize_debug.publish('p1 undistorted: %s'% str(p1))
            localize_debug.publish('left dist: %f' % left_dist)
            
            counter += 1
        if type(cam_right_line) != int and np.array(right_lane).tolist() != [0,0]:
            p0 = cam_right_line[0]
            p1 = cam_right_line[1]
            p0 = undist_cam(p0)
            p1 = undist_cam(p1)
            vec = np.array(p0) - np.array(p1)
            vec = align_vec(vec, np.array([0,1]))
            right_lane = align_vec(np.array(right_lane), np.array(car_orient))
            right_ang = find_ang(vec, right_lane)
            right_dist = abs(300 - p1[0])
            right_dist = float(right_dist)*0.6/185
            localize_debug.publish('right angle: %f' % right_ang)
            localize_debug.publish('p0 undistorted: %s' % str(p0))
            localize_debug.publish('p1 undistorted: %s'% str(p1))
            localize_debug.publish('right dist: %f' % right_dist)
            
            counter += 1
        if counter != 0:
            # ang_off is the angle in world frame (angle from image line to real line)
            ang_off = (left_ang + right_ang)/float(counter)
            localize_debug.publish('-------------------final ang offset: %f' % np.degrees(ang_off))
            orient = rotate_by_rad(car_orient, ang_off)
            localize_debug.publish('predicted orient: %s' % str(orient))
            orient_diff = np.array(orient) - np.array(car_orient)
            orient_ang_diff = find_ang(np.array(car_orient), np.array(orient))
            localize_debug.publish('orient diff: %s | orient_ang_diff: %f' % (str(orient_diff), np.degrees(orient_ang_diff)))
            if abs(orient_ang_diff) < np.pi/4.0:
                # Update the car's orientation
                car_orient_prev = car_orient
                weight = gauss_distrib(orient_ang_diff, 0, 1/(2*np.pi), 1)
                localize_debug.publish('weight: %f' % weight)
                car_orient = np.array(car_orient) + weight*orient_diff
                car_orient = car_orient.tolist()
                localize_debug.publish('previous orientation: %s | camera updated orientation: %s' % ( str(car_orient_prev), str(car_orient)))
            else:
                # There must be some issues, update everything
                update_toggle = [1,1,1]

            # Update the car's position shift
            if counter == 2:
                if print_local == 1:
                    print 'predicted road width: ', left_dist + right_dist
                dist = (right_dist - left_dist)/2.0
                # Update error for PID control
                e_p = 0.2*e_p + 0.8*dist
                localize_debug.publish('Cross track error: %f ' % e_p)
            elif left_dist != 0:
                dist = car_width/2.0 - left_dist
            elif right_dist != 0:
                dist = right_dist - car_width/2.0
            
            # find the midpoint of the lanes
            dist_vec = dist_mid_lane()
            if print_local == 1:
                print 'vector to middle of the lane: ', dist_vec
            # Find the vector orthogonal to the car's orientation
            orient = np.array([car_orient[1],-car_orient[0]])
            orient = dist*orient/np.linalg.norm(orient)
            if print_local == 1:
                print 'orthogonal direction: ', orient
                print '==========> proposed car_pos: ', np.array(car_pos) + dist_vec + orient
            if abs(left_dist + right_dist - 0.6) <= 0.1:
                cam_pos_shift = dist_vec + orient

# initalize a guess for the orientation
def init_orient():
    global init
    global driver
    global now_time
    global prev_time
    global prev_pos
    global odom_pos
    global now_pos
    global car_pos
    global car_orient
    global left_lane, left_lane_pts
    global right_lane, right_lane_pts
    global update_pose_prev_time, update_pose_now_time

    global init_pos_list
    global init_orient_list

    if print_main == 1:
        print 'time diff inside: ', now_time - prev_time

    if prev_pos[0] == 1j and prev_pos[1] == 1j:
        init_pos_list.append(odom_pos)
        print 'number of pos collected: ', len(init_pos_list)
        if len(init_pos_list) >= 10:
            prev_pos = avg_list(init_pos_list)
            print 'init prev_pos: ', prev_pos
            print 'init timer'
            prev_time = now_time = rospy.get_time()

    elif (init == 1) and (now_time - prev_time <= 1):
        driver.velocity = 20
        driver.angle = 0
        drive_pub.publish(driver)
        print 'moving ... '

        now_time = rospy.get_time()

    elif init == 1:
        print  'stop'
        driver.velocity = 0
        driver.angle = 0
        drive_pub.publish(driver)
        init = 2
    elif init == 2:
        print 'init orientation'
        now_pos = odom_pos
        print 'now_pos: ', now_pos
        print 'prev_pos: ', prev_pos
        orient = np.array(now_pos) - np.array(prev_pos)
        if np.linalg.norm(orient) <= 0.001:
            print 'Not moving: ', now_pos, prev_pos
            init = 1
            # Reset timer
            prev_time = now_time = rospy.get_time()
            # Reset position
            prev_pos = [1j,1j]
            now_pos = [0,0]
            return
        orient = orient/np.linalg.norm(orient)
        orient = orient.tolist()
        init_orient_list.append(orient)
        print 'number of orient collected: ', len(init_orient_list)
        if len(init_orient_list) == 10:
            orient = avg_list(init_orient_list)
            car_orient = orient
            car_pos = np.array(car_orient) + np.array(prev_pos)
            car_pos = car_pos.tolist()
            print 'measured orientation: ', orient
            init = 3
            # stop the car
            driver.velocity = 0
            drive_pub.publish(driver)
            # reset timer
            prev_time = now_time = rospy.get_time()
            update_pose_prev_time = update_pose_now_time = rospy.get_time()
    elif init == 3:
        # align the orient with the sides
        # find all neighoring lines that have the smallest angle with the measured orientation
        print 'first updating the environment'
        global env_lines

        update_debug.publish('First updating the probes')
        update_probes()
        update_env()

        car_orient_tmp = car_orient

        dot_max = -10
        for line in env_lines:
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            vec = np.array([x1, y1]) - np.array([x0, y0])
            vec = vec/(np.linalg.norm(vec))
            print 'line vec: ', vec
            dot = np.dot(vec, np.array(car_orient))
            print 'dot: ', dot
            dot = dot/(float(np.linalg.norm(car_orient)))
            print 'cos_ang: ', dot

            if abs(dot) > dot_max:
                print 'Better guess: ', abs(dot)
                dot_max = abs(dot)
                if dot < 0:
                    print 'opposite'
                    vec = -1*vec
                    print vec
                car_orient_tmp = vec.tolist()
                print 'update orientation: ', car_orient_tmp

        car_orient = car_orient_tmp

        print '------------------final aligned orientation: ', car_orient
        car_pos = np.array(car_pos) + offset*np.array(car_orient)
        car_pos = car_pos.tolist()
        print 'final shifted car_pos: ', car_pos
        get_left_right()
        find_intersection()
        global lane_predict
        print 'init lane_predict: ', lane_predict

        init += 1
        # update_probes()
        # update_env()

        init += 1

        # reset timer
        prev_time = now_time = rospy.get_time()

# Transform a vector from base frame to world frame (defined by orient)
# Only consider rotation
def base_to_world(vec, orient):
    # rotation:
    orient_ang = find_ang(np.array([1,0]), np.array(orient))
    vec_ret = rotate_by_rad(vec, orient_ang)

    return vec_ret

###### Prediction functions ######

def fuse_speed():
    global imu_vel
    global driver_vel, driver_ang
    global car_orient

    scale = 0.1 # parameter to weigh the different variables
    speed = 0

    driver_vec = [driver_vel*np.cos(-1*driver_ang), driver_vel*np.sin(-1*driver_ang)]
    driver_vec = rotate_by_rad(driver_vec, ang)
    imu_vel_vec = rotate_by_rad(imu_vel, ang)
    
    vel_scale = 0.1
    vel = (1-vel_scale)*np.array(driver_vec) + vel_scale*np.array(imu_vel)
    speed = np.linalg.norm(vel)

    return speed

def fuse_angle():
    global imu_ang
    global driver_vel, driver_ang

    scale = 0.01
    ang = 0

    if driver_vel < 0:
        driver_ang = -1*driver_ang

    # We trust the driver a lot more
    ang = (1-scale)*driver_ang + scale*imu_ang

    return ang

def fuse_speed_ang(ang):
    global imu_vel, imu_ang
    global driver_vel, driver_ang
    global odom_orient_tmp

    # counter-closewise is negative, clockwise is positive
    # so invert the sign
    # driver_vec = [driver_vel*np.cos(-1*driver_ang), driver_vel*np.sin(-1*driver_ang)]
    # driver_vec = rotate_by_rad(driver_vec, ang)
    # imu_vel_vec = rotate_by_rad(imu_vel, ang)
    
    # vel_scale = 0.1
    # vel = (1-vel_scale)*np.array(driver_vec) + vel_scale*np.array(imu_vel)
    # speed = np.linalg.norm(vel)
    
    imu_speed = np.linalg.norm(np.array(imu_vel))
    vel_scale = gauss_distrib(imu_speed, driver_vel, 1/(2*np.pi), 5)
    #speed = (1-vel_scale)*(imu_speed+driver_vel)/2.0 + vel_scale*imu_speed
    speed = (1-vel_scale)*(imu_speed+driver_vel)/2.0 + vel_scale*driver_vel

    ang_scale = 0.1
    ang_ret = (1-ang_scale)*imu_ang + ang_scale*driver_ang

    # if np.dot(vel, odom_orient_tmp) < 0:
    #     speed = -1*speed
    #     ang_ret = -1*ang_ret
    
    if driver_vel < 0:
        speed = -1*speed
        ang_ret = -1*ang_ret

    # Try to correct imu_speed
    #imu_vel = (1.0*speed/imu_speed)*imu_vel

    localize_debug.publish("--------fuse speed ang------")
    localize_debug.publish('driver_vel: %f | driver_ang: %f' % (driver_vel, driver_ang))
    # localize_debug.publish("driver vec: %s | imu_vel: %s | vel: %s | speed %f" % (driver_vec, imu_vel, vel, speed))
    localize_debug.publish("driver vel: %f | imu speed: %f | vel_scale: %f | speed %f" % (driver_vel, imu_speed, vel_scale, speed))
    localize_debug.publish("driver ang: %s | imu_ang: %s | ang_ret: %f" % (driver_ang, imu_ang, ang_ret))

    return speed, ang_ret

def gauss_distrib(a, m, b, c):
    return (1/(np.sqrt(2*np.pi*b)))*np.exp(-0.5*c*(a-m)**2/float(b))

def predict_orient(orient_tmp):
    global turning, offset, car_length
    global localize_debug, dt_pred

    car_ang = find_ang([1,0], orient_tmp)
    speed, ang = fuse_speed_ang(car_ang)
    orient = np.array(orient_tmp)/np.linalg.norm(orient_tmp)
    orient = orient.astype(float)
    localize_debug.publish("predicting orientation:")
    localize_debug.publish("speed: %f | ang: %f | orient: %s" % (speed, ang, str(orient)))
    if (turning != 0) or (abs(ang) > 0.001):
        orient = orient.reshape([2,1])
        r = car_length/(2*np.tan(ang))
        r = abs(r)
        if r > 0.001:
            phi = 0
            if turning == 0:
                # PID update
                # dt_tmp = 0.001
                dt_tmp = dt_pred
                if ang <0:
                    # Turn left
                    phi = -1*speed*dt_tmp/r
                    localize_debug.publish("anti-clockwise: %f" % phi)
                else:
                    # Turn right
                    phi = speed*dt_tmp/r
                    localize_debug.publish("clockwise: %f" % phi)
            if turning == 2:
                phi = speed*dt_pred/r
                localize_debug.publish("anti-clockwise: %f" % phi)
            elif turning == 3:
                phi = -1*speed*dt_pred/r
                localize_debug.publish("clockwise: %f" % phi)
            R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
            orient = np.matmul(R, orient)
            orient = orient.reshape([2])
            orient = orient.tolist()
        else:
            orient = orient.tolist()
    else:
        orient = orient.tolist()
    localize_debug.publish("predict orient: %s" % str(orient))
    return orient

def predict_pos(pos_tmp, orient_tmp):
    global turning, offset, car_length
    global localize_debug, dt_pos

    car_ang = find_ang([1,0], orient_tmp)
    speed, ang = fuse_speed_ang(car_ang)
    x = pos_tmp[0]
    y = pos_tmp[1]
    p = np.array([x,y], dtype=np.float64)
    speed = abs(speed)
    orient = np.array(orient_tmp)/np.linalg.norm(orient_tmp)
    orient = orient.astype(float)
    localize_debug.publish("predicting orientation:")
    localize_debug.publish("speed: %f | ang: %f | orient: %s" % (speed, ang, str(orient)))
    if (turning != 0) or (abs(ang) > 0.001):
        p = p.reshape([2,1])
        orient = orient.reshape([2,1])
        r = car_length/(2*np.tan(ang))
        r = abs(r)
        if r > 0.001:
            O_x = x+r*np.sin(car_ang)
            O_y = y-r*np.cos(car_ang)
            O = [O_x, O_y]
            localize_debug.publish('O: %s' % str(O))
            O = np.array(O)
            O = O.reshape([2,1])

            p = p - O
            phi = 0
            if turning == 0:
                # PID update
                # dt_tmp = 0.001
                dt_tmp = dt_pos
                if ang <0:
                    # Turn left
                    phi = -1*speed*dt_tmp/r
                    localize_debug.publish("anti-clockwise: %f" % phi)
                else:
                    # Turn right
                    phi = speed*dt_tmp/r
                    localize_debug.publish("clockwise: %f" % phi)
            if turning == 2:
                phi = speed*dt_pos/r
                localize_debug.publish("anti-clockwise: %f" % phi)
            elif turning == 3:
                phi = -1*speed*dt_pos/r
                localize_debug.publish("clockwise: %f" % phi)
            R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
            p = np.matmul(R, p)
            p = p + O
            p = p.reshape([2])
            p = p.tolist()
        else:
            p += float(speed)*orient*dt_pos
            p = p.tolist()
            orient = orient.tolist()
    else:
        p += float(speed)*orient*dt_pos
        p = p.tolist()
        orient = orient.tolist()
    localize_debug.publish("predict pos: %s" % str(p))
    return p

# Predict the distribution of pose (position and orientation)
# dt later from driver and IMU data
def predict_pose_distrib(pos_tmp, orient_tmp):
    # Use driver and IMU data to find position reletive to the center line
    # Predict the motion time dt later
    # Make a normal distribution in the predicted odom frame
    # Transform the predicted odom frame to the world frame
    # Given a new coordinate from GPS, find the reliability of that coord

    global turning, pred_distrib, offset, car_length
    global dt_pred
    localize_debug.publish("turning: %d" % turning)

    # speed = fuse_speed()
    # ang = fuse_angle()
    car_ang = find_ang([1,0], orient_tmp)
    speed, ang = fuse_speed_ang(car_ang)

    x = pos_tmp[0]
    y = pos_tmp[1]
    p = np.array([x,y], dtype=np.float64)
    speed = abs(speed)

    
    orient = np.array(orient_tmp)/np.linalg.norm(orient_tmp)
    orient = orient.astype(float)

    # localize_debug.publish("check turning: %d | angle: %f" % (turning, np.degrees(ang)))

    if (turning != 0) and (abs(ang) > 0.001):
        p = p.reshape([2,1])
        orient = orient.reshape([2,1])

        r = car_length/(2*np.tan(ang))
        r = abs(r)

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
        # cross = check_cross(np.array(orient_tmp), ang_vec)
        # if cross >= 0:

        phi = 0

        if turning == 2:
            localize_debug.publish("Anti-clockwise")
            if print_local == 1:
                print 'Anti-clockwise'
            phi = speed*dt_pred/r
        elif turning == 3:
            localize_debug.publish("clockwise")
            if print_local == 1:
                print 'Clockwise'
            phi = -1*speed*dt_pred/r

        localize_debug.publish("Rotation phi: %f" % np.degrees(phi))

        R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
        p = np.matmul(R, p)
        p = p + O
        p = p.reshape([2])
        p = p.tolist()

        orient = np.matmul(R, orient)
        orient = orient.reshape([2])
        
    else:
        p += float(speed)*orient*dt_pred
        p = p.tolist()
        orient = orient.tolist()

    localize_debug.publish('updated position tmp: %s' % str(p))

    dist = np.linalg.norm(np.array(p)-np.array(pos_tmp))
    
    # x distribution is the horizontal distribution of angles
    x0 = 0.3
    x1 = 0.3*(1-x0)
    x2 = (1-x0)*(1+abs(speed))
    x3 = 5*(1-x0)
    var_x = x0*dist + x1*abs(speed) + x2*abs(ang) + x3*dt_pred 

    # y distribution is the verticle distribution of distances
    y0 = 0.3
    y1 = 1
    y2 = 0.7
    y3 = 5
    var_y = y0*dist + y1*abs(speed) + y2*abs(ang) + y3*dt_pred
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

    return ell, p, orient

# Update pose during turning
def update_turning(dt):
    global car_pos
    global car_orient
    global driver
    global turning

    speed = map_velocity(driver.velocity)
    ang = map_angle(driver.angle)

    x = car_pos[0]
    y = car_pos[1]
    p = np.array([x,y])
    p = p.reshape([2,1])
    speed = abs(speed)

    car_ang = find_ang([1,0], car_orient)
    if print_main == 1:
        print 'car angle wrt +x-axis: ', np.degrees(car_ang)

    orient = np.array(car_orient).reshape([2,1])

    if (turning != 0) and (ang > 0.001):
        r = car_length/(2*np.tan(ang))
        r = abs(r)
        if print_main == 1:
            print 'dist to O: ', r

        O_x = x+r*np.sin(car_ang)
        O_y = y-r*np.cos(car_ang)
        O = [O_x, O_y]
        if print_main == 1:
            print 'O: ', O

        O = np.array(O)
        O = O.reshape([2,1])

        p = p - O
        
        if print_main == 1:
            print 'speed: ', speed

        ang_vec = np.array([1, np.tan(ang)])
        cross = check_cross(np.array(car_orient), ang_vec)
        if cross >= 0:
            if print_main == 1:
                print 'Anti-clockwise'
            phi = speed*dt/r
        else:
            if print_main == 1:
                print 'Clockwise'
            phi = -1*speed*dt/r
        
        if print_main == 1:
            print 'rotation: ', np.degrees(phi)

        R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
        p = np.matmul(R, p)
        p = p + O
        p = p.reshape([2])
        p = p.tolist()

        if print_main == 1:
            print 'Updated position: ', car_pos
        car_pos = p
        
        orient = np.matmul(R, orient)
        orient = orient.reshape([2])
        car_orient = orient.tolist()

def dist_to_path():
    global car_pos, car_path


def navigate():
    global now_time, prev_time
    global left_probe, right_probe, front_probe, turn_left_probe, turn_right_probe
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line
    global env_lines
    global driver, drive_pub
    global turning, turning_complete, left_lane_turn, right_lane_turn
    global imu_ang, odom_orient_prev
    global lane_predict, off_lane, off_lane_toggle
    global odom_pos, odom_pos_prev, odom_pos_tmp, odom_orient_tmp
    global odom_pos_1, odom_pos_prev_1, odom_pos_tmp_1, odom_orient_tmp_1, odom_pos_2, odom_pos_prev_2, odom_pos_tmp_2, odom_orient_tmp_2
    global destination, destination_reached, reset_dest
    global car_pos, car_orient, prev_pos
    global map_graph, map_edges, line_map, car_path, car_path_string

    if print_main == 1:
        print '------------------------- START NAVIGATE --------------------------'
        print 'car_pos: ', car_pos
        print 'car_orient: ', car_orient
    
    print 'reset_dest: ', reset_dest
    if reset_dest == 1 and len(destination) == 2:
        print 'destination changed'
        reset_dest = 0
        update_debug.publish('Stop, recalculate the path')
        print 'Stop, recalculate the path'
        driver.velocity = 0
        driver.angle = 0
        drive_pub.publish(driver)
        if type(car_orient) == np.ndarray:
            car_orient = car_orient.tolist()
        try:
            path = parse_map.find_path(car_pos, destination, car_orient, map_graph, map_edges, line_map)
        except:
            print 'Cannot find path.'
            destination = []
            return
        num_pt = len(path)
        print 'num_pt: ', num_pt
        if num_pt > 0:
            pt_list = [car_pos]
            for i in range(num_pt):
                pt = map_graph.nodes[path[i]]['pos']
                pt_list.append(pt)
            pt_list.append(destination)
            #print 'pt_list before: ', pt_list
            pt_list = parse_map.parse_spline(pt_list, car_orient)
            #print 'pt_list after: ', pt_list
            spline_pts = parse_map.draw_spline(pt_list)
            #print 'spline_pts: ', spline_pts
            car_path = [[spline_pts[0][i], spline_pts[1][i]] for i in range(len(spline_pts[0]))]
        else:
            car_path = [car_pos, destination]
        path_x_list = [car_path[i][0] for i in range(len(car_path))]
        path_y_list = [car_path[i][1] for i in range(len(car_path))]
        #print 'path_x_list: ', path_x_list, ' path_y_list: ', path_y_list
        car_path = [tuple(pt) for pt in car_path]
        #print car_path
        update_debug.publish("car_path updated: %s" % str(car_path))
        car_path_string = LineString(car_path)
        path_list = [[path_x_list[i], path_y_list[i]] for i in range(len(path_x_list))]
        path_pub.publish('%s' % str(path_list))
    global lane_width, car_width, path_orient
    if len(destination) == 2 and len(car_path) >= 2:
        if destination_reached == 0:
            print 'Following lane'
            driver.velocity = 19.5
            update_pid_error()
            PID()

# Callback functions

# Initializing and restarting
def joy_callback(joy_data):
    global START
    global toggle
    global init, turning, turning_complete, left_lane_turn, right_lane_turn, turning_start, destination, destination_reached, reset_dest
    global car, car_pos, car_orient, car_orient_prev, imu_orient, ell_list, car_pos_pred, path_orient, car_path_string
    global prev_time, now_time, update_pose_prev_time, update_pose_now_time
    global prev_pos, now_pos
    global init_orient_list, init_pos_list
    global env_lines, env_pts
    global left_lane, right_lane, left_lane_pts, right_lane_pts, lane_predict, off_lane, off_lane_toggle, update_toggle
    global car_x_off, car_y_off, car_ang_off, pred_distrib
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line, cam_pos_shift
    global odom_pos, odom_pos_prev, odom_pos_tmp, odom_orient_tmp, e_p, e_p_prev, e_d, e_i, dist_error, dist_error_prev, ang_error
    global odom_pos_1, odom_pos_prev_1, odom_pos_tmp_1, odom_orient_tmp_1, odom_pos_2, odom_pos_prev_2, odom_pos_tmp_2, odom_orient_tmp_2
    global imu_lin_accel, imu_ang_vel, imu_vel, imu_now_time, imu_prev_time, imu_time_lapse, imu_vel, imu_ang, odom_orient_prev

    # RB
    if (joy_data.buttons[5] == 1):
        toggle = (toggle+1)%2

        if toggle == 1:
            START = 1
            init = 1
            print 'turn on'
        elif toggle == 0:
            START = 0
            init = 1
            turning = 0
            turning_complete = 0
            turning_start = 0
            destination = []
            destination_reached = 0
            car_path = []
            car_path_string = []
            reset_dest = 1
            path_orient = []

            # Reset the car back to initial position
            car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
            car_pos = [0,0]
            car_orient = [1,0]
            car_orient_prev = [1,0]
            car_pos_pred = []
            imu_orient = [1,0]
            odom_pos = [0,0]
            odom_pos_prev = [0,0]
            odom_pos_tmp = [0,0] # odom_pos is the odometry data, NOT the exact position
            odom_orient_tmp = [1,0]
            odom_pos_1 = [0,0]
            odom_pos_prev_1 = [0,0]
            odom_pos_tmp_1 = [0,0] # odom_pos is the odometry data, NOT the exact position
            odom_orient_tmp_1 = [1,0]
            odom_pos_2 = [0,0]
            odom_pos_prev_2 = [0,0]
            odom_pos_tmp_2 = [0,0] # odom_pos is the odometry data, NOT the exact position
            odom_orient_tmp_2 = [1,0]
            odom_orient_prev = 1j
            ell_list = []
            
            car_x_off = car_y_off = car_ang_off = 0
            pred_distrib = [[0,0],0,0,0]

            # Reset timer
            prev_time = now_time = rospy.get_time()
            update_pose_prev_time = 0
            update_pose_now_time = 0
            
            # Reset position
            prev_pos = [1j,1j]
            now_pos = car_pos
            init_orient_list = []
            init_pos_list = []
            cam_pos_shift = [0,0]

            # Reset IMU
            imu_lin_accel = [0,0]
            imu_ang_vel = 0
            imu_vel = 0
            imu_now_time = 0
            imu_prev_time = 0
            imu_time_lapse = 0
            imu_vel = [0,0]
            imu_ang = 0

            # Reset PID control errors
            e_p = 0 # Cross track error
            e_p_prev = 0
            e_d = 0 # Cross track error rate
            e_i = 0 # Cross track error integral
            dist_error = 0
            ang_error = 0
            dist_error_prev = 0

            # Reset environment
            env_lines = []
            env_pts = tuple([])
            left_lane = 0
            right_lane = 0
            left_lane_pts = 0
            right_lane_pts = 0
            cam_left_line = 0
            cam_right_line = 0
            cam_mid_line = 0
            cam_mid_left_line = 0
            cam_mid_right_line = 0
            left_lane_turn = right_lane_turn = 0
            lane_predict = [[],[]]
            off_lane = 0
            off_lane_toggle = 0
            update_toggle = [1,1,1]

            print 'turn off'

def callback_imu(data):
    # Under construction
    global imu_lin_accel, imu_ang_vel, driver_vel
    global imu_now_time, imu_prev_time, imu_time_lapse
    global imu_vel, imu_ang
    global imu_orient
    global car_pos, car_orient
    global START, init
    # IMU angle is in the base frame
    # IMU velocity is in the base frame

    imu_now_time = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*1e-9
    if START == 1 and init > 3:
        x = data.linear_acceleration.x
        y = data.linear_acceleration.y
        if abs(x) < 0.15:
            x = 0
        if abs(y) > 0.8 or abs(y) < 0.3:
            # Ignore all y that are too large or too small
            y = 0
        imu_lin_accel = [x,y]
        imu_ang_vel = -1*data.angular_velocity.z

        if imu_prev_time != 0:
            dt = imu_now_time - imu_prev_time
            imu_time_lapse += abs(dt)
            imu_vel = np.array(imu_vel) + np.array(imu_lin_accel)*dt
            imu_vel = imu_vel.tolist()
            imu_ang = imu_ang_vel*dt
            localize_debug.publish('imu_lin_accel %s ; imu_ang_vel: %f' % (str(imu_lin_accel), imu_ang_vel))
            localize_debug.publish('imu_vel integrated: %s ; imu_speed: %f ; driver velocity: %f | imu_ang integrated: %f' 
                                % (str(imu_vel), np.linalg.norm(imu_vel), driver.velocity, imu_ang))
            localize_debug.publish('imu_ang_vel: %f | imu_ang: %f | dt_imu: %f' % (imu_ang_vel, imu_ang, dt_imu))

    imu_prev_time = imu_now_time

def callback_param(data):
    global driver_vel, driver_ang
    global turning, turning_complete, turning_start
    global START, init

    driver_vel = data.velocity
    driver_ang = data.angle
    driver_vel = map_velocity(driver_vel)
    driver_ang = map_angle(driver_ang)
    driver_ang = -1*driver_ang #counter-clockwiseis positive

# Extract the camera data to update the orientation
def callback_camera(string):
    global START
    global init
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line
    global turning, turning_complete

    if START == 1 and init > 3:

        data = eval(string.data)
        cam_left_line = data[0]
        cam_mid_left_line = data[1]
        cam_mid_line = data[2]
        cam_mid_right_line = data[3]
        cam_right_line = data[4]

# Obtaining the car's location
def callback_tag_1(data):
    global START, odom_pos_1
    global car_pos

    odom = eval(data.data)[0]
    if START == 1:
        odom_pos_1 = [odom[0],odom[1]]

def callback_tag_2(data):
    global START, odom_pos_2
    global car_pos
    odom = eval(data.data)[0]
    if START == 1:
        odom_pos_2 = [odom[0],odom[1]]

def callback_map(data):
    global destination, reset_dest
    if len(destination) > 0:
        reset_dest = 1
    destination = eval(data.data)
    print 'reset destination: ', destination

def callback_web(data):
    global destination, reset_dest
    dest = eval(data.data)
    print 'received web data: ', dest;
    if type(dest) == list:
        if len(destination) > 0:
            reset_dest = 1
        destination = dest
        print 'reset destination: ', destination

rospy.init_node('lane_driver', anonymous=True)

START = 0
toggle = 0
init = 1
turning = 0
turning_complete = 0
turning_start = 0

# Navigation parameters
destination = []
destination_reached = 0
car_path = []
car_path_string = []
reset_dest = 1
path_orient = []

# Initialize environment variables
env_lines = []
env_pts = tuple([])
left_lane = 0
right_lane = 0
left_lane_pts = 0
right_lane_pts = 0
cam_left_line = 0
cam_right_line = 0
cam_mid_line = 0
cam_mid_left_line = 0
cam_mid_right_line = 0
lane_predict = [[],[]]
off_lane = 0
off_lane_toggle = 0
update_toggle = [1,1,1]
map_poly = []

num_cycles = 20
rate = rospy.Rate(num_cycles)
dt_pred = 0.08 # Set this manually for now
dt_pos = 0.22
dt_imu = 0.22

# Start new threads
threads = []
update_thread = Update()
localize_thread = Localize()

threads.append(update_thread)
threads.append(localize_thread)

if __name__ == '__main__':

    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber('line_cnt/slope', String, callback_camera)
    rospy.Subscriber('imu_transformed_1', Imu, callback_imu)
    rospy.Subscriber("drive_param_fin", drive_param, callback_param)
    rospy.Subscriber("DecaWave_tag_1", String, callback_tag_1)
    rospy.Subscriber("DecaWave_tag_2", String, callback_tag_2)
    rospy.Subscriber("map_interface", String, callback_map)
    rospy.Subscriber('web_socket/path', String, callback_web)    

    # Initialize the map
    map_path = '/home/antonio/catkin_ws/src/race/src/map/map_3.txt'
    line_map, line_map_plot, map_poly = map_gen.map_gen(map_path, np.array(p0), np.array(p1), map_p0_idx=7, map_p1_idx=16)

    target = [2.714665, 1.557787]
    map_pts = parse_map.get_map_pts(map_poly)
    map_graph, map_edges = parse_map.get_map_graph(map_pts, line_map)
    print map_graph.edges(data=True)
    # Initialize timer
    prev_time = now_time = rospy.get_time()
    update_pose_prev_time = update_pose_now_time = 0

    while not rospy.is_shutdown():
        car_pos, car_orient, left_lane_pts, right_lane_pts, car_pos_pred, odom_pos = check_type([car_pos, car_orient, left_lane_pts, right_lane_pts, car_pos_pred, odom_pos])
        # if type(car_orient) == np.ndarray:
        #     car_orient = car_orient.tolist()
        visual_pub_car.publish("car_pos: %s; car_orient: %s; left_right: %s; lane_predict: %s; car_pos_pred: %s; odom_pos: %s"
            % (str(car_pos), str(car_orient), str([left_lane_pts, right_lane_pts]), str(lane_predict), str(car_pos_pred), str(odom_pos)))
        # visual_pub_car.publish("[%s, %s, %s, %s, %s, %s, %s]" 
        # % (str(car_pos), str(car_orient), str(pred_distrib), str([left_lane_pts, right_lane_pts]), str(lane_predict), str(car_pos_pred), str(odom_pos)))
        localize_debug.publish('car_pos: %s | car_orient: %s' % (str(car_pos), str(car_orient)))
        if START == 1:
            if init <= 3:
                init_orient()
            else:
                navigate()
                if print_main == 1:
                    print '------------------------- END NAVIGATE --------------------------'
                # Update timer
                now_time = rospy.get_time()
        rate.sleep()
















































