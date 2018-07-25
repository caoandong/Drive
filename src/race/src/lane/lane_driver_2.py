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

drive_pub = rospy.Publisher('lane_driver', drive_param, queue_size=1)
visual_pub_car = rospy.Publisher('lane_driver/car', String, queue_size=1)
# visual_pub_FOV = rospy.Publisher('lane_driver/FOV', String, queue_size=1)

# Parameters of the car, unit in meters
# Car represented as a shapely polygon
car_length = 0.5
car_width = 0.3
offset = 0.13 # meters
max_angle = 0.431139 # in radian

car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
car_pos = car_pos_prev = [0,0] # Center of the car, NOT the beacon
car_orient = car_orient_prev = [1,0] # Inital direction is along the x-axis
odom_pos = car_pos # odom_pos is the odometry data, NOT the exact position
car_x_off = car_y_off = car_ang_off = 0.0

# Initialize FOV
FOV = Polygon([(0,0),(0.75,0.59),(0.75,-0.59)])
show_FOV = 0
FOV_pos = FOV_pos_new = [0,0]
FOV_orient = FOV_orient_new = car_orient

# Initialize probes
side_probe_len = 0.6
front_probe_len = 1
shift = 0.2
left_probe = LineString([(0.0, 0.0), (0.0, side_probe_len)])
right_probe = LineString([(0.0, 0.0), (0.0, -1*side_probe_len)])
front_probe = LineString([(0.0, 0.0), (front_probe_len+shift, 0.0)])
turn_left_probe = LineString([(0.0, 0.0), (0.1+shift, 0.0101), (0.2+shift, 0.0417), (0.3+shift, 0.1), (0.4+shift, 0.2), (0.5+shift, 0.5)])
turn_right_probe = LineString([(0.0, 0.0), (0.1+shift, -0.0101), (0.2+shift, -0.0417), (0.3+shift, -0.1), (0.4+shift, -0.2), (0.5+shift, -0.5)])

# Drive parameters
driver = drive_param()
driver.velocity = 0
driver.angle = 0

# Map parameters that determine the size and orientation of the map
p0 = [0.455, 4.408]
p1 = [2.30, 2.15]

# Printing options
print_main = 1
print_local = 0


# Threadings

class Predict (threading.Thread):
    def __init__(self):
        self.running = False
        super(Predict, self).__init__()

        # threading.Thread.__init__(self)
    def start(self):
        self.running = True
        super(Predict, self).start()
    def run(self):
        global START
        while self.running and START == 1:
            print 'Predicting ...'
    def stop(self):
        self.running = False

class Update (threading.Thread):
    def __init__(self):
        self.running = False
        super(Update, self).__init__()
        # threading.Thread.__init__(self)
    def start(self):
        self.running = True
        super(Update, self).start()
    def run(self):
        global START
        while self.running and START == 1:
            print 'Predicting ...'

    def stop(self):
        self.running = False

class Localize (threading.Thread):
    def __init__(self):
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()

    def run(self):
        global START, init
        global car_pos, car_orient, odom_pos, print_local
        global update_pose_prev_time, update_pose_now_time, cam_pos_shift

        if print_local == 1:
            print '//////////Running'

        while True:
            if START == 1 and init > 3:
                if print_local == 1:
                    print '/////////////odom_pos: ', odom_pos
                # First predict the possible pose distribution
                ell = predict_pose_distrib()
                # Modify odom_pos
                pt = Point(odom_pos[0], odom_pos[1])
                if ell.contains(pt):
                    car_pos = odom_pos
                    
                    dt = update_pose_now_time - update_pose_prev_time
                    
                    if print_local == 1:
                        print '//////////now_time: ', update_pose_now_time
                        print '//////////prev_time: ', update_pose_prev_time
                        print '//////////ellipse contains the odom pos: ', car_pos
                        print '//////////dt: ', dt
                    update_pose_prev_time = update_pose_now_time = rospy.get_time()
                    if print_local == 1:
                        print '//////////updated time: ', update_pose_now_time
                # Then correct the pose using camera data
                cam_correct_pose()

class Navigate (threading.Thread):
    def __init__(self):
        self.running = False
        super(Navigate, self).__init__()
        # threading.Thread.__init__(self)
    def start(self):
        self.running = True
        super(Navigate, self).start()
    def run(self):
        global START
        while self.running and START == 1:
            print 'Predicting ...'
    def stop(self):
        self.running = False

class Visualize (threading.Thread):
    def __init__(self):
        self.running = False
        super(Visualize, self).__init__()
        # Initialize the plot
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-3,5), ylim=(-1,8))

    def start(self):
        self.running = True
        super(Visualize, self).start()

    def plot_line(self, lines):
        line_pts = []
        for line in lines:
            pts = []
            for p in line.boundary:
                pts.append((p.x,p.y))
            line_pts.append(pts)
        lc = mc.LineCollection(line_pts, linewidths=1)
        self.ax.add_collection(lc)
        x_lim, y_lim = get_lim(line_map_plot)
        self.ax.set_xlim(x_lim)
        self.ax.set_ylim(y_lim)
    def run(self):
        global START
        while self.running and START == 1:
            # Initialize the plot
            car_plot, = self.ax.plot([], [], color='#6699cc', fillstyle='full',
                linewidth=3, solid_capstyle='round')
            probe_plot, = self.ax.plot([], [], color='r', alpha=0.5, fillstyle='full',
                linewidth=3, solid_capstyle='round')
            left_probe_plot, = self.ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
                linewidth=3, solid_capstyle='round')
            right_probe_plot, = self.ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
                linewidth=3, solid_capstyle='round')
            front_probe_plot, = self.ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
                linewidth=3, solid_capstyle='round')
            turn_left_probe_plot, = self.ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
                linewidth=3, solid_capstyle='round')
            turn_right_probe_plot, = self.ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
                linewidth=3, solid_capstyle='round')
            
            global line_map
            self.plot_line(line_map)

            self.plt.show()

    def stop(self):
        self.running = False
# Basic helper functions:

def quat_to_ang(quat):
    # Convert quaternion to angle in radian
    q1 = quat.x
    q2 = quat.y
    q3 = quat.z
    q0 = quat.w
    ang = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))

    return ang

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
    p0 = l[0]
    p1 = l[1]
    line = LineString([(p0[0],p0[1]), (p1[0],p1[1])])
    return line

def cmp_line_list(line_list):
    global car_orient
    line_ret = 0
    dot_max = -10
    for line in line_list:
        p0 = line[0]
        p1 = line[1]
        vec = np.array(p1) - np.array(p0)
        vec = vec/(np.linalg.norm(vec))
        dot = np.dot(vec, np.array(car_orient))
        dot = dot/(float(np.linalg.norm(car_orient)))
        if abs(dot) > dot_max:
            dot_max = abs(dot)
            if dot < 0:
                line_ret = [p1, p0]
            line_ret = line
    return line_ret

def get_left_right():
    global car_orient
    global car_pos
    global line_map

    left_lines = left_line = []
    right_lines = right_line = []
    left_lines_pts = []
    right_lines_pts = []
    left_line_pts = []
    right_line_pts = []

    left_orient = [-1*car_orient[1], car_orient[0]]
    left_orient = np.array(left_orient)/np.linalg.norm(left_orient)
    right_orient = [car_orient[1], -1*car_orient[0]]
    right_orient = np.array(right_orient)/np.linalg.norm(right_orient)
    left_probe = np.array(car_pos) + left_orient
    right_probe = np.array(car_pos) + right_orient
    left_probe = left_probe.tolist()
    right_probe = right_probe.tolist()
    left_probe = list_to_line([car_pos, left_probe])
    right_probe = list_to_line([car_pos, right_probe])

    for line in line_map:
        x0, x1 = line.xy[0]
        y0, y1 = line.xy[1]
        line_tmp = LineString([(x0, y0), (x1, y1)])
        if left_probe.intersects(line_tmp):
            left_lines_pts.append([[x0, y0], [x1, y1]])
        elif right_probe.intersects(line_tmp):
            right_lines_pts.append([[x0, y0], [x1, y1]])

    left_line_pts = cmp_line_list(left_lines_pts)
    right_line_pts = cmp_line_list(right_lines_pts)


    return left_line, right_line, left_line_pts, right_line_pts

def get_left_right():
    global car_orient
    global car_pos

    global left_probe
    global right_probe

    left_lines = left_line = []
    right_lines = right_line = []
    left_lines_pts = []
    right_lines_pts = []
    left_line_pts = []
    right_line_pts = []

    for line in env_lines:
        x0, x1 = line.xy[0]
        y0, y1 = line.xy[1]
        vec = np.array([x1, y1]) - np.array([x0, y0])
        vec = vec/(np.linalg.norm(vec))

        if left_probe.intersects(line):
            left_lines.append(vec)
            left_lines_pts.append([[x0,y0],[x1,y1]])
            if print_main == 1:
                print 'found a left line: ', line

        elif right_probe.intersects(line):
            right_lines.append(vec)
            right_lines_pts.append([[x0,y0],[x1,y1]])
            if print_main == 1:
                print 'found a right line: ', line

    dot_max = -10
    num_left = len(left_lines)
    num_right = len(right_lines)

    if print_main == 1:
        print 'right: ', right_lines_pts

    if num_left > 1:
        for i in range(num_left):
            line_vec = left_lines[i]
            dot_orient = np.dot(line_vec, car_orient)/(float(np.linalg.norm(car_orient)))
            if dot_orient > dot_max:
                if print_main == 1:
                    print 'found a better left line: ', line_vec
                dot_max = dot_orient
                left_line = line_vec
                left_line_pts = left_lines_pts[i]
    elif num_left == 1:
        left_line = left_lines[0]
        left_line_pts = left_lines_pts[0]
    else:
        left_line = left_line_pts = [0,0]

    if num_right > 1:
        for i in range(num_right):
            line_vec = right_lines[i]
            dot_orient = np.dot(line_vec, car_orient)/(float(np.linalg.norm(car_orient)))
            if dot_orient > dot_max:
                if print_main == 1:
                    print 'found a better right line: ', line_vec
                dot_max = dot_orient
                right_line = line_vec
                right_line_pts = right_lines_pts[i]
    elif num_right == 1:
        right_line = right_lines[0]
        right_line_pts = right_lines_pts[0]
    else:
        if num_left != 0:
            right_line = left_line
            right_line_pts = left_line_pts
        else:
            right_line = right_line_pts = [0,0]

    if num_left == 0 and num_right != 0:
        left_line = right_line
        left_line_pts = right_line_pts

    if print_main == 1:
        print 'left_lane_pts: ', left_line_pts
        print 'right_lane_pts: ', right_line_pts

    return left_line, right_line, left_line_pts, right_line_pts

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
    dot = float(np.dot(vec, target))/(np.linalg.norm(vec)*np.linalg.norm(target))
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
        # So correct this by inverting the sign
        ang_map = lin_map(float(angle), -100, 100, -0.431139, 0.431139)
        # ang_map = -1*ang_map

        return ang_map

# Turn left
def turn_left():
    global turning, driver, drive_pub
    global prev_time, now_time
    global car_pos, car_orient, prev_pos

    print '------------------Turn left!'
    turning = 2 # 2 means left
    driver.velocity = 20
    driver.angle = -100
    drive_pub.publish(driver)
    # reset timer
    prev_time = now_time = rospy.get_time()
    # reset position
    prev_pos = car_pos

# Turn right
def turn_right():
    global turning, driver, drive_pub
    global prev_time, now_time
    global car_pos, car_orient, prev_pos

    print '--------------------Turn right!'
    turning = 3 # 3 means right
    driver.velocity = 20
    driver.angle = 100
    drive_pub.publish(driver)
    # reset timer
    prev_time = now_time = rospy.get_time()
    # reset position
    prev_pos = car_pos

def find_lane_turn(line):
    global car_pos, car_orient
    if print_main == 1:
        print 'input to find_lane_turn: ', line
    # Find the point in front of the car
    p0 = np.array(line[0])
    p1 = np.array(line[1])
    dist_0 = p0-np.array(car_pos)
    dist_1 = p1-np.array(car_pos)
    if print_main == 1:
        print 'dist_0: ', dist_0, 'dist_1: ', dist_1
    # Compare the orientations
    orient = np.array(car_orient)
    dot0 = np.dot(dist_0, orient)
    dot1 = np.dot(dist_1, orient)
    pt = p0 # 0 means turn left
    if dot1 > dot0:
        pt = p1
    cross = check_cross(orient, pt)
    turn = 0 # 0 means turn right
    if cross > 0:
        turn = 1 # 1 means turn left
    return turn

def find_intersection():
    global left_lane_pts, right_lane_pts
    global left_lane_turn, right_lane_turn
    global env_pts

    left_lane_turn = find_lane_turn(left_lane_pts)
    right_lane_turn = find_lane_turn(right_lane_pts)

    if print_main == 1:
        print 'left lane turn: ', left_lane_turn
        print 'right lane turn: ', right_lane_turn

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
    if print_main == 1:
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
    slope = left_lane
    # slope = np.array(left_lane_pts[1])-np.array(left_lane_pts[0])
    # slope = slope/np.linalg.norm(slope)
    # slope = align_vec(slope, car_orient)
    print 'slope between left and right: ', slope

    dist = dist_pt_line(car_pos, mid_pt, slope)
    return dist

# Update functions

# update the pose change of the car
def update_pose_diff():
    global car
    global car_pos
    global car_pos_prev
    global car_orient
    global car_orient_prev
    global car_x_off
    global car_y_off
    global car_ang_off

    if print_main == 1:
        print 'pose diff input car_pos_prev: ', car_pos_prev
        print 'pose diff input car_orient_prev: ', car_orient_prev
        print 'pose diff car_pos: ', car_pos
        print 'pose diff car_orient: ', car_orient

    if abs(car_orient[0] - car_orient_prev[0]) >= 0.001 and abs(car_orient[1] - car_orient_prev[1]) >= 0.00001:
        
        car_ang_off = np.dot(np.array(car_orient_prev), np.array(car_orient))/(np.linalg.norm(car_orient)*np.linalg.norm(car_orient_prev))   
        if print_main == 1:
            print 'pose diff dot: ', car_ang_off
        car_ang_off = np.arccos(car_ang_off)
        cross = check_cross(car_orient_prev, car_orient)
        if cross < 0:
            car_ang_off = -1*car_ang_off

        car_orient_prev = car_orient
        if print_main == 1:
            print 'pose diff angle offset: ', np.degrees(car_ang_off)
    else:
        car_ang_off = 0

    car_x_off = car_pos[0] - car_pos_prev[0]
    car_y_off = car_pos[1] - car_pos_prev[1]
    car_pos_prev = car_pos
    if print_main == 1:
        print 'pose diff x: ', car_x_off
        print 'pose diff y: ', car_y_off

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
    global init
    # env_pts are the vertices of the nearest lines
    # env_lines are the nearest lines + their neighbors

    env_lines_new = []
    env_pts_new = []

    pos_pt = Point(car_pos[0], car_pos[1]).buffer(0.5)
    for line in line_map:
        if pos_pt.intersects(line):
            env_lines_new.append(line)
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            env_pts_new.append((x0, y0))
            env_pts_new.append((x1, y1))
    
    # print 'surrounding line: ', env_lines_new
    # print 'surrounding points: ', env_pts_new

    if init <= 3:
        print 'basic update env_lines'
        env_lines = env_lines_new
        env_pts = env_pts_new

    else:
        print 'extensive update env_lines'
        if init > 4:
            if set(tuple(env_pts_new)) == set(tuple(env_pts)):
                print 'No need to update ', env_lines
                return

        for line in line_map:
            if line in env_lines_new:
                continue
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            if bool(set(((x0, y0),(x1, y1))).intersection(env_pts_new)):
                env_lines_new.append(line)

            # if ([x0, y0] in env_pts) or ([x1, y1] in env_pts):
            #     env_lines.append(line)

        env_lines = env_lines_new
        env_pts = env_pts_new

        # print 'updated line: ', env_lines
        # print 'updated points: ', env_pts
    print 'number of updated lines: ', len(env_lines)

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

# Update the car's position and orientation based on camera images
def cam_correct_pose():
    global START
    global init
    global car_pos, car_orient
    global left_lane, right_lane
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line
    global turning, turning_complete
    global cam_pos_shift

    if START == 1 and init > 3 and turning == 0:

        print 'left_lane: ', left_lane
        print 'right_lane: ', right_lane
        print 'left line: ', cam_left_line
        print 'right line: ', cam_right_line
        print 'mid left line: ', cam_mid_left_line
        print 'mid right line: ', cam_mid_right_line
        print 'mid line: ', cam_mid_line

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
            left_lane = align_vec(np.array(left_lane), np.array(car_orient))
            left_ang = find_ang(vec, left_lane)
            print 'left angle: ', np.degrees(left_ang)

            # Find the distance to the center
            print 'p0 undistorted: ', p0
            print 'p1 undistorted: ', p1
            left_dist = abs(300 - p0[0])
            left_dist = float(left_dist)*0.6/185
            print 'left_dist: ', left_dist

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
            print 'right_ang: ', np.degrees(right_ang)

            # Find the distance to the center
            print 'p0 undistorted: ', p0
            print 'p1 undistorted: ', p1
            right_dist = abs(300 - p1[0])
            right_dist = float(right_dist)*0.6/185
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

        print '------------------find left and right lane---------------------'

        print 'Updating probes'
        update_probes()
        left_lane, right_lane, left_lane_pts, right_lane_pts = get_left_right()
        find_intersection()

        init += 1

        update_env()

        init += 1

        # reset timer
        prev_time = now_time = rospy.get_time()


###### Prediction functions ######

def fuse_speed():
    global imu_vel
    global driver_vel

    scale = 0.1 # parameter to weigh the different variables
    speed = 0

    # We trust the driver a lot more
    speed = (1-scale)*driver_vel + scale*imu_vel
    imu_vel = speed

    return speed

def fuse_angle():
    global imu_ang
    global driver_ang

    scale = 0.01
    ang = 0

    # We trust the driver a lot more
    ang = (1-scale)*driver_ang + scale*imu_ang
    imu_ang = ang

    return ang

def gauss_distrib(a, b):
    return 1/np.sqrt(2*np.pi*b)*np.exp(-0.5*a**2/float(b))

# Predict the distribution of pose (position and orientation)
# dt later from driver and IMU data
def predict_pose_distrib():
    # Use driver and IMU data to find position reletive to the center line
    # Predict the motion time dt later
    # Make a normal distribution in the predicted odom frame
    # Transform the predicted odom frame to the world frame
    # Given a new coordinate from GPS, find the reliability of that coord

    global car_pos, car_orient
    global driver, turning, pred_distrib
    global update_pose_prev_time, update_pose_now_time, print_local

    update_pose_now_time = rospy.get_time()
    dt = update_pose_now_time - update_pose_prev_time
    dt = dt*2 # times 2 because this time measures half from
    # last car_pos update to next car_pos update  
    # dt = 0.3 # Set this manually for now
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

    if (turning != 0) and (ang > 0.001):
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

        ang_vec = np.array([1, np.tan(ang)])
        cross = check_cross(np.array(car_orient), ang_vec)
        if cross >= 0:
            if print_local == 1:
                print 'Anti-clockwise'
            phi = speed*dt/r
        else:
            if print_local == 1:
                print 'Clockwise'
            phi = -1*speed*dt/r
        
        if print_local == 1:
            print 'rotation: ', np.degrees(phi)

        R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
        p = np.matmul(R, p)
        p = p + O
        p = p.reshape([2])
        p = p.tolist()

        orient = np.matmul(R, orient)
        orient = orient.reshape([2])
        
    else:
        p += speed*orient*dt
        p = p.tolist()
        orient = orient.tolist()

    print 'predicted position: ', p
    print 'predicted orientation: ', orient

    dist = np.linalg.norm(np.array(p)-np.array(car_pos))
    
    if print_local == 1:
        print 'distance to original position: ', dist

    # x distribution is the horizontal distribution of angles
    x0 = 0.3
    x1 = 0.3*(1-x0)
    x2 = (1-x0)*(1+abs(speed))
    x3 = 5*(1-x0)
    var_x = x0*dist + x1*abs(speed) + x2*abs(ang) + x3*dt
    print 'var_x: ', var_x

    # y distribution is the verticle distribution of distances
    y0 = 0.3
    y1 = 1
    y2 = 0.7
    y3 = 5
    var_y = y0*dist + y1*abs(speed) + y2*abs(ang) + y3*dt

    print 'var_y: ', var_y

    pred_distrib = [p, var_x, var_y]

    ang_off = find_ang([0,1], orient)

    ell = Point(p[0], p[1]).buffer(1)
    ell = affinity.scale(ell, var_x, var_y)
    ell = affinity.rotate(ell, ang_off, origin='center', use_radians=True)

    return ell

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


def navigate():
    global now_time, prev_time
    global car_pos, car_orient, prev_pos
    global left_probe, right_probe, front_probe, turn_left_probe, turn_right_probe
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line
    global env_lines
    global driver, driver_pub
    global turning, turning_complete, left_lane_turn, right_lane_turn

    if print_main == 1:
        print '------------------------- START NAVIGATE --------------------------'

    if print_main == 1:
        print 'Updating probes'
    update_probes()

    if turning == 0:
        if print_main == 1:
            print 'number of env_lines: ', len(env_lines)
        for line in env_lines:
            if print_main == 1:
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
                if print_main == 1:
                    print 'There is a line, slowing down'
                driver.velocity = 15.6
                driver.angle = 0
                drive_pub.publish(driver)
                turning = 1
                break
        if turning == 1:
            turning = 0
            if print_main == 1:
                print 'Here is what the lane tells me: '
                print 'left lane: ', left_lane_turn
                print 'right lane: ', right_lane_turn
                print '0 means right and 1 means left'

            left_cnt = 0
            right_cnt = 0
            for line in env_lines:
                if turn_left_probe.intersects(line):
                    left_cnt += 1
                if turn_right_probe.intersects(line):
                    right_cnt += 1
            if print_main == 1:
                print 'left intersection count: ', left_cnt
                print 'right intersection count: ', right_cnt

            if left_cnt == 0 and type(cam_left_line) == int:
                if print_main == 1:
                    print 'left got nothing'
                turn_left()
                return
            if right_cnt == 0 and type(cam_right_line) == int:
                if print_main == 1:
                    print 'right got nothing'
                turn_right()
                return
            if left_lane_turn == 1:
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
            else:
                print 'Cannot decide, slowly moving forward.'
                turning = 0
                driver.velocity = 16
                driver.angle = 0
                drive_pub.publish(driver)
    else:
        if print_main == 1:
            print 'turning value: ', turning
            print 'Update position and orientation'
        now_time = rospy.get_time()
        dt = now_time - prev_time
        prev_time = now_time
        if print_main == 1:
            print 'dt: ', dt
        update_turning(dt)
        if print_main == 1:
            print 'Updating probes'
        update_probes()

        if turning != 0:
            if turning == 2:
                print 'turning left'
                if left_lane_turn != 1:
                    print 'Are we sure about this direction?'
                    if exist_front_corner():
                        print '==============Turn right!'
                        turn_right()
                        return
                print 'Keep turning left'
                driver.velocity = 20
                driver.angle = -100
                drive_pub.publish(driver)
            if turning == 3:
                print 'turning right'
                if right_lane_turn != 0:
                    print 'Are we sure about this direction?'
                    if exist_front_corner():
                        print '==============Turn left!'
                        turn_left()
                        return
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
            if print_main == 1:
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
            if print_main == 1:
                print 'dot of right vec: ', dot
            if abs(dot) > 0.85:
                print 'Go ahead'
                driver.velocity = 16
                driver.angle = 0
                drive_pub.publish(driver)
                turning = 0
                turning_complete = 1
                return

# Callback functions

# Initializing and restarting
def joy_callback(joy_data):
    global START
    global toggle
    global init, turning, turning_complete, left_lane_turn, right_lane_turn
    global car, car_pos, car_orient
    global prev_time, now_time, update_pose_prev_time, update_pose_now_time
    global prev_pos, now_pos
    global init_orient_list, init_pos_list
    global env_lines, env_pts
    global left_lane, right_lane, left_lane_pts, right_lane_pts
    global car_x_off, car_y_off, car_ang_off, pred_distrib
    global cam_left_line, cam_right_line, cam_mid_line, cam_mid_left_line, cam_mid_right_line, cam_pos_shift
    
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

            # Reset the car back to initial position
            car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
            car_pos = [0,0]
            car_orient = [1,0]
            visual_pub_car.publish("[%s, %s]" % (str(car_pos), str(car_orient)))
            car_x_off = car_y_off = car_ang_off = 0
            pred_distrib = 0

            # Reset timer
            prev_time = now_time = rospy.get_time()
            update_pose_prev_time = update_pose_now_time = 0
            
            # Reset position
            prev_pos = [1j,1j]
            now_pos = car_pos
            init_orient_list = []
            init_pos_list = []
            cam_pos_shift = []

            # Reset environment
            env_lines = []
            env_pts = tuple([])
            left_lane = right_lane = left_lane_pts = right_lane_pts = [0,0]
            cam_left_line = cam_right_line = cam_mid_line = cam_mid_left_line = cam_mid_right_line = 0
            left_lane_turn = right_lane_turn = 0

            print 'turn off'

def callback_imu(data):
    # Under construction
    global imu_lin_accel, imu_ang_vel
    global imu_now_time, imu_prev_time
    global imu_vel, imu_ang

    imu_now_time = data.header.stamp.secs + float(data.header.stamp.nsecs)*1e-9
    imu_lin_accel = data.linear_acceleration.x
    imu_ang_vel = data.angular_velocity.z
    if imu_prev_time != 0:
        dt = imu_now_time - imu_prev_time
        imu_vel += imu_lin_accel*dt
        imu_ang += imu_ang_vel*dt

def callback_param(data):
    global driver_vel, driver_ang
    driver_vel = data.velocity
    driver_ang = data.angle
    driver_vel = map_velocity(driver_vel)
    driver_ang = map_angle(driver_ang)


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

        if print_main == 1:
            print 'left line: ', cam_left_line
            print 'right line: ', cam_right_line
            print 'mid left line: ', cam_mid_left_line
            print 'mid right line: ', cam_mid_right_line
            print 'mid line: ', cam_mid_line


# Obtaining the car's pose
def callback(odom):
    global START, odom_pos
    global car_pos, prev_pos

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


rospy.init_node('lane_driver', anonymous=True)

START = 0
toggle = 0
init = 1
turning = 0
turning_complete = 0

# Initialize position
prev_pos = [1j,1j]
now_pos = car_pos
init_orient_list = []
init_pos_list = []
cam_pos_shift = []

# Initialize turning parameter
left_lane_turn = right_lane_turn = 0

# Initialize control parameter
driver_vel = driver_ang = 0
imu_lin_accel = imu_ang_vel = imu_vel = 0
imu_now_time = imu_prev_time = 0
imu_vel = imu_ang = 0
pred_distrib = 0

# Initialize environment variables
env_lines = []
env_pts = tuple([])
left_lane = right_lane = left_lane_pts = right_lane_pts = [0,0]
cam_left_line = cam_right_line = cam_mid_line = cam_mid_left_line = cam_mid_right_line = 0

num_cycles = 20
rate = rospy.Rate(num_cycles)

# Start new threads
threads = []
predict_thread = Predict()
update_thread = Update()
localize_thread = Localize()
navigate_thread = Navigate()

threads.append(predict_thread)
threads.append(update_thread)
threads.append(localize_thread)
threads.append(navigate_thread)

if __name__ == '__main__':

    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber('odometry/filtered', Odometry, callback)
    rospy.Subscriber('line_cnt/slope', String, callback_camera)
    rospy.Subscriber('imu_transformed', Imu, callback_imu)
    rospy.Subscriber("drive_param_fin", drive_param, callback_param)

    # Initialize the map
    map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
    line_map, line_map_plot = map_gen.map_gen(map_path, np.array(p0), np.array(p1))

    # Initialize timer
    prev_time = now_time = rospy.get_time()
    update_pose_prev_time = update_pose_now_time = 0

    # predict_thread.start()
    # update_thread.start()
    # localize_thread.start()
    # print 'Start running localization thread:'
    # localize_thread.run()
    # navigate_thread.start()

    while not rospy.is_shutdown():
        visual_pub_car.publish("[%s, %s, %s]" % (str(car_pos), str(car_orient), str(pred_distrib)))
        print 'car_orient: ', car_orient
        if START == 1:
            if init <= 3:
                init_orient()
            else:
                if turning_complete == 1:
                    turning_complete = 0
                    # Update
                    print 'Slow down, updating ...'
                    driver.velocity = 16
                    driver.angle = 0
                    drive_pub.publish(driver)
                    update_env()
                    print 'Updating probes'
                    update_probes()
                    left_lane, right_lane, left_lane_pts, right_lane_pts = get_left_right()
                    print 'Updated left and right lanes: ', left_lane, right_lane
                    find_intersection()
                    print 'Updated turns: ', left_lane_turn, right_lane_turn
                    print 'Update finished'
                    driver.velocity = 18
                    driver.angle = 0
                    drive_pub.publish(driver)

                navigate()
                if print_main == 1:
                    print '------------------------- END NAVIGATE --------------------------'
            # Update timer
            now_time = rospy.get_time()
            # print 'time diff outside: ', now_time - prev_time
        rate.sleep()
















































