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
car_pos = [0,0] # Center of the car, NOT the beacon
car_orient = [1,0] # Inital direction is along the x-axis
odom_pos = car_pos # odom_pos is the odometry data, NOT the exact position

# Initialize FOV
FOV = Polygon([(0,0),(0.75,0.59),(0.75,-0.59)])
show_FOV = 0
FOV_pos = FOV_pos_new = [0,0]
FOV_orient = FOV_orient_new = car_orient

# Drive parameters
driver = drive_param()
driver.velocity = 0
driver.angle = 0

# Map parameters that determine the size and orientation of the map
p0 = [4.180, 4.329]
p1 = [1.780, 2.558]

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
def undist_cam(p):
    M = np.array([[-0.24825365335095023, -1.3835463057733923, 360.9934788630403],
                  [-0.09005486236297656, -3.1198655572818654, 712.6915509275204], 
                  [-0.00013159536138092456, -0.0047202178254111115, 1.0]])
    p.append(1)
    p = np.array(p).reshape([-1,1])
    print 'p vector: ', p
    p = np.matmul(M, p).reshape([-1]).tolist()
    p = [p[0]/p[2], -1*p[1]/p[2]]
    print 'undistorted: ', p

    return p

# Find the left and the right lane to the car
def get_left_right():
    global car_orient
    global car_pos

    left_probe = [-car_orient[1], car_orient[0]]
    left_probe = (np.array(car_pos) + np.array(left_probe)).tolist()
    left_probe = LineString([(car_pos[0], car_pos[1]), (left_probe[0], left_probe[1])])

    right_probe = [car_orient[1], -car_orient[0]]
    right_probe = (np.array(car_pos) + np.array(right_probe)).tolist()
    right_probe = LineString([(car_pos[0], car_pos[1]), (right_probe[0], right_probe[1])])

    for line in env_lines

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
            cross = np.cross(slope_vec, line_vec)
            if cross >= 0:
                # rotate slope_vec counter-clockwise
                ang_ret = abs(ang_min)
                print 'ang_ret: ', ang_ret
            else:
                # rotate slope_vec clockwise
                ang_ret = -1*abs(ang_min)
                print 'ang_ret: ', ang_ret

    return ang_ret

# Update functions

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

    print 'old FOV_orient: ', FOV_orient
    print 'new FOV_orient: ', FOV_orient_new
    if abs(FOV_orient[0] - FOV_orient_new[0]) >= 0.00001 and abs(FOV_orient[1] - FOV_orient_new[1]) >= 0.00001:
        FOV_ang_off = np.dot(np.array(FOV_orient), np.array(FOV_orient_new))/(np.linalg.norm(FOV_orient)*np.linalg.norm(FOV_orient_new))
        FOV_ang_off = np.arccos(FOV_ang_off)
        cross = np.cross(np.array(FOV_orient), np.array(FOV_orient_new))
        if cross < 0:
            FOV_ang_off = -1*FOV_ang_off
        FOV_orient = FOV_orient_new
        print 'angle offset of FOV: ', FOV_ang_off
        FOV = affinity.rotate(FOV, FOV_ang_off, origin=(FOV_pos[0], FOV_pos[1]), use_radians=True)

    FOV_x_off = FOV_pos_new[0] - FOV_pos[0]
    FOV_y_off = FOV_pos_new[1] - FOV_pos[1]
    FOV_pos = FOV_pos_new
    print 'FOV position offset: ', FOV_x_off, FOV_y_off
    FOV = affinity.translate(FOV, xoff=FOV_x_off, yoff=FOV_y_off)


env_lines = []
env_pts = tuple([])

# update surrounding environment
def update_env():
    global car_pos
    global line_map
    global env_lines
    global env_pts
    # env_pts are the vertices of the nearest lines
    # env_lines are the nearest lines + their neighbors

    env_lines_new = []
    env_pts_new = []

    pos_pt = Point(car_pos[0], car_pos[1]).buffer(1.0)
    for line in line_map:
        if pos_pt.intersects(line):
            env_lines_new.append(line)
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            env_pts_new.append((x0, y0))
            env_pts_new.append((x1, y1))
    
    # print 'surrounding line: ', env_lines_new
    # print 'surrounding points: ', env_pts_new

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

    global init_pos_list
    global init_orient_list

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
    elif init == 3:
        # align the orient with the sides
        # find all neighoring lines that have the smallest angle with the measured orientation
        print 'first updating the environment'
        global env_lines
        update_env()
        dot_max = -10
        for line in env_lines:
            x0, x1 = line.xy[0]
            y0, y1 = line.xy[1]
            vec = np.array([x1, y1]) - np.array([x0, y0])
            vec = vec/(np.linalg.norm(vec))
            print 'line vec: ', vec
            print 'car_orient: ', car_orient
            dot = np.dot(vec, np.array(car_orient))
            print 'dot: ', dot
            dot = dot/(float(np.linalg.norm(car_orient)))
            print 'cos_ang: ', dot

            if abs(dot) > dot_max:
                print 'Better guess: ', dot
                dot_max = dot
                if dot < 0:
                    print 'opposite'
                    vec = -1*vec
                    print vec
                car_orient = vec.tolist()

        print 'final aligned orientation: ', car_orient
        car_pos = np.array(car_pos) + offset*np.array(car_orient)
        car_pos = car_pos.tolist()
        print 'final shifted car_pos: ', car_pos
        init += 1

# update the orientation based on image
# def find_orient()


def navigate():
    global now_time
    global prev_time

    # update_FOV()


# Callback functions

# Initializing and restarting
def joy_callback(joy_data):
    global START
    global toggle
    global init
    global car
    global car_pos
    global car_orient
    global prev_time
    global now_time
    global prev_pos
    global now_pos
    global init_orient_list
    global init_pos_list
    
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

            # Reset the car back to initial position
            car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
            car_pos = [0,0]
            car_orient = [1,0]
            visual_pub_car.publish("[%s, %s]" % (str(car_pos), str(car_orient)))

            # Reset timer
            prev_time = now_time = rospy.get_time()
            # Reset position
            prev_pos = [1j,1j]
            now_pos = car_pos
            init_orient_list = []
            init_pos_list = []

            print 'turn off'

# Extract the camera data to update the orientation
def callback_camera(string):
    global car_orient
    global env_lines

    left_lane, right_lane = get_left_right()

    data = eval(string.data)
    left_line = data[0]
    mid_line = data[1]
    right_line = data[2]
    left_ang = mid_ang = right_ang = ang_off = 0
    counter = 0

    if left_line != 0:
        p0 = left_line[0]
        p1 = left_line[1]
        p0 = undist_cam(p0)
        p1 = undist_cam(p1)
        vec = np.array(p1) - np.array(p0)
        left_ang = find_slope_ang(vec)
        counter += 1
    if mid_line != 0:
        p0 = mid_line[0]
        p1 = mid_line[1]
        p0 = undist_cam(p0)
        p1 = undist_cam(p1)
        vec = np.array(p1) - np.array(p0)
        mid_ang = find_slope_ang(vec)
        counter += 1
    if right_line != 0:
        p0 = right_line[0]
        p1 = right_line[1]
        p0 = undist_cam(p0)
        p1 = undist_cam(p1)
        vec = np.array(p0) - np.array(p1)
        right_ang = find_slope_ang(vec)
        counter += 1
    if counter != 0:
        ang_off = (left_ang + mid_ang + right_ang)/counter
        print 'final ang offset: ', ang_off



# Updating the car's pose
def callback(odom):
    global START
    global init
    global turning

    global odom_pos
    global car_pos
    global car_orient

    if START == 1:
        odom_pos = [odom.pose.pose.position.x,odom.pose.pose.position.y]
        # Initializing stage:
        if init <= 3:
            car_pos = odom_pos
        # Updating stage
        elif turning == 0:
            odom_pos = np.array(odom_pos) + offset*np.array(car_orient)
            vec = odom_pos - np.array(car_pos)
            dist = np.linalg.norm(vec)
            if dist > 0.00001:
                vec = vec/float(dist)
                if dist <= 1:
                    print 'vec: ', vec
                    ang = np.dot(vec, np.array(car_orient))/np.linalg.norm(car_orient)
                    print 'dot: ', ang
                    ang = np.arccos(ang)
                    print 'ang: ', np.degrees(ang.tolist())
                    if (abs(ang) <= max_angle) or (abs(ang) >= (np.pi - max_angle)):
                        car_pos = odom_pos.tolist()
                        print 'car_pos updated: ', car_pos
                        visual_pub_car.publish("[%s, %s]" % (str(car_pos), str(car_orient)))
        # Turning stage
        else:
            print 'Turning'

rospy.init_node('lane_driver', anonymous=True)

START = 0
toggle = 0
init = 1
turning = 0

# Initialize position
prev_pos = [1j,1j]
now_pos = car_pos
init_orient_list = []
init_pos_list = []

num_cycles = 20
rate = rospy.Rate(num_cycles)

if __name__ == '__main__':

    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber('odometry/filtered', Odometry, callback)
    rospy.Subscriber('line_cnt/slope', String, callback_camera)

    # Initialize the map
    map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
    line_map, line_map_plot = map_gen.map_gen(map_path, np.array(p0), np.array(p1))

    # Initialize timer
    prev_time = now_time = rospy.get_time()

    while not rospy.is_shutdown():
        visual_pub_car.publish("[%s, %s]" % (str(car_pos), str(car_orient)))
        print 'car_orient: ', car_orient
        if START == 1:
            if init <= 3:
                init_orient()
            else:
                if now_time - prev_time > 5:
                    driver.velocity = 0
                    drive_pub.publish(driver)
                    print 'slow down, updating the environment'
                    update_env()
                    # reset timer
                    prev_time = now_time = rospy.get_time()

                navigate()

            # Update timer
            now_time = rospy.get_time()
            # print 'time diff outside: ', now_time - prev_time
        rate.sleep()