#!/usr/bin/env python

import rospy

from race.msg import drive_values
from race.msg import drive_param
from race.msg import drive_angle
from race.msg import drive_speed
from std_msgs.msg import String
from std_msgs.msg import Bool
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

max_speed = 18
min_speed = 15.5
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

car = Point([0,0,0])
car_length = 0.5
car_width = 0.3
# distance from mobile beacon to the COM of the car
offset = 0.13 # meters

driver = drive_param()
driver.velocity = 0
driver.angle = 0

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


def init_car(pos):
    global car
    global init
    global prev_time
    global now_time
    global prev_pos
    global driver
    global car_pos
    global car_ang

    # TODO: find the median of the position
    if init == 1:
        print 'record position: ', pos
        prev_pos = np.array(pos)
        prev_time = now_time = rospy.get_time()

        while now_time - prev_time <= 1:
            print 'move forward'

            driver.velocity = 20
            driver.angle = 0
            drive_pub.publish(driver)

            print 'moving ...'
            now_time = rospy.get_time()
        
        driver.velocity = 0
        driver.angle = 0
        drive_pub.publish(driver)
        init += 1
    if init == 2:
        new_pos = np.array(pos)
        orient = new_pos - prev_pos
        orient = orient/np.linalg.norm(orient)
        print 'init orientation: ', orient
        car_ang = np.arctan(orient[1]/float(orient[0]))
        print 'init angle: ', car_ang

        center = new_pos + offset*orient
        car_pos = center
        angle = car_ang
        car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
        car = affinity.rotate(car, angle)
        car = affinity.translate(car, xoff=center[0], yoff=center[1])
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
    ax.set_xlim([-3,5])
    ax.set_ylim([-1,8])
    plt.pause(0.0001)

def predict_car(speed, ang, dt):
    global car
    global car_pos
    global car_ang

    x = car_pos[0]
    y = car_pos[1]
    p = np.array([x,y])
    # print 'car pos: ', p
    p = p.reshape([2,1])

    r = car_length/2*np.tan(ang)
    # print 'dist to O: ', r

    O_x = x+r*np.sin(car_ang)
    O_y = y-r*np.cos(car_ang)
    O = [O_x, O_y]
    # print 'O: ', O

    O = np.array(O)
    O = O.reshape([2,1])

    p = p - O
    phi = -speed*dt/r
    # print 'rotation: ', phi

    R = np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])
    p = np.matmul(R, p)
    p = p + O
    p = p.reshape([2])

    # print 'pos after rotation: ', p

    sin_val = (O_x-p[0])/r
    cos_val = (p[1]-O_y)/r

    car_ang_f = inv_angle(sin_val, cos_val)
    # print 'changed angle: ', car_ang_f

    d_ang = car_ang_f - car_ang
    # print 'change of angle: ', d_ang

    x_off = p[0]-x
    y_off = p[1]-y
    # print 'offset: ', x_off, y_off
    # print 'offset amount: ', np.sqrt(x_off**2 + y_off**2)

    # print 'dist_travel: ', speed*dt

    return d_ang, x_off, y_off, p, car_ang_f
    

def col_free(pos, orient, param, line_map):
    global car
    global car_pos
    global car_ang

    car_prev = car

    pos = pos + offset*orient

    # for every 0.2 second, check if there is collision
    dt = 0
    x = pos[0]
    y = pos[1]
    speed = param[0]

    for t in range(5):
        dt += t/5
        
        speed = map_velocity(param[0])
        ang = map_angle(param[1])
        
        d_ang, x_off, y_off, p, car_ang_f = predict_car(speed, ang, dt)

        car = affinity.rotate(car, d_ang, origin='centroid', use_radians=True)
        car = affinity.translate(car, xoff=x_off, yoff=y_off)

        car_pos = np.array([p[0], p[1]])
        car_ang = car_ang_f

        plot_poly(car)

        for line in line_map:
            if car.intersects(line):
                print 'intersects ', line.boundary
                # Revert to previous position
                car = car_prev
                return 0
    
    # Revert to previous position
    car = car_prev
    return 1

def update_car(pos, orient):
    global car_pos
    global car_ang

    orient = orient/float(np.linalg.norm(orient))
    center = np.array(pos) + offset*orient
    x_off = center[0] - car_pos[0]
    y_off = center[1] - car_pos[1]
    car_pos = center

    new_ang = np.arctan2(float(orient[1]), float(orient[0]))
    d_ang = new_ang - car_ang
    car_ang = new_ang

    car = affinity.rotate(car, d_ang, origin='centroid', use_radians=True)
    car = affinity.translate(car, xoff=x_off, yoff=y_off)

    plot_poly(car)

def random_walk(pos, orient, line_map):
    global driver
    global car
    global drive_param_list

    # update the car first
    update_car(pos, orient)
    
    driver.velocity = 0
    driver.angle = 0

    for param in drive_param_list:
        if col_free(pos, orient, param, line_map) == 1:
            driver.velocity = param[0]
            driver.angle = param[1]


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
    global init
    global line_map
    global orient

    # position of the beacon
    # NOT the center of the car
    pos = [odom.pose.pose.position.x,odom.pose.pose.position.y]

    if init <= 2:
        init_car(pos)
    else:
        q1 = odom.pose.pose.orientation.x
        q2 = odom.pose.pose.orientation.y
        q3 = odom.pose.pose.orientation.z
        q0 = odom.pose.pose.orientation.w
        q_ang = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))

        orient = [orient[0]*np.cos(q_ang)-orient[1]*np.sin(q_ang), 
                  orient[0]*np.sin(q_ang)+orient[1]*np.cos(q_ang)]

        

        random_walk(pos, orient, line_map)

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



rospy.init_node('lane_driver', anonymous=True)

map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
p0 = np.array([1.373, 2.211])
p1 = np.array([3.942, 3.689])
line_map, line_map_plot = map_gen.map_gen(map_path, p0, p1)

fig = plt.figure()
ax = fig.add_subplot(111)
ax2 = fig.add_subplot(111)

# Question: how to determine the orientation?
orient = np.array([1,0])
car_pos = np.array([0,0])
car_ang = np.arctan(orient[1]/float(orient[0]))

init = 1
prev_time = now_time = rospy.get_time()
prev_pos = np.array([0,0,0])

rospy.Subscriber('/hedge_odom', Odometry, callback)

if init == 1:
    init_car(car_pos, orient)
    init += 1

num_cycles = 10
rate = rospy.Rate(num_cycles)

while not rospy.is_shutdown():

    if init > 2:
        drive_pub.publish(driver)

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
plt.show()