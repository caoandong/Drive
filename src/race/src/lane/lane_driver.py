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
    car = affinity.rotate(car, angle)
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

    print 'ang: ', ang

    r = car_length/2*np.tan(ang)
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

    if r != 0:
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
    else:
        car_ang_f = car_ang
        d_ang = 0

    x_off = p[0]-x
    y_off = p[1]-y
    print 'offset: ', x_off, y_off
    print 'offset amount: ', np.sqrt(x_off**2 + y_off**2)

    print 'dist_travel: ', speed*dt
    print '-----------------START PREDICTING-----------------------'

    return d_ang, x_off, y_off, p, car_ang_f
    

def col_free(pos, orient, param, line_map):
    global car
    global car_pos
    global car_ang

    print '----------------start checking collision----------------'

    car_prev = car

    pos = pos + offset*orient

    # for every 0.2 second, check if there is collision
    dt = 0
    x = pos[0]
    y = pos[1]
    speed = param[0]

    for t in range(5):
        dt = 1/5.0
        
        speed = map_velocity(param[0])
        ang = map_angle(param[1])
        
        d_ang, x_off, y_off, p, car_ang_f = predict_car(speed, ang, dt)
        print 'predicted result: ', x_off, y_off

        car = affinity.rotate(car, d_ang, origin='centroid', use_radians=True)
        car = affinity.translate(car, xoff=x_off, yoff=y_off)

        car_pos = np.array([p[0], p[1]])
        car_ang = car_ang_f

        # plot_poly(car)

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

def update_car(orient):
    global car
    global car_pos
    global car_ang
    global pos

    # print '------------update car----------------'

    orient = orient/float(np.linalg.norm(orient))
    center = np.array(pos) + offset*orient
    x_off = center[0] - car_pos[0]
    y_off = center[1] - car_pos[1]
    car_pos = center

    # print 'new car_pos: ', car_pos

    new_ang = np.arctan2(float(orient[1]), float(orient[0]))
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

    # update the car first
    update_car(orient)
    
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
    global START
    global init
    global line_map
    global orient
    global prev_pos
    global pos
    global START_WALKING

    if START == 1:
        # position of the beacon
        # NOT the center of the car
        pos = [odom.pose.pose.position.x,odom.pose.pose.position.y]

        if init == 1:
            if prev_pos[0] == 1j or prev_pos[1] == 1j :
                prev_pos = np.array([pos[0],pos[1]])
                print '////////////////record position: ', prev_pos
                print 'INIT CAR 1'
                init_car_1()
                
        # elif init == 2:
        #     print 'INIT CAR 2'
        #     pos = [odom.pose.pose.position.x,odom.pose.pose.position.y]
        #     init_car_2()
            
        else:
            # print 'Start driving'
            # print 'current position: ', pos
            q1 = odom.pose.pose.orientation.x
            q2 = odom.pose.pose.orientation.y
            q3 = odom.pose.pose.orientation.z
            q0 = odom.pose.pose.orientation.w
            q_ang = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))

            orient = [orient[0]*np.cos(q_ang)-orient[1]*np.sin(q_ang), 
                      orient[0]*np.sin(q_ang)+orient[1]*np.cos(q_ang)]
            orient = np.array(orient)
            # print 'current orientation: ', orient

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
            prev_pos = np.array([1j,1j])
            print 'turn on'
            prev_time = now_time = rospy.get_time()
        elif toggle == 0:
            START = 0
            START_WALKING = 0
            init = 1
            prev_pos = np.array([1j,1j])
            print 'turn off'

def update_anim(num):
    global car
    global poly

    x,y = car.exterior.xy
    poly.set_data(x,y)
    return poly,

rospy.init_node('lane_driver', anonymous=True)

START = 0
toggle = 0
START_WALKING = 0

map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
p0 = np.array([0.510, 4.370])
p1 = np.array([2.875, 2.622])
line_map, line_map_plot = map_gen.map_gen(map_path, p0, p1)

print 'map generated: ', line_map


fig = plt.figure()
# ax = fig.add_subplot(111)
ax = plt.axes(xlim=(-3,5), ylim=(-1,8))
# poly = ax.plot([], [], color='#6699cc', alpha=0.7,
#     linewidth=3, solid_capstyle='round', zorder=2)
poly, = ax.plot([],[],'r-')
plot_line(line_map)
# ax.set_xlim([-3,5])
# ax.set_ylim([-1,8])

# plot_line(line_map)
# ax.set_xlim([-2,4])
# ax.set_ylim([3,9])
# plt.pause(0.001)

# Question: how to determine the orientation?
orient = np.array([1,0])
car_pos = np.array([0,0])
car_ang = np.arctan(orient[1]/float(orient[0]))

init = 1
prev_time = now_time = rospy.get_time()
prev_pos = np.array([1j,1j])
pos = prev_pos

if __name__ == '__main__':

    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber('odometry/filtered', Odometry, callback)

    print 'START ANIMATION'
    animation = anim.FuncAnimation(fig, update_anim, frames=25,
                               interval=200, blit=True)
    # plt.pause(0.0001)
    # if init == 1:
    #     init_car(car_pos)
    #     init += 1

    num_cycles = 10
    rate = rospy.Rate(num_cycles)

    while not rospy.is_shutdown():

        print 'init: ', init, ' START: ', START, ' START_WALKING: ', START_WALKING
        if START == 1:
            if init > 2:
                print 'published driver: ', driver
                drive_pub.publish(driver)
                
                # plt.draw()
                plt.pause(0.01)

            if START_WALKING == 1:
                if init == 2:
                    print 'INIT CAR 2'
                    init_car_2()
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
    