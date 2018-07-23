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
        visual_pub_car.publish("[%s, %f]" % (str(car_pos.tolist()), car_ang))
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
    