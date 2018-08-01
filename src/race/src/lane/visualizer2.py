#!/usr/bin/env python

# Game Plan:
# Phase 1:
# Init the car, update the car's location in real time
# Phase 2:
# Init the FOV, and update the FOV and plot the guessed orientation
# Phase 3:
# Init the pred_car, which only shows up when the predict function is called

import rospy
import numpy as np

from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
from marvelmind_nav.msg import hedge_pos
from marvelmind_nav.msg import beacon_pos_a
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import shapely
from shapely.geometry import Point, LineString, Polygon, MultiLineString
from shapely import affinity
import map_gen
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.animation as anim


# Parameters of the car, unit in meters
# Car represented as a shapely polygon
car_length = 0.34
car_width = 0.3
offset = 0.13 # meters
car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
car_pos = [0,0] # Center of the car, NOT the beacon
car_x_off = 0.0
car_y_off = 0.0
car_ang_off = 0.0
car_orient = car_orient_new = [1,0] # Inital direction is along the x-axis
# Initialize car
car_pos = car_pos_new = [0,0]

# Initialize FOV
FOV = Polygon([(0,0),(0.75,0.59),(0.75,-0.59)])
show_FOV = 0
FOV_pos = FOV_pos_new = [0,0]
FOV_orient = FOV_orient_new = car_orient

# Initialize probes
side_probe_len = 0.7
front_probe_len = 1
shift = 0.2
probes = Polygon([(0.0, 0.0), (0.0, side_probe_len), (0.0, 0.0), (0.0, -1*side_probe_len),
                 (0.0, 0.0), (front_probe_len+shift, 0.0), (0.0, 0.0), (0.1+shift, 0.0101), (0.2+shift, 0.0417),
                 (0.3+shift, 0.1), (0.4+shift, 0.2), (0.5+shift, 0.5), (0.4+shift, 0.2), (0.3+shift, 0.1), (0.2+shift, 0.0417),
                 (0.1+shift, 0.0101), (0.0, 0.0), (0.1+shift, -0.0101), (0.2+shift, -0.0417), (0.3+shift, -0.1), 
                 (0.4+shift, -0.2), (0.5+shift, -0.5), (0.4+shift, -0.2), (0.3+shift, -0.1), (0.2+shift, -0.0417), (0.1+shift, -0.0101), (0.0, 0.0)])

# left_probe = LineString([(0.0, 0.0), (0.0, side_probe_len)])
# right_probe = LineString([(0.0, 0.0), (0.0, -1*side_probe_len)])
left_probe = LineString([(0.0, 0.0), (side_probe_len/2.0, np.sqrt(3)/2*side_probe_len), (0, side_probe_len), (-0.5*side_probe_len, np.sqrt(3)/2*side_probe_len), (0.0, 0.0)])
right_probe = LineString([(0.0, 0.0), (side_probe_len/2.0, -1*np.sqrt(3)/2*side_probe_len), (0, -1*side_probe_len), (-0.5*side_probe_len, -1*np.sqrt(3)/2*side_probe_len), (0.0, 0.0)])
front_probe = LineString([(0.0, 0.0), (front_probe_len+shift, 0.0)])
turn_left_probe = LineString([(0.0, 0.0), (0.1+shift, 0.0101), (0.2+shift, 0.0417), (0.3+shift, 0.1), (0.4+shift, 0.2), (0.5+shift, 0.5)])
turn_right_probe = LineString([(0.0, 0.0), (0.1+shift, -0.0101), (0.2+shift, -0.0417), (0.3+shift, -0.1), (0.4+shift, -0.2), (0.5+shift, -0.5)])

# Initialize the ellpitical distribution
ell_distrib = [[0,0],0,0,0]

# Initialize left-right lanes
left_right = []
lane_predict = []

# Map parameters that determine the size and orientation of the map
p0 = [3.75, 4.66]
p1 = [1.26, 2.88]

# Helper functions
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

# Find the plot limits
def get_lim(line_map_plot):
    x_min = 1000
    x_max = -1000
    y_min = 1000
    y_max = -1000
    for line in line_map_plot:
        x0 = line[0][0]
        x1 = line[1][0]
        x_min_tmp = min(x0, x1)
        x_max_tmp = max(x0, x1)
        # print 'x_min_tmp: ', x_min_tmp
        # print 'x_max_tmp: ', x_max_tmp
        if x_min_tmp < x_min:
            x_min = x_min_tmp
        if x_max_tmp > x_max:
            x_max = x_max_tmp
        y0 = line[0][1]
        y1 = line[1][1]
        y_min_tmp = min(y0, y1)
        y_max_tmp = max(y0, y1)
        # print 'y_min_tmp: ', y_min_tmp
        # print 'y_max_tmp: ', y_max_tmp
        if y_min_tmp < y_min:
            y_min = y_min_tmp
        if y_max_tmp > y_max:
            y_max = y_max_tmp
        x_lim = [x_min-1, x_max+1]
        y_lim = [y_min-1, y_max+1]
    return x_lim, y_lim

# Plot the map
def plot_line(lines):
    global ax
    global line_map_plot

    line_pts = []
    for line in lines:
        pts = []
        for p in line.boundary:
            pts.append((p.x,p.y))
        line_pts.append(pts)
    lc = mc.LineCollection(line_pts, linewidths=1)
    ax.add_collection(lc)
    x_lim, y_lim = get_lim(line_map_plot)
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)

# Callback functions
def callback_car(string):
    global car_pos_new
    global car_orient_new
    global ell_distrib
    global lane_predict

    data = eval(string.data)
    car_pos_new = data[0]
    car_orient_new = data[1]
    try:
        ell_distrib = data[2]
    except:
        pass
    try:
        left_right = data[3]
    except:
        pass
    try:
        lane_predict = data[4]
    except:
        pass

def update_pose_diff():
    global car, car_pos, car_pos_new, car_orient, car_orient_new

    rotate = 0

    if abs(car_orient[0] - car_orient_new[0]) >= 0.00001 and abs(car_orient[1] - car_orient_new[1]) >= 0.00001:
        rotate = 1
        car_ang_off = np.dot(np.array(car_orient), np.array(car_orient_new))/(np.linalg.norm(car_orient)*np.linalg.norm(car_orient_new))   
        # print 'dot: ', car_ang_off
        car_ang_off = np.arccos(car_ang_off)
        cross = np.cross(np.array(car_orient), np.array(car_orient_new))
        if cross < 0:
            car_ang_off = -1*car_ang_off
        car_orient = car_orient_new
        # print 'angle offset: ', np.degrees(car_ang_off)

    center = (car_pos[0], car_pos[1])
    car_x_off = car_pos_new[0] - car_pos[0]
    car_y_off = car_pos_new[1] - car_pos[1]
    car_pos = car_pos_new

    return rotate, car_ang_off, center, car_x_off, car_y_off

def update_car(rotate, car_ang_off, center, car_x_off, car_y_off):
    global car

    if rotate:
        car = affinity.rotate(car, car_ang_off, origin='center', use_radians=True)

    car = affinity.translate(car, xoff=car_x_off, yoff=car_y_off)

# def update_probes()

# def update_FOV()

# Update animation
def update_anim(num):
    global car
    global car_pos
    global car_pos_new
    global car_orient
    global car_orient_new

    global FOV
    global FOV_pos
    global FOV_pos_new
    global FOV_orient
    global FOV_orient_new

    global probes
    global probe_plot

    global left_probe, right_probe
    global front_probe
    global turn_left_probe, turn_right_probe

    global ell_distrib

    print 'input car_pos_new: ', car_pos_new
    print 'input car_orient_new: ', car_orient_new

    center = (car_pos[0], car_pos[1])
    car_ang_off = 0

    if abs(car_orient[0] - car_orient_new[0]) >= 0.00001 and abs(car_orient[1] - car_orient_new[1]) >= 0.00001:
        
        car_ang_off = np.dot(np.array(car_orient), np.array(car_orient_new))/(np.linalg.norm(car_orient)*np.linalg.norm(car_orient_new))   
        print 'dot: ', car_ang_off
        car_ang_off = np.arccos(car_ang_off)
        cross = np.cross(np.array(car_orient), np.array(car_orient_new))
        if cross < 0:
            car_ang_off = -1*car_ang_off
        car_orient = car_orient_new
        print 'angle offset: ', np.degrees(car_ang_off)
        car = affinity.rotate(car, car_ang_off, origin='center', use_radians=True)

    
    car_x_off = car_pos_new[0] - car_pos[0]
    car_y_off = car_pos_new[1] - car_pos[1]
    car_pos = car_pos_new
    # print 'car position offset: ', car_x_off, car_y_off
    car = affinity.translate(car, xoff=car_x_off, yoff=car_y_off)

    x,y = car.exterior.xy
    car_plot.set_data(x,y)

    # Update probes
    if car_ang_off != 0:
        probes = affinity.rotate(probes, car_ang_off, origin=center, use_radians=True)
    probes = affinity.translate(probes, xoff=car_x_off, yoff=car_y_off)
    x,y = probes.exterior.xy
    probe_plot.set_data(x,y)

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

    x,y = left_probe.coords.xy
    left_probe_plot.set_data(x,y)
    x,y = right_probe.coords.xy
    right_probe_plot.set_data(x,y)
    x,y = front_probe.coords.xy
    front_probe_plot.set_data(x,y)
    x,y = turn_left_probe.coords.xy
    turn_left_probe_plot.set_data(x,y)
    x,y = turn_right_probe.coords.xy
    turn_right_probe_plot.set_data(x,y)

    # Update ell
    p = ell_distrib[0]
    var_x = ell_distrib[1]
    var_y = ell_distrib[2]
    ang_off = ell_distrib[3]
    ell = Point(p[0], p[1]).buffer(1)
    ell = affinity.scale(ell, var_x, var_y)
    ell = affinity.rotate(ell, ang_off, origin='center', use_radians=True)
    x,y = ell.exterior.xy
    ell_plot.set_data(x,y)

    # Update left right
    if len(left_right) > 0:
        try:
            l_lane = left_right[0]
            x = [l_lane[0][0], l_lane[1][0]]
            y = [l_lane[0][1], l_lane[1][1]]
            left_plot.set_data(x,y)
            r_lane = left_right[1]
            x = [r_lane[0][0], r_lane[1][0]]
            y = [r_lane[0][1], r_lane[1][1]]
            right_plot.set_data(x,y)
            print 'left right okay'
        except:
            pass

    # Update prediction
    if len(lane_predict) > 0:
        try:
            ll_lane = lane_predict[0][0]
            x = [ll_lane[0][0], ll_lane[1][0]]
            y = [ll_lane[0][1], ll_lane[1][1]]
            ll_plot.set_data(x,y)
            lr_lane = lane_predict[0][1]
            x = [lr_lane[0][0], lr_lane[1][0]]
            y = [lr_lane[0][1], lr_lane[1][1]]
            lr_plot.set_data(x,y)
            rl_lane = lane_predict[1][0]
            x = [rl_lane[0][0], rl_lane[1][0]]
            y = [rl_lane[0][1], rl_lane[1][1]]
            rl_plot.set_data(x,y)
            rr_lane = lane_predict[1][1]
            x = [rr_lane[0][0], rr_lane[1][0]]
            y = [rr_lane[0][1], rr_lane[1][1]]
            rr_plot.set_data(x,y)
            print 'predict okay'
        except:
            pass

    # Update FOV
    if show_FOV:
        FOV_pos_new = car_pos
        FOV_orient_new = car_orient
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
        
        x,y = FOV.exterior.xy
        FOV_plot.set_data(x,y)

    return car_plot, FOV_plot, probe_plot, left_probe_plot, right_probe_plot, front_probe_plot, turn_left_probe_plot, turn_right_probe_plot, ell_plot, left_plot, right_plot, ll_plot, lr_plot, rl_plot, rr_plot

rospy.init_node('visualizer')

if __name__ == '__main__':

    rospy.Subscriber('/lane_driver/car', String, callback_car)

    # Initialize the plot
    fig = plt.figure()
    ax = plt.axes(xlim=(-3,5), ylim=(-1,8))
    car_plot, = ax.plot([], [], color='#6699cc', fillstyle='full',
        linewidth=3, solid_capstyle='round')
    FOV_plot, = ax.plot([], [], color='b', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    probe_plot, = ax.plot([], [], color='r', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    left_probe_plot, = ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    right_probe_plot, = ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    front_probe_plot, = ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    turn_left_probe_plot, = ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    turn_right_probe_plot, = ax.plot([], [], color='g', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    ell_plot, = ax.plot([], [], color='r', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    left_plot, = ax.plot([], [], color='r', alpha=0.3, fillstyle='full',
        linewidth=5, solid_capstyle='round')
    right_plot, = ax.plot([], [], color='r', alpha=0.3, fillstyle='full',
        linewidth=5, solid_capstyle='round')
    ll_plot, = ax.plot([], [], color='m', alpha=0.3, fillstyle='full',
        linewidth=5, solid_capstyle='round')
    lr_plot, = ax.plot([], [], color='m', alpha=0.3, fillstyle='full',
        linewidth=5, solid_capstyle='round')
    rl_plot, = ax.plot([], [], color='y', alpha=0.3, fillstyle='full',
        linewidth=5, solid_capstyle='round')
    rr_plot, = ax.plot([], [], color='y', alpha=0.3, fillstyle='full',
        linewidth=5, solid_capstyle='round')

    # Initialize the map
    map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
    line_map, line_map_plot = map_gen.map_gen(map_path, np.array(p0), np.array(p1))
    plot_line(line_map)

    animation = anim.FuncAnimation(fig, update_anim, frames=30, blit=True)

    plt.show()
