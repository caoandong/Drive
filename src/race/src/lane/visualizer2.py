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
from shapely.geometry import Point, LineString, Polygon
from shapely import affinity
import map_gen
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.animation as anim


# Parameters of the car, unit in meters
# Car represented as a shapely polygon
car_length = 0.5
car_width = 0.3
offset = 0.13 # meters
car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                       (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
car_pos = [0,0] # Center of the car, NOT the beacon
car_orient = car_orient_new = [1,0] # Inital direction is along the x-axis
# Initialize car
car_pos = car_pos_new = [0,0]

# Initialize FOV
FOV = Polygon([(0,0),(0.75,0.59),(0.75,-0.59)])
show_FOV = 0
FOV_pos = FOV_pos_new = [0,0]
FOV_orient = FOV_orient_new = car_orient

# Map parameters that determine the size and orientation of the map
p0 = [4.180, 4.329]
p1 = [1.780, 2.558]


# Helper functions

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

    data = eval(string.data)
    car_pos_new = data[0]
    car_orient_new = data[1]
    

def callback_FOV(string):
    global show_FOV

    show_FOV = eval(string.data)

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

    print 'input car_pos_new: ', car_pos_new
    print 'input car_orient_new: ', car_orient_new

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

    # Update FOV
    if show_FOV:
        FOV_pos_new = car_pos
        FOV_orient_new = car_orient
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
        
        x,y = FOV.exterior.xy
        FOV_plot.set_data(x,y)

    return car_plot, FOV_plot



rospy.init_node('visualizer')

if __name__ == '__main__':

    rospy.Subscriber('/lane_driver/car', String, callback_car)
    rospy.Subscriber('/lane_driver/FOV', String, callback_FOV)

    # Initialize the plot
    fig = plt.figure()
    ax = plt.axes(xlim=(-3,5), ylim=(-1,8))
    car_plot, = ax.plot([], [], color='#6699cc', fillstyle='full',
        linewidth=3, solid_capstyle='round')
    FOV_plot, = ax.plot([], [], color='b', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')

    # Initialize the map
    map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
    line_map, line_map_plot = map_gen.map_gen(map_path, np.array(p0), np.array(p1))
    plot_line(line_map)

    animation = anim.FuncAnimation(fig, update_anim, frames=30, blit=True)

    plt.show()
