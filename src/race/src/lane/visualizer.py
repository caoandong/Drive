#!/usr/bin/env python

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

# pub = rospy.Publisher('visualize_hedge', Marker, queue_size=1)
# pts = Marker()
# lines = Marker()
# vecs = Marker()

def init_shapes():
    global pts
    global lines
    global vecs

    pts.header.frame_id = lines.header.frame_id = vecs.header.frame_id = 'odom'
    pts.ns = lines.ns = vecs.ns = 'pts_lines_vecs'
    pts.id = 0
    lines.id = 1
    vecs.id = 2

    vecs.action = 0 # ADD one vector. That's it.

    pts.type = Marker.POINTS
    lines.type = Marker.LINE_STRIP
    vecs.type = Marker.ARROW

    pts.scale.x = 0.1
    pts.scale.y = 0.1
    lines.scale.x = 0.05
    vecs.scale.x = 0.07 # length
    vecs.scale.y = 0.09 # width
    vecs.scale.z = 0.5
    
    pts.color.b = 1.0
    pts.color.a = 1.0
    lines.color.b = 1.0
    lines.color.a = 1.0
    vecs.color.r = 1.0
    vecs.color.a = 1.0

def quat_to_R(q):
    # 1st col
    R00 = q.x**2+q.y**2-q.z**2-q.w**2
    R10 = 2*(q.y*q.z+q.x*q.w)
    R20 = 2*(q.y*q.w-q.x*q.z)

    # 2nd col
    R01 = 2*(q.y*q.z-q.x*q.w)
    R11 = q.x**2-q.y**2+q.z**2-q.w**2
    R21 = 2*(q.z*q.w+q.x*q.y)

    # 3rd col
    R02 = 2*(q.y*q.w+q.x*q.z)
    R12 = 2*(q.z*q.w-q.x*q.y)
    R22 = q.x**2-q.y**2-q.z**2+q.w**2

    R = np.array([[R00, R01, R02],[R10, R11, R12],[R20, R21, R22]])

    return R

def callback_hedge(odom):
    global pts
    global lines
    global vecs

    pts.header.stamp = lines.header.stamp = vecs.header.stamp = rospy.get_rostime()
    
    pts.action = lines.action = 0 # ADD new ones

    # Create vertices for the pts and lines
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    # print('heard it: ', x, y, z)

    p = Point()
    p.x = x
    p.y = y
    p.z = z

    vecs.pose.position = odom.pose.pose.position
    vecs.pose.orientation = odom.pose.pose.orientation

    # Orientation vector
    # p_array = np.array([x,y,z])
    # q = odom.pose.pose.orientation
    # R = quat_to_R(q)
    # p_orient = np.matmul(R, p_array)
    # p_orient = p_array + p_orient + p_array/np.linalg.norm(p_array)
    # p2 = Point()
    # p2.x = p_orient[0]
    # p2.y = p_orient[1]
    # p2.z = p_orient[2]

    if len(pts.points) > 10:
        del pts.points[0]
    pts.points.append(p)
    if len(lines.points) > 10:
        del lines.points[0]
        del lines.points[0]
    lines.points.append(p)
    # lines.points.append(p2)
    vecs.points.append(p)
    if len(vecs.points) > 2:
        del vecs.points[0]
    # vecs.points = [p, p2]


def callback_car(string):
    global car_pos_new
    global car_ang_new

    data = eval(string.data)
    car_pos_new = np.array(data[0])
    car_ang_new = data[1]

def callback_FOV(string):
    global show_FOV
    global FOV_pos
    global FOV_ang

    data = eval(string.data)
    show_FOV = data[0]
    FOV_pos = np.array([data[1], data[2]])
    FOV_ang = float(data[3])

def callback_pred(string):  
    global show_pred_car
    global pred_car_pos_new
    global pred_car_ang_new

    data = eval(string.data)
    show_pred_car = data[0]
    pred_car_pos_new = np.array(data[1])
    pred_car_ang_new = float(data[2])

def plot_line(lines):
    global ax
    # global axes
    line_pts = []
    for line in lines:
        pts = []
        for p in line.boundary:
            pts.append((p.x,p.y))
        line_pts.append(pts)
    # print 'line_pts: ', line_pts
    lc = mc.LineCollection(line_pts, linewidths=1)
    ax.add_collection(lc)

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

def update_anim(num):
    global car
    global car_plot
    global car_pos
    global car_pos_new
    global car_ang
    global car_ang_new

    global show_pred_car
    global pred_car
    global pred_car_plot
    global pred_car_pos
    global pred_car_pos_new
    global pred_car_ang
    global pred_car_ang_new
    
    global show_FOV
    global FOV
    global FOV_plot
    global FOV_pos
    global FOV_ang

    car_x_off = car_pos_new[0] - car_pos[0]
    car_y_off = car_pos_new[1] - car_pos[1]
    car_ang_off = car_ang_new - car_ang
    car_pos = car_pos_new
    car_ang = car_ang_new

    car = affinity.rotate(car, car_ang_off, origin='center', use_radians=True)
    car = affinity.translate(car, xoff=car_x_off, yoff=car_y_off)
    x,y = car.exterior.xy
    car_plot.set_data(x,y)

    if show_pred_car:

        pred_car_x_off = pred_car_pos_new[0] - pred_car_pos[0]
        pred_car_y_off = pred_car_pos_new[1] - pred_car_pos[1]
        pred_car_ang_off = pred_car_ang_new - pred_car_ang
        pred_car_pos = pred_car_pos_new
        pred_car_ang = pred_car_ang_new

        pred_car = affinity.rotate(pred_car, pred_car_ang_off, origin='center', use_radians=True)
        pred_car = affinity.translate(pred_car, xoff=pred_car_x_off, yoff=pred_car_y_off)
        x,y = pred_car.exterior.xy
        pred_car_plot.set_data(x,y)

    else:
        pred_car.set_alpha = 0.0

    if show_FOV:

        FOV = affinity.translate(FOV, xoff=FOV_pos[0], yoff=FOV_pos[1])
        FOV = affinity.rotate(FOV, FOV_ang, origin=(FOV_pos[0], FOV_pos[1]), use_radians=True)
        x,y = FOV.exterior.xy
        FOV_plot.set_data(x,y)

    if show_FOV == 0:
        FOV_plot.set_alpha = 0.0

    return car_plot,pred_car_plot,FOV_plot

car_length = 0.5
car_width = 0.3
car = Polygon([(-car_length/2, car_width/2),(car_length/2, car_width/2),
                   (car_length/2, -car_width/2),(-car_length/2, -car_width/2)])
car_pos = car_pos_new = np.array([0,0])
car_x_off = car_y_off = 0
car_ang = car_ang_new = 0.0
car_ang_off = 0.0
orient = np.array([0,0])
show_FOV = 0
FOV = Polygon([(0,0),(0.75,0.6),(0.75,-0.6)])
FOV_pos = np.array([0,0])
FOV_ang = 0.0
show_pred_car = 0
pred_car = car
pred_car_pos = pred_car_pos_new = np.array([0,0])
pred_car_x_off = pred_car_y_off = 0
pred_car_ang = pred_car_ang_new = 0,0
pred_car_ang_off = 0.0
# p0 = p1 = np.array([0,0])
p0 = np.array([2.29, 1.20])
p1 = np.array([3.98, 3.75])


if __name__ == '__main__':
    rospy.init_node('visualize_rviz')

    # init_shapes()

    # rospy.Subscriber('/hedge_odom', Odometry, callback_hedge)
    rospy.Subscriber('/lane_driver/car', String, callback_car)
    rospy.Subscriber('/lane_driver/FOV', String, callback_FOV)
    rospy.Subscriber('/lane_driver/pred_car', String, callback_pred)

    map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'
    line_map, line_map_plot = map_gen.map_gen(map_path, p0, p1)


    fig = plt.figure()
    ax = plt.axes(xlim=(-3,5), ylim=(-1,8))
    car_plot, = ax.plot([], [], color='#6699cc', fillstyle='full',
        linewidth=3, solid_capstyle='round')
    pred_car_plot, = ax.plot([], [], color='#666666', alpha=0.7,
        linewidth=3, solid_capstyle='round')
    FOV_plot, = ax.plot([], [], color='b', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    plot_line(line_map)

    animation = anim.FuncAnimation(fig, update_anim, frames=20, blit=True)

    plt.show()

    # rate = rospy.Rate(20)

    # while not rospy.is_shutdown():
        # print 'orientation: ', vecs.pose.orientation
        # print 'heard pts: ', pts.points[0]
        # pub.publish(pts)
        # if len(lines.points) >= 2:
        #   pub.publish(lines)
        # if len(vecs.points) == 2:
        #   p1 = np.array([vecs.points[1].x,vecs.points[1].y,vecs.points[1].z])
        #   p0 = np.array([vecs.points[0].x,vecs.points[0].y,vecs.points[0].z])
        #   p1 = p1+10*(p1-p0)/np.linalg.norm(p1-p0)
        #   vecs.points[1].x = p1[0]
        #   vecs.points[1].y = p1[1]
        #   vecs.points[1].z = p1[2]
        #   pub.publish(vecs)

        # rate.sleep()

    # rospy.spin()