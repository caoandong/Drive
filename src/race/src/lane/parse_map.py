#!/usr/bin/env python

# generate the map plot
# find the intersection
# add each intersection into a graph
# test finding a shortest route

import numpy as np
import shapely
from shapely.geometry import Point, LineString, Polygon
from shapely import affinity
import map_gen
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.animation as anim
import matplotlib.patches as patches
import itertools

map_path = '/home/antonio/catkin_ws/src/race/src/map/map_2.txt'
lane_width = 0.6
# Map parameters that determine the size and orientation of the map
p0 = [3.75, 4.66]
p1 = [1.26, 2.88]

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

def get_map_pts(map_poly):
    vert_dict = {}
    pts_list = []
    for poly in map_poly:
        x,y = poly.exterior.coords.xy
        for i in range(len(x)):
            try:
                check = vert_dict["%f_%f" % (float(x[i]),float(y[i]))]
                print 'pt already found'
                continue
            except:
                pass
            pt = [x[i], y[i]]
            pts_list.append(pt)
            vert_dict["%f_%f" % (float(x[i]),float(y[i]))] = 1;
    return pts_list

def get_ball_line(ball, line_map):
    line_list = []
    for line in line_map:
        if ball.intersects(line):
            line_list.append(line)
    return line_list

def check_ball_list(ball, ball_list, ball_line_dict):
    ball_list_ret = ball_list[:]
    if len(ball_list_ret) > 1:
        for i in range(len(ball_list_ret)):
            for ball_tmp in ball_list_ret[i]:
                if ball.intersects(ball_tmp):
                    x,y = ball_tmp.centroid.xy
                    line_ball_tmp = ball_line_dict["%f_%f"%(float(x[0]), float(y[0]))]
                    x,y = ball.centroid.xy
                    line_ball = ball_line_dict["%f_%f"%(float(x[0]), float(y[0]))]
                    if line_ball != line_ball_tmp:
                        ball_list_ret[i].append(ball)
                        return ball_list_ret
    ball_list_ret.append([ball])

    return ball_list_ret

def plot_ball_list(ball_list, radius):
    global ax
    for i in range(len(ball_list)):
        color = float(i)/len(ball_list)
        color = (1-0.1*color,0.5*color,0.6*color)
        for ball in ball_list[i]:
            x,y = ball.centroid.xy
            circle = plt.Circle((x[0],y[0]), radius, color=color, alpha=0.1)
            ax.add_artist(circle)

def get_line_list(ball_list, line_map):
    line_list = []
    for i in range(len(ball_list)):
        lines_tmp = []
        for ball in ball_list[i]:
            for line in line_map:
                if ball.intersects(line):
                    lines_tmp.append(line)
        print 'number of lines: ', len(lines_tmp)
        line_list.append(lines_tmp)
    return line_list

def get_node_line_vec(line_pts, node):
    max_dist = -1
    max_pt = []
    line_vec = []
    assert(len(line_pts) == 2)
    for pt in line_pts:
        dist = np.linalg.norm(np.array(pt) - np.array(node))
        if dist > max_dist:
            max_dist = dist
            max_pt = pt
    if len(max_pt) == 2:
        min_pt = []
        for pt in line_pts:
            if pt != max_pt:
                min_pt = pt
        if len(min_pt) == 2:
            line_vec = np.array(max_pt) - np.array(min_pt)
            line_vec = line_vec/np.linalg.norm(line_vec)
            line_vec = line_vec.tolist()
            return line_vec
    return line_vec

def get_node_list(ball_list, node_dict, ball_line_dict):
    global lane_width
    node_list = []
    for i in range(len(ball_list)):
        pt_list = []
        line_list = []
        for ball in ball_list[i]:
            x,y = ball.centroid.xy
            pt_list.append([x[0],y[0]])
            # try:
            line_list_tmp = ball_line_dict["%f_%f"%(float(x[0]), float(y[0]))]
            for line in line_list_tmp:
                if line not in line_list:
                    line_list.append(line)
            # except:
            #     pass
        line_pts_list = []
        line_vec_list = []
        for line in line_list:
            x,y = line.xy
            p0 = [x[0],y[0]]
            p1 = [x[1],y[1]]
            line_pts = [p0,p1]
            if line_pts not in line_pts_list:
                line_pts_list.append(line_pts)
        if len(pt_list) == 2:
            node = (np.array(pt_list[0]) + np.array(pt_list[1]))/2.0
            node = node.tolist()
            node_list.append(node)
            dist_vec = np.array(pt_list[0]) - np.array(pt_list[1])
            dist = np.linalg.norm(dist_vec)
            print dist, lane_width, lane_width*np.sqrt(2)
            for line_pts in line_pts_list:
                line_vec = get_node_line_vec(line_pts, node)
                if line_vec not in line_vec_list:
                    line_vec_list.append(line_vec)
            if len(line_vec_list) > 2:
                line_vec_list_tmp = []
                for pair in itertools.combinations(line_vec_list, 2):
                    dot = np.dot(np.array(pair[0]), np.array(pair[1]))
                    print 'dot product of ', pair, ' is ', dot
                    if dot < 0.1:
                        line_vec_list_tmp = [pair[0], pair[1]]
                        line_vec_list = line_vec_list_tmp
                        break
            if abs(dist - lane_width*np.sqrt(2)) < lane_width*0.2:
                print 'node at ', node, ' is L'
                node_dict["%f_%f"%(float(node[0]), float(node[1]))] = ["L", line_vec_list]
            elif abs(dist - lane_width) < lane_width*0.2:
                line_vec_list_tmp = line_vec_list[:]
                print len(line_vec_list)
                for line_vec in line_vec_list:
                    dot = np.dot(dist_vec, np.array(line_vec))
                    print dot
                    if abs(dot) < 0.2:
                        node = np.array(node)-lane_width/2.0*np.array(line_vec)
                        node = node.tolist()
                    if abs(abs(dot)-1) < 0.5:
                        line_vec_list_tmp.append((-1*np.array(line_vec)).tolist())       
                print 'node at ', node, ' is T'
                node_dict["%f_%f"%(float(node[0]), float(node[1]))] = ["T", line_vec_list_tmp]
            continue
        elif len(pt_list) == 4:
            max_dist = -1
            max_pts = []
            for pair in itertools.combinations(pt_list, 2):
                dist = np.linalg.norm(np.array(pair[0]) - np.array(pair[1]))
                if dist > max_dist:
                    max_dist = dist
                    max_pts = [pair[0], pair[1]]
            print max_pts
            node = (np.array(max_pts[1]) + np.array(max_pts[0]))/2.0
            node = node.tolist()
            node_list.append(node)
            print 'node at ', node, ' is X'
            for line_pts in line_pts_list:
                line_vec = get_node_line_vec(line_pts, node)
                if line_vec not in line_vec_list:
                    line_vec_list.append(line_vec)
            if len(line_vec_list) > 4:
                line_vec_list_tmp = []
                for pair in itertools.combinations(line_vec_list, 2):
                    dot = np.dot(np.array(pair[0]), np.array(pair[1]))
                    print 'dot product of ', pair, ' is ', dot
                    if dot < 0.1:
                        line_vec_list_tmp.append(pair[0])
                        line_vec_list_tmp.append((-1*np.array(pair[0])).tolist())
                        line_vec_list_tmp.append(pair[1])
                        line_vec_list_tmp.append((-1*np.array(pair[1])).tolist())
                        line_vec_list = line_vec_list_tmp
                        break
            node_dict["%f_%f"%(float(node[0]), float(node[1]))] = ["X", line_vec_list]
            continue
    return node_list, node_dict

def get_line_vec(ball_list, ball_line_dict):
    line_vec_list = []
    for i in range(len(ball_list)):
        line_vec_list_tmp = []
        for ball in ball_list[i]:
            x,y = ball.centroid.xy
            ball_lines = ball_line_dict["%f_%f"%(float(x[0]), float(y[0]))]
            for line in ball_lines:
                x,y = line.xy
                p0 = np.array([x[0],y[0]])
                p1 = np.array([x[1],y[1]])
                line_vec = np.array(p1) - np.array(p0)
                line_vec = line_vec.tolist()

                line_vec_list_tmp.append(line_vec)
        print 'number of lines: ', len(line_vec_list_tmp)
        line_vec_list.append(line_vec_list_tmp)
    return line_vec_list

def plot_node(node_dict):
    global ax
    x_list = []
    y_list = []
    color = []
    for node, val in node_dict.items():
        x,y = node.split("_")
        x = float(eval(x))
        y = float(eval(y))
        x_list.append(x)
        y_list.append(y)
        if val[0] == "L":
            color.append("y")
        elif val[0] == "T":
            color.append("b")
        elif val[0] == "X":
            color.append("g")
        else:
            color.append("r")
        for vec in val[1]:
            dx,dy = vec
            arrow = patches.Arrow(x,y,0.5*dx,0.5*dy,width=0.3)
            ax.add_artist(arrow)
    ax.scatter(x_list,y_list,c=color)

def get_intersection(map_pts, line_map):
    global lane_width
    radius = (lane_width+0.05)/2.0*np.sqrt(2)
    ball_list = []
    ball_line_dict = {}
    node_dict = {}
    for pt in map_pts:
        ball = Point(pt[0], pt[1]).buffer(radius)
        line_list = get_ball_line(ball, line_map)
        ball_line_dict["%f_%f"%(float(pt[0]), float(pt[1]))] = line_list
        ball_list = check_ball_list(ball, ball_list, ball_line_dict)
    plot_ball_list(ball_list, radius)
    node_list, node_dict = get_node_list(ball_list, node_dict, ball_line_dict)
    # line_vec_list = get_line_vec(ball_list, ball_line_dict)
    plot_node(node_dict)


if __name__ == '__main__':
    fig = plt.figure()
    ax = plt.axes(xlim=(-3,5), ylim=(-1,8))
    node_plot, = ax.plot([], [], color='b', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    line_map, line_map_plot, map_poly = map_gen.map_gen(map_path, np.array(p0), np.array(p1))
    map_pts = get_map_pts(map_poly)
    intersection = get_intersection(map_pts, line_map)
    plot_line(line_map)
    plt.show()