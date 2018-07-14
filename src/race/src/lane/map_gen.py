#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon
from matplotlib import collections as mc

# map_path = 'map.txt'

def map_gen(map_path, p0, p1):
    # have to provide the first and second transformed point in the map

    map = open(map_path, 'r')
    counter = 0
    map_lines = []
    for line in map:
        if counter%2 == 0:
            map_pts = eval(line)
        elif counter%2 == 1:
            map_lines = eval(line)
        counter += 1
    vec1 = p1 - p0
    vec2 = np.array(map_pts[1]) - np.array(map_pts[0])
    #print 'v1: ', vec1
    #print 'v2: ', vec2
    
    l1 = np.linalg.norm(vec1)
    l2 = np.linalg.norm(vec2)
    dot = np.dot(vec1, vec2)

    #print 'val: ', dot/(l1*l2)
    ang = np.arccos(dot/(l1*l2))

    #print 'angle: ', ang

    map_pts = np.transpose(np.array(map_pts))
    #print 'pts matrix: ', map_pts

    # Scaling:
    scale = float(l1)/l2
    #print 'scale: ', scale
    S = [[scale,0],[0,scale]]
    mat_pts = np.matmul(S, map_pts)
    #print 'Scaling: ', mat_pts

    # Rotation:
    R = np.array([[np.cos(ang), -np.sin(ang)],[np.sin(ang), np.cos(ang)]])
    map_pts = np.matmul(R, map_pts)
    #print 'Rotation: ', map_pts

    # Translation:
    for i in range(map_pts.shape[1]):
        map_pts[0][i] = map_pts[0][i] + p0[0]
        map_pts[1][i] = map_pts[1][i] + p0[1]

    map_pts = np.transpose(map_pts).tolist()
    #print 'Translation: ', map_pts

    lines = []
    line_pts = []
    for pair in map_lines:
        line_pt = [tuple(map_pts[pair[0]]), tuple(map_pts[pair[1]])]
        lines.append(LineString(line_pt))
        line_pts.append(line_pt)
    return lines, line_pts

def map_plot(line_pts):
    lc = mc.LineCollection(line_pts, linewidths=1)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot([],[], 'ro')
    ax.add_collection(lc)
    axes = plt.gca()
    axes.set_xlim([-2,5])
    axes.set_ylim([0,10])
    axes.add_collection(lc)

    plt.show()

# p0 = np.array([1.373, 2.211])
# p1 = np.array([3.942, 3.689])

# lines, line_pts = map_gen(map_path, p0, p1)

#plot it

