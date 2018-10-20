#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon
from matplotlib import collections as mc

# map_path = 'map.txt'

# map must be in strict order:
# largest outline: the first
# interior outlines: the next
# points: must be connected in order

def map_gen(map_path, p0, p1, map_p0_idx=0, map_p1_idx=1):
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
    p0 = np.array(p0)
    p1 = np.array(p1)
    map_p0 = np.array(map_pts[map_p0_idx])
    map_p1 = np.array(map_pts[map_p1_idx])
    vec1 = p1 - p0
    vec2 = map_p1 - map_p0
    delta_p0 = p0 - map_p0
    print 'v1: ', vec1
    print 'v2: ', vec2
    print 'p0: ', p0, ' map_p0: ', map_p0, ' p1: ', p1, ' map_p1: ', map_p1,' delta_p0: ', delta_p0
    
    l1 = np.linalg.norm(vec1)
    l2 = np.linalg.norm(vec2)
    dot = np.dot(vec1, vec2)

    print 'dot: ', dot/(l1*l2)
    ang = np.arccos(dot/(l1*l2))

    cross = np.cross(vec2, vec1)
    print 'cross: ', cross
    if cross < 0:
        ang = 2*np.pi - ang
    elif cross > 0:
        ang = ang
    else:
        assert abs(ang) <= 0.0001

    map_pts = np.transpose(np.array(map_pts))
    print 'pts matrix: ', map_pts

    # Translation:
    for i in range(map_pts.shape[1]):
        map_pts[0][i] = map_pts[0][i] - map_p0[0]
        map_pts[1][i] = map_pts[1][i] - map_p0[1]

    # Scaling:
    scale = float(l1)/l2
    print 'scale: ', scale
    S = [[scale,0],[0,scale]]
    map_pts = np.matmul(S, map_pts)

    # Rotation:
    print 'angle: ', np.degrees(ang)
    R = np.array([[np.cos(ang), -np.sin(ang)],[np.sin(ang), np.cos(ang)]])
    print 'Rotation: ', R.tolist()
    print 'Scaling: ', map_pts.tolist()
    map_pts = np.matmul(R, np.array(map_pts))
    print 'Rotation: ', map_pts

    # Translation:
    for i in range(map_pts.shape[1]):
        map_pts[0][i] = map_pts[0][i] + p0[0]
        map_pts[1][i] = map_pts[1][i] + p0[1]

    map_pts = np.transpose(map_pts).tolist()
    print 'Translation: ', map_pts

    lines = []
    line_pts = []
    for pair in map_lines:
        line_pt = [tuple(map_pts[pair[0]]), tuple(map_pts[pair[1]])]
        lines.append(LineString(line_pt))
        line_pts.append(line_pt)

    poly = [[]]
    poly_idx = []
    poly_count = 0
    for pair in map_lines:
        new_pts = [p for p in pair if p not in poly_idx]
        if len(new_pts) == 0:
            # create new poly
            poly_idx = []
            poly_count += 1
            poly.append([])
        else:
            for pt in new_pts:
                poly_idx.append(pt)
                poly[poly_count].append(tuple(map_pts[pt]))
    
    poly_list = []
    for pts in poly:
        if len(pts) >= 3:
            shape = Polygon(pts)
            poly_list.append(shape)
    # print poly_list[0].centroid.coords[0]

    return lines, line_pts, poly_list

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

# p0 = np.array([3.75, 4.66])
# p1 = np.array([1.26, 2.88])
# map_path = '/home/antonio/catkin_ws/src/race/src/map/map.txt'

# lines, line_pts, poly_list = map_gen(map_path, p0, p1)