#!/usr/bin/env python

# generate the map plot
# find the intersection
# add each intersection into a graph
# test finding a shortest route

import numpy as np
import threading
import shapely
from shapely.geometry import Point, LineString, Polygon
from shapely import affinity
import map_gen
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.animation as anim
import matplotlib.patches as patches
import itertools
import networkx as nx
from collections import deque
from heapq import heappush, heappop
from itertools import count
from networkx.utils import generate_unique_node
import scipy as sp
import scipy.interpolate

map_path = '/home/antonio/catkin_ws/src/race/src/map/map_3.txt'
lane_width = 0.6
car_length = 0.34
turning_radius = 0.45
lane_probe = 0
# Map parameters that determine the size and orientation of the map
# p0 = [3.75, 4.66]
# p1 = [1.26, 2.88]

p0 = [3.50, 1.45]
p1 = [3.62, 1.95]

class Visualize (threading.Thread):
    def __init__(self):
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()
    def run(self):
        global fig, ax, line_map
        global lane_probe, probe_plot
        print 'Start visualize'
        plot_line(line_map)


def check_cross(vec1, vec2):
    cross = np.cross(np.array(vec1), np.array(vec2))
    return np.asscalar(cross)

def find_ang(vec1, vec2):
    ang_ret = 0
    if (np.array(vec1).tolist() != 0) and (np.array(vec2).tolist() != 0):
        vec1 = np.array(vec1)/float(np.linalg.norm(vec1))
        vec2 = np.array(vec2)/float(np.linalg.norm(vec2)) 
        dot = np.dot(vec1, vec2)

        cross = check_cross(vec1, vec2)
        if cross >= 0:
            ang_ret = np.arccos(np.clip(dot, -1.0, 1.0))
        else:
            ang_ret = -1*np.arccos(np.clip(dot, -1.0, 1.0))

    return ang_ret

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
                # print 'pt already found'
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

def plot_ball_list(ball_list, radius, ax):
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
        # print 'number of lines: ', len(lines_tmp)
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
            # print dist, lane_width, lane_width*np.sqrt(2)
            for line_pts in line_pts_list:
                line_vec = get_node_line_vec(line_pts, node)
                if line_vec not in line_vec_list:
                    line_vec_list.append(line_vec)
            if len(line_vec_list) > 2:
                line_vec_list_tmp = []
                for pair in itertools.combinations(line_vec_list, 2):
                    dot = np.dot(np.array(pair[0]), np.array(pair[1]))
                    # print 'dot product of ', pair, ' is ', dot
                    if dot < 0.1:
                        line_vec_list_tmp = [pair[0], pair[1]]
                        line_vec_list = line_vec_list_tmp
                        break
            if abs(dist - lane_width*np.sqrt(2)) < lane_width*0.2:
                # print 'node at ', node, ' is L'
                node_dict["%f_%f"%(float(node[0]), float(node[1]))] = ["L", line_vec_list, pt_list]
            elif abs(dist - lane_width) < lane_width*0.2:
                line_vec_list_tmp = line_vec_list[:]
                for line_vec in line_vec_list:
                    dot = np.dot(dist_vec, np.array(line_vec))
                    if abs(dot) < 0.2:
                        node = np.array(node)-lane_width/2.0*np.array(line_vec)
                        node = node.tolist()
                    if abs(abs(dot)-1) < 0.5:
                        line_vec_list_tmp.append((-1*np.array(line_vec)).tolist())       
                # print 'node at ', node, ' is T'
                node_dict["%f_%f"%(float(node[0]), float(node[1]))] = ["T", line_vec_list_tmp, pt_list]
            continue
        elif len(pt_list) == 4:
            max_dist = -1
            max_pts = []
            for pair in itertools.combinations(pt_list, 2):
                dist = np.linalg.norm(np.array(pair[0]) - np.array(pair[1]))
                if dist > max_dist:
                    max_dist = dist
                    max_pts = [pair[0], pair[1]]
            node = (np.array(max_pts[1]) + np.array(max_pts[0]))/2.0
            node = node.tolist()
            node_list.append(node)
            # print 'node at ', node, ' is X'
            for line_pts in line_pts_list:
                line_vec = get_node_line_vec(line_pts, node)
                if line_vec not in line_vec_list:
                    line_vec_list.append(line_vec)
            if len(line_vec_list) > 4:
                line_vec_list_tmp = []
                for pair in itertools.combinations(line_vec_list, 2):
                    dot = np.dot(np.array(pair[0]), np.array(pair[1]))
                    # print 'dot product of ', pair, ' is ', dot
                    if dot < 0.1:
                        line_vec_list_tmp.append(pair[0])
                        line_vec_list_tmp.append((-1*np.array(pair[0])).tolist())
                        line_vec_list_tmp.append(pair[1])
                        line_vec_list_tmp.append((-1*np.array(pair[1])).tolist())
                        line_vec_list = line_vec_list_tmp
                        break
            node_dict["%f_%f"%(float(node[0]), float(node[1]))] = ["X", line_vec_list, pt_list]
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
        # print 'number of lines: ', len(line_vec_list_tmp)
        line_vec_list.append(line_vec_list_tmp)
    return line_vec_list

def plot_node(node_dict, ax):
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

def update_anim(num):
    global lane_probe, probe_plot
    if type(lane_probe) == shapely.geometry.linestring.LineString:
        x,y = lane_probe.coords.xy
        x = np.array([x[0],x[1]])
        y = np.array([y[0],y[1]])
        probe_plot.set_data(x,y)
    return probe_plot,

def nodeidx_to_edgeidx(node_idx_1, node_idx_2, dist):
    node_idx = str(node_idx_1) + str(node_idx_2) + str(dist)
    edge_idx = [s for s in str(node_idx) if s.isdigit()]
    edge_idx_tmp = []
    for sublist in edge_idx:
        for item in sublist:
            if type(item) == str:
                for c in item:
                    edge_idx_tmp.append(c)
    edge_idx_tmp.sort(key=int)
    edge_idx_ret = ""
    for c in edge_idx_tmp:
        edge_idx_ret += c
    return edge_idx_ret

def create_node_graph(node_dict, line_map, ball_line_dict, ax=None):
    global lane_width, lane_probe
    G = nx.Graph()
    node_idx = 0
    node_ball_list = []
    graph_edges = {}
    for node, val in node_dict.items():
        x,y = node.split("_")
        x = float(eval(x))
        y = float(eval(y))
        node_idx = str(round(x,2))+str(round(y,2))
        if ax is not None:
            ax.text(x,y,"%s"%node_idx, fontsize=15)
        G.add_node(node_idx, pos=[x,y], val=val)
        ball = Point(x,y).buffer(lane_width/2.0)
        node_ball_list.append(ball)
    for node_idx, node_data in G.nodes(data=True):
        print 'The ', node_idx, ' node: ', node_data["val"][1]
        node_pos = node_data["pos"]
        node_val = node_data["val"]
        for vec in node_val[1]:
            hit = 0
            probe_length = 0
            node_hit = 0
            while hit == 0:
                probe_length += lane_width
                target = np.array(node_pos) + probe_length*np.array(vec)
                target = tuple(target.tolist())
                probe = LineString([tuple(node_pos), target])
                lane_probe = probe
                # print 'probe: ', probe
                for line in line_map:
                    if probe.intersects(line):
                        hit = 1
                        break
                for ball_hit in node_ball_list:
                    ball_x, ball_y = ball_hit.centroid.xy
                    error = np.sqrt((ball_x[0]-node_pos[0])**2+(ball_y[0]-node_pos[1])**2)
                    if error < 0.1:
                        continue
                    if probe.intersects(ball_hit):
                        hit = 1
                        ball_x, ball_y = ball_hit.centroid.xy
                        node_hit = [ball_x[0],ball_y[0]]
                        node_hit_idx = str(round(ball_x[0],2))+str(round(ball_y[0],2))
                        break
            if G.has_node(node_hit_idx):
                print 'graph has node: ', node_hit_idx
                nb_pos = G.nodes[node_hit_idx]['pos']
                dist = np.array(node_pos) - np.array(nb_pos)
                mid = np.array(nb_pos) + dist/2.0
                dist = np.linalg.norm(dist)
                if ax is not None:
                    ax.text(mid[0],mid[1],"%f"%dist, fontsize=10)
                G.add_edge(node_idx, node_hit_idx, dist=dist)
                edge = LineString([tuple(node_pos), tuple(nb_pos)])
                edge_idx = nodeidx_to_edgeidx(node_idx, node_hit_idx, dist)
                graph_edges[edge_idx] = edge
            else:
                print 'node hit: ', node_hit
                if type(node_hit) == list:
                    for nb_idx, nb_data in G.nodes(data=True):
                        if nb_idx == node_idx:
                            continue
                        # print 'checking ', nb_idx, ' my idx: ', node_idx
                        error = np.linalg.norm(np.array(node_hit) - np.array(nb_data['pos']))
                        # print 'nb position: ', nb_data['pos']
                        # print 'position error: ',error
                        if error < 0.1:
                            dist = np.array(node_pos) - np.array(nb_data['pos'])
                            mid = np.array(nb_data['pos']) + dist/2.0
                            dist = np.linalg.norm(dist)
                            if ax is not None:
                                ax.text(mid[0],mid[1],"%f"%dist, fontsize=10)
                            G.add_edge(node_idx, nb_idx, dist=dist)
                            edge = LineString([tuple(node_pos), tuple(nb_data['pos'])])
                            edge_idx = nodeidx_to_edgeidx(node_idx, nb_idx, dist)
                            graph_edges[edge_idx] = edge
                            break
    return G, graph_edges

def get_map_graph(map_pts, line_map, ax=None):
    global lane_width
    radius = (lane_width+0.05)/2.0*np.sqrt(2)
    ball_list = []
    ball_line_dict = {}
    node_dict = {}
    graph = []
    map_edges = []
    for pt in map_pts:
        ball = Point(pt[0], pt[1]).buffer(radius)
        line_list = get_ball_line(ball, line_map)
        ball_line_dict["%f_%f"%(float(pt[0]), float(pt[1]))] = line_list
        ball_list = check_ball_list(ball, ball_list, ball_line_dict)
    if ax is not None:
        plot_ball_list(ball_list, radius, ax)
    node_list, node_dict = get_node_list(ball_list, node_dict, ball_line_dict)
    if ax is not None: 
        plot_node(node_dict, ax)
        graph, map_edges = create_node_graph(node_dict, line_map, ball_line_dict, ax=ax)
    else:
        graph, map_edges = create_node_graph(node_dict, line_map, ball_line_dict)
    return graph, map_edges

def dist_pt_line(pt, line_pt, line_slope):
    a = np.array(line_pt)
    n = np.array(line_slope)
    p = np.array(pt)
    dist = (a-p)-np.dot(n, (a-p))*n
    dist = np.linalg.norm(dist)
    return dist

def find_nearby_nodes(start, map_graph, map_edges):
    global lane_width
    node_idx_list = []
    ball = Point(start[0], start[1]).buffer(lane_width/2.0)
    # dist_min = lane_width*1000
    for key,line in map_edges.items():
        if ball.intersects(line):
            x,y = line.xy
            p0 = [x[0],y[0]]
            p1 = [x[1],y[1]]
            line_slope = np.array(p1) - np.array(p0)
            line_slope = line_slope/np.linalg.norm(line_slope)
            dist = dist_pt_line(np.array(start), p0, line_slope)
            idx0 = str(round(x[0],2))+str(round(y[0],2))
            idx1 = str(round(x[1],2))+str(round(y[1],2))
            node_idx_list.append(idx0)
            node_idx_list.append(idx1)
            # if dist < dist_min:
            #     dist_min = dist
            #     node_idx_list.append(idx0)
            #     node_idx_list.append(idx1)
                # node_idx_list = [idx0, idx1]
                # if dist < lane_width/2.0:
                #     print 'return'
                #     return node_idx_list
    return node_idx_list

def find_path(start, end, orient, map_graph, map_edges, line_map):
    global lane_width
    path_ret = []
    orient = np.array(orient)
    orient = orient/np.linalg.norm(orient)
    dist_min = lane_width*10000
    start_node_idx_list = find_nearby_nodes(start, map_graph, map_edges)
    end_node_idx_list = find_nearby_nodes(end, map_graph, map_edges)
    start_idx = 0
    start_node = []
    start_dist = 0
    dot_min = 999
    for start_idx_tmp in start_node_idx_list:
        start_node_tmp = map_graph.nodes[start_idx_tmp]
        start_node_pos = start_node_tmp['pos']
        start_dist_vec = np.array(start_node_pos) - np.array(start)
        start_dist_tmp = np.linalg.norm(start_dist_vec)
        # if start_dist_tmp < lane_width/2.0:
        #     print 'change start node: ', start
        #     start = start_node_pos
        #     start_idx = start_idx_tmp
        #     start_node = start_node_tmp
        #     start_dist = start_dist_tmp
        #     break
        start_dist_vec = start_dist_vec/start_dist_tmp
        dot = np.dot(start_dist_vec, orient)
        print 'start idx: ', start_idx_tmp, ' dot: ', dot
        if dot > 0.5 and dot < dot_min:
            dot_min = dot
            start_idx = start_idx_tmp
            start_node = start_node_tmp
            start_dist = start_dist_tmp
    print 'start idx: ', start_idx
    for end_idx in end_node_idx_list:
        if end_idx == start_idx:
            print 'start and end are the same'
            direct = np.array(end) - np.array(start)
            direct = direct/np.linalg.norm(direct)
            dot = np.dot(direct, orient)
            if dot >= 0.5:
                print 'dot: ', dot
                probe = LineString([tuple(start), tuple(end)])
                check = 1
                for line in line_map:
                    if probe.intersects(line):
                        print 'intersects'
                        check = 0
                        break
                if check == 0:
                    continue
                else:
                    print 'Just go'
                    return []
            else:
                continue
        end_node = map_graph.nodes[end_idx]
        end_node_pos = end_node['pos']
        print 'start_dist: ', start_dist
        end_dist = np.array(end_node_pos) - np.array(end)
        end_dist = np.linalg.norm(end_dist)
        print 'end_dist: ', end_dist
        path_dist = nx.shortest_path_length(map_graph, source=start_idx, target=end_idx, weight="dist")
        dist = path_dist + start_dist + end_dist
        print 'total dist: ', dist
        if dist < dist_min:
            dist_min = dist
            path_ret = my_shortest_path(map_graph, source=start_idx, target=end_idx, weight="dist", orient=orient, stop_pt=end)
    return path_ret

def my_single_source_dijkstra(G, source, target=None, cutoff=None, weight='weight', orient=None, stop_pt=None):
    global lane_width, turning_radius
    if source == target:
        print 'source is the same as the target'
        return ({source: 0}, {source: [source]})
    push = heappush
    pop = heappop
    dist = {}  # dictionary of final distances
    paths = {source: [source]}  # dictionary of paths
    seen = {source: 0}
    c = count()
    fringe = []  # use heapq with (distance,label) tuples
    push(fringe, (0, next(c), source, orient))
    stop_ball = 0
    if stop_pt is not None:
        stop_ball = Point(stop_pt[0], stop_pt[1]).buffer(lane_width/2.0)
    while fringe:
        (d, _, v, orient_tmp) = pop(fringe)
        print 'vertex? ', v
        print 'orientation: ', orient_tmp
        if v in dist:
            continue  # already searched this node.
        dist[v] = d
        if v == target:
            break
        v_pos = G.nodes[v]['pos']
        # for ignore,w,edgedata in G.edges_iter(v,data=True):
        # is about 30% slower than the following
        if G.is_multigraph():
            edata = []
            for w, keydata in G[v].items():
                minweight = min((dd.get(weight, 1)
                                 for k, dd in keydata.items()))
                edata.append((w, {weight: minweight}))
        else:
            edata = iter(G[v].items())
        print 'G[v]: ', G[v]
        for w, edgedata in edata:
            print 'edata w:', w, ' edata edgedata: ', edgedata
            vw_dist = dist[v] + edgedata.get(weight, 1)
            w_orient = orient_tmp
            if orient is not None:
                w_pos = G.nodes[w]['pos']
                w_orient = np.array(w_pos) - np.array(v_pos)
                w_orient = w_orient/np.linalg.norm(w_orient)
                print 'w_orient: ', w_orient
                probe_end_pt = (np.array(v_pos) + vw_dist*w_orient).tolist()
                probe = LineString([tuple(v_pos), tuple(probe_end_pt)])
                dot = np.dot(orient_tmp, w_orient)
                print 'dot wv: ', dot
                if dot <= -0.35:
                    print 'orientation not right'
                    continue

                if probe.intersects(stop_ball):
                    print 'Stop because probe reaches the stop point'
                    paths[target] = paths[v]
                    return (dist, paths)

            if cutoff is not None:
                if vw_dist > cutoff:
                    continue
            if w in dist:
                if vw_dist < dist[w]:
                    raise ValueError('Contradictory paths found:',
                                     'negative weights?')
            elif w not in seen or vw_dist < seen[w]:
                seen[w] = vw_dist
                push(fringe, (vw_dist, next(c), w, w_orient))
                paths[w] = paths[v] + [w]
            
    return (dist, paths)

def my_dijkstra_path(G, source, target, weight='weight', orient=None, stop_pt=None):
    (length, path) = my_single_source_dijkstra(G, source, target=target, weight=weight, orient=orient, stop_pt=stop_pt)
    try:
        return path[target]
    except KeyError:
        raise nx.NetworkXNoPath(
            "node %s not reachable from %s" % (source, target))

def my_shortest_path(G, source=None, target=None, weight=None, orient=None, stop_pt=None):
    if source is None:
        if target is None:
            ## Find paths between all pairs.
            if weight is None:
                paths=nx.all_pairs_shortest_path(G)
            else:
                paths=nx.all_pairs_dijkstra_path(G,weight=weight)
        else:
            ## Find paths from all nodes co-accessible to the target.
            with nx.utils.reversed(G):
                if weight is None:
                    paths=nx.single_source_shortest_path(G, target)
                else:
                    paths=nx.single_source_dijkstra_path(G, target, weight=weight)

                # Now flip the paths so they go from a source to the target.
                for target in paths:
                    paths[target] = list(reversed(paths[target]))

    else:
        if target is None:
            ## Find paths to all nodes accessible from the source.
            if weight is None:
                paths=nx.single_source_shortest_path(G,source)
            else:
                paths=nx.single_source_dijkstra_path(G,source,weight=weight)
        else:
            ## Find shortest source-target path.
            if weight is None:
                paths=nx.bidirectional_shortest_path(G,source,target)
            else:
                if orient is None:
                    paths=nx.dijkstra_path(G,source,target,weight)
                else:
                    paths=my_dijkstra_path(G,source,target,weight,orient=orient, stop_pt=stop_pt)

    return paths

def plot_path(source, target, path, map_graph):
    global ax
    line_pts = []
    if len(path) > 0:
        p_prev = tuple(source)
        for p in path:
            node_pos_tmp = tuple(map_graph.nodes[p]['pos'])
            line_pts.append([p_prev, node_pos_tmp])
            p_prev = node_pos_tmp
        line_pts.append([p_prev, tuple(target)])
    else:
        line_pts = [[source, target]]
    lineCol = mc.LineCollection(line_pts, color="r", linewidths=2)
    ax.add_collection(lineCol)

def onclick(event):
    global ix, iy
    global ax
    global map_graph, map_edges, line_map
    ix, iy = event.xdata, event.ydata
    print 'x = %f, y = %f'%(ix, iy)
    global coords
    coords.append((ix, iy))
    if len(coords) == 2:
        fig.canvas.mpl_disconnect(cid)
        source = coords[0]
        target = coords[1]
        orient = [1,0.5]
        arrow = patches.Arrow(source[0],source[1],orient[0],orient[1],width=0.5)
        ax.add_artist(arrow)
        path = find_path(source, target, orient, map_graph, map_edges, line_map)
        print path
        plot_path(source, target, path, map_graph)
        num_pt = len(path)
        if num_pt > 0:
            pt_list = [list(source)]
            for i in range(num_pt):
                pt = map_graph.nodes[path[i]]['pos']
                pt_list.append(pt)
            pt_list.append(list(target))
            pt_list = parse_spline(pt_list, orient)
            spline_pts = draw_spline(pt_list)
            x_list = [pt[0] for pt in pt_list]
            y_list = [pt[1] for pt in pt_list]
            ax.plot(spline_pts[0], spline_pts[1], color='g', linewidth=3)
            ax.scatter(x_list,y_list, c='r')
            # spline = mc.LineCollection(spline_pts, color="g", linewidths=2)
            # ax.add_collection(spline)
        plt.show()

    return coords

def parse_spline(pts, orient):
    global turning_radius, car_length
    pts_list = []
    pts_list.append(pts[0])
    num_pts = len(pts)
    dist_0 = -1
    new_pt = np.array(pts[0]) + car_length*np.array(orient)
    pts_list.append(new_pt.tolist())
    for i in range(1,num_pts-1):
        p0 = pts[i-1]
        p1 = pts[i]
        p2 = pts[i+1]
        orient_0 = np.array(p0) - np.array(p1)
        if i == 1:
            dist_0 = np.linalg.norm(orient_0)
        orient_0 = orient_0/np.linalg.norm(orient_0)
        orient_1 = np.array(p2) - np.array(p1)
        orient_1 = orient_1/np.linalg.norm(orient_1)
        ang = find_ang(orient_0, orient_1)
        new0 = []
        new1 = []
        if ang >= np.radians(90.0):
            new0 = np.array(p1) + turning_radius*orient_0
            new1 = np.array(p1) + turning_radius*orient_1
            if i == 1 and dist_0 > 0:
                if dist_0 < turning_radius:
                    print 'tooooo short'
                    #new0 = np.array(p0) + car_length*np.array(orient)
                    new1 = np.array(p0) + car_length*np.array(orient) - turning_radius*orient_0 + turning_radius*orient_1
                    #pts_list.append(new0.tolist())
                    pts_list.append(new1.tolist())
                    continue
                elif dist_0 < turning_radius + car_length:
                    print 'tooo short'
                    pts_list.append(new1.tolist())
                    continue
        else:
            dist = turning_radius/abs(np.tan(ang/2.0))
            new0 = np.array(p1) + dist*orient_0
            new1 = np.array(p1) + dist*orient_1
            if i == 1 and dist_0 > 0:
                if dist_0 < dist:
                    print 'toooo short'
                    #new0 = np.array(p0) + car_length*np.array(orient)
                    new1 = np.array(p0) + car_length*np.array(orient) - dist*orient_0 + dist*orient_1
                    #pts_list.append(new0.tolist())
                    pts_list.append(new1.tolist())
                    continue
                elif dist_0 < dist + car_length:
                    print 'too short'
                    pts_list.append(new1.tolist())
                    continue
        pts_list.append(new0.tolist())
        pts_list.append(new1.tolist())
    pts_list.append(pts[-1])
    return pts_list

def draw_spline(pts):
    ctr = np.array(pts)
    x = ctr[:, 0]
    y = ctr[:, 1]
    l = len(x)
    t = np.linspace(0, 1, l-2, endpoint=True)
    t = np.append([0,0,0],t)
    t = np.append(t,[1,1,1])
    # print x
    # print y
    # print t
    tck = [t,[x,y],3]
    u3 = np.linspace(0,1,(max(l*2, 100)), endpoint=True)
    out = scipy.interpolate.splev(u3,tck)
    # tck, u = scipy.interpolate.splprep([x,y],k=3,s=0)
    # print 'tck: ', tck
    # u = np.linspace(0,1,num=100,endpoint=True)
    # out = scipy.interpolate.splev(u,tck)
    return out

if __name__ == '__main__':
    fig = plt.figure()
    ax = plt.axes(xlim=(-3,5), ylim=(-1,8))
    node_plot, = ax.plot([], [], color='b', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    probe_plot, = ax.plot([], [], color='r', alpha=0.5, fillstyle='full',
        linewidth=3, solid_capstyle='round')
    line_map, line_map_plot, map_poly = map_gen.map_gen(map_path, np.array(p0), np.array(p1), map_p0_idx=7, map_p1_idx=16)
    # line_map, line_map_plot, map_poly = map_gen.map_gen(map_path, np.array(p0), np.array(p1))
    map_pts = get_map_pts(map_poly)
    visual_thread = Visualize()
    map_graph, map_edges = get_map_graph(map_pts, line_map, ax=ax)
    print map_edges.items()
    coords = []
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    # source = [0.79,4.03]
    # target = [2.05, 3.8]
    # target = [1.07, 3.68]
    # target = [-2.31, 5.0]
    # orient = [-0.5, 1]
    # source = [2.56,3.44]
    # target = [2.17, 2.31]
    # orient = [1,0.5]
    # source = coords[0]
    # target = coords[1]
    # orient = [1,0.5]
    # path = find_path(source, target, orient, map_graph, map_edges, line_map)
    # print path
    # plot_path(source, target, path, map_graph)
    plt.show()
