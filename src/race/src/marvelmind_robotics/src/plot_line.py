import matplotlib.pyplot as plt
from matplotlib import collections as mc
import sys
import numpy

map_path = 'map.txt'
map = open(map_path, 'r')
counter = 0
map_lines = []
for line in map:
    if counter%2 == 0:
        map_pts = eval(line)
    elif counter%2 == 1:
        map_lines = eval(line)
    counter += 1

print 'map_pts: ', map_pts
print 'map_lines: ', map_lines

line_pts = []
for pair in map_lines:
    line_pt = [tuple(map_pts[pair[0]]), tuple(map_pts[pair[1]])]
    line_pts.append(line_pt)

print 'line points: ', line_pts
color = [(255.0/255,200.0/255,10.0/255,5.0/255)]
lc = mc.LineCollection(line_pts, linewidths=1)

'''
fig, ax = plt.subplots()
ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)
plt.show()
'''
#create plot
global fig
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot([],[], 'ro')
ax.add_collection(lc)
axes = plt.gca()
axes.set_xlim([0,10])
axes.set_ylim([-5,5])
axes.add_collection(lc)

plt.show()
