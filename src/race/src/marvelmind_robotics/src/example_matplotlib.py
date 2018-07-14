from marvelmind import MarvelmindHedge
from time import sleep
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import sys
import numpy
from threading import Thread




def update_line():

    xdata = numpy.append(plt.gca().lines[0].get_xdata(), plt.gca().lines[1].get_xdata())
    ydata = numpy.append(plt.gca().lines[0].get_ydata(), plt.gca().lines[1].get_ydata())

    pos = hedge.position()
    new_x = pos[1]
    new_y = pos[2]
    plt.gca().lines[0].set_xdata(xdata[-29:])
    plt.gca().lines[0].set_ydata(ydata[-29:])
    
    plt.gca().lines[1].set_xdata([new_x])
    plt.gca().lines[1].set_ydata([new_y])
    
    plt.draw()

def printThread():
    while True:
        try:
            #sleep(3)
            pos = hedge.position()
            print (pos) # get last position and print
        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()


def main():
    
    #read map
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
    line_pts = []
    for pair in map_lines:
        line_pt = [tuple(map_pts[pair[0]]), tuple(map_pts[pair[1]])]
        line_pts.append(line_pt)
    
    lc = mc.LineCollection(line_pts, linewidths=1)
    #create plot
    global fig
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot([],[], 'ro')
    #ax.add_collection(lc)
    ax.grid(True)
    bx = fig.add_subplot(111)
    bx.plot([],[], 'bo')
    plt.axis('equal')
    cx = fig.add_subplot(111)
    cx.add_collection(lc)
    
    axes = plt.gca()
    axes.set_xlim([0,10])
    axes.set_ylim([-5,5])
    
    global hedge
    hedge = MarvelmindHedge(tty = "/dev/ttyACM2", adr=10, recieveUltrasoundPositionCallback=update_line) # create MarvelmindHedge thread
    hedge.start()
    
    plotThread = Thread(target=printThread) # create and start console out thread
    plotThread.start()
    
    plt.show()
    
main()
