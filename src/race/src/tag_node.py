#!/usr/bin/env python

import math
import rospy
import serial
from sensor_msgs.msg import Range
from std_msgs.msg import String

NODE_NAME = "decawave_tag"
DIST_TOPIC = "range"


class DecaWaveTag:

    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyACM1')
        baud = rospy.get_param('~baud', 115200)
        # self.tag_names = rospy.get_param("tag_names")
        self.offsets = rospy.get_param("offsets", [0,0,0,0,0])
        self.rate = rospy.Rate(rospy.get_param("frequency", 100))
        self.rng = Range()
        self.rng.field_of_view = math.pi * 0.1
        self.rng.min_range = 0
        self.rng.max_range = 300
        self.ser = serial.Serial(port=port, timeout=None, baudrate=baud)
        self.pub = rospy.Publisher('DecaWave_tag', String, queue_size=1)
        self.offset = 0.0

    def start(self):
        self.ser.close()
        self.ser.open()
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                pos, q = self.get_dist()
                if (pos is not None) and (q is not None) and (self.pub is not None):
                    self.pub.publish('[%s, %f]' % (str(pos), q))
            except:
                print 'No data'
                pass
            self.rate.sleep()
        self.ser.close()

    def get_dist(self):
        raw_data = self.ser.readline()
        data = raw_data.split()

        # if self.pub is None:
        #     try:
                  # tag_id = int(eval(data[-1].split(":")[0][3:])[-1])
        #         self.offset = float(self.offsets[tag_id])
        #         self.rng.header.frame_id = self.tag_names[tag_id]
        #         topic_name = "/{}/{}".format(self.tag_names[tag_id], DIST_TOPIC)
        #         topic_name = 'decawave_tag'
        #         self.pub = rospy.Publisher(topic_name, Range, queue_size=1)
        #     except IndexError:
        #         pass
        # self.rng.header.stamp = rospy.Time.now()


        # if len(data) > 0 and data[0] == 'mc':

        #     mask = int(data[1], 16)
        #     if (mask & 0x01):
        #         dist = int(data[2], 16) / 1000.0

        #     return dist + self.offset *(dist/8.0)

        try:
            pos = eval(data[-1].split(":")[0][3:])
            x = pos[0]
            y = pos[1]
            z = pos[2]
            q = pos[3]
            pos = [x,y,z]
            print 'my position: ', [x,y,z]
            print 'quality factor: ', q
            return pos, q
        except:
            try:
                pos = eval(data[-1].split("POS,")[-1])
                x = pos[0]
                y = pos[1]
                z = pos[2]
                q = pos[3]
                pos = [x,y,z]
                print 'my position: ', [x,y,z]
                print 'quality factor: ', q
                return pos, q
            except:
                pass



if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    da = DecaWaveTag()
    da.start()