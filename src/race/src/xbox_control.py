#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import curses

from race.msg import drive_speed
from race.msg import drive_angle
from race.msg import drive_param

#pub_speed = rospy.Publisher('drive_speed', drive_speed, queue_size=10)
#pub_angle = rospy.Publisher('drive_angle', drive_angle, queue_size=10)
pub_param = rospy.Publisher('xbox_control', drive_param, queue_size=10)

stdscr = curses.initscr()

scale_vert = 0.20;
scale_hori = 1.0;

def callback(data):
	#vertical: left_stick, axes[1]  (down: -1.0 -- up: 1.0)
	#horizontal: right_stick, axes[2]  (right: -1.0 -- left: 1.0)
	if (data.buttons[1]==1):
		print("braking")
		horizontal = 0
		vertical = -20
		stdscr.refresh()
		stdscr.addstr(1, 25, 'Emergency Stop')
		stdscr.addstr(2, 25, '%.2f' % vertical)
		stdscr.addstr(3, 25, '%.2f' % horizontal)

	else:
		horizontal = -data.axes[2]*100*scale_hori       #Warning... this one changes often

		vertical = data.axes[1]*100*scale_vert

		stdscr.refresh()
		stdscr.addstr(1, 25, '               ')
		stdscr.addstr(2, 25, '%.2f' % vertical)
		stdscr.addstr(3, 25, '%.2f' % horizontal)


	# msg_speed = drive_speed()
	# msg_speed.speed = vertical
	# pub_speed.publish(msg_speed)

	# msg_angle = drive_angle()
	# msg_angle.angle = horizontal
	# pub_angle.publish(msg_angle)

	msg_param = drive_param()
	msg_param.velocity = vertical
	msg_param.angle = horizontal
	pub_param.publish(msg_param)

if __name__ == '__main__':
		rospy.Subscriber("joy", Joy, callback)
		rospy.init_node('xbox_wireless_controller')

		#TODO: check if drive_speed is already being pubilshed by someone else (such as speed_controller)

		stdscr.refresh()
		rospy.spin()
