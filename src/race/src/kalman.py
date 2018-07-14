#!/usr/bin/env python

import sys
import os
import datetime
import rospy
from race.msg import kalman
import cv2
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np

# Publish:
# corrected position and velocity of the car

pos_pub = rospy.Publisher("kalman", kalman, queue_size=1)

# Predict
#	x0 - initial state; p0: initial covariant mat; u0: initial action (from IMU)
#	A, B, Q : pre-defined parameters
#	x1 = A*x0 + B*u0
#	p1 = A*P*A^T + Q^-1
def predict(x0, p0, u0, A, B, Q):
	x1 = np.matmul(A, x0)+np.matmul(B, u0)
	p1 = np.matmul(A, np.matmul(P, np.transpose(A))) + np.linalg.inv(Q)

	return x1, p1

# Update:
def update(x1, p1, z, H, R):
	# Innovation
	# v = z - H*x1
	v = z - np.matmul(H, x1)

	# S = H*p1*H^T + R
	S = np.matmul(H, np.matmul(p1, np.transpose(H))) + R

	# Kalman mat
	# k = p1*H^T*S^-1
	K = np.matmul(p1, np.matmul(np.transpose(H), np.linalg.inv(S)))

	# Update prediction
	x2 = x1 + np.matmul(K, v)

	# Update covariant matrix
	p2 = p1 - np.matmul(K, np.matmul(S, np.transpose(K)))

	return x2, p2

# Subscribe:
# GPS and IMU and xbox_control
# TODO: camera
def kalman():
	rospy.Subscriber("joy", Joy, joy_callback_2)