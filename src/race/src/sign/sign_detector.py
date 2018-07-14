#!/usr/bin/env python

import rospy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import tensorflow as tf
import ts_classifier.cnn as cnn
import ts_classifier.data as tscd
import ts_classifier.util as util

from datetime import datetime

frame = None
bridge = CvBridge()

camera1_pub = rospy.Publisher("camera_sign", Image, queue_size=10)

# Current camera focal length (estimated)
focal_length = 319.15

# Tensorflow globals
#session = None
#tf_x = None
#top_k_predictions = None

# Sign information
sign_name_key = {
	14.0: 'stop',
	35.0: 'ahead only',
	2.0: 'speed limit 50'	
}

# Width of signs, in mm
sign_size_key = {
	14.0: 43.0,
	35.0: 43.0,
	2.0: 43.0
}


def image_callback(ros_image):
	global frame
	frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")


def initialize_sign_detection():
	global session, tf_x, top_k_predictions

    	params = cnn.default_params
    	paths = util.Paths(params)

	session = tf.InteractiveSession()
	tf_x = tf.placeholder(
		tf.float32, shape = (None, params.image_size[0], params.image_size[1], 1))
	is_training = tf.constant(False)
	with tf.variable_scope(paths.var_scope):
		predictions = tf.nn.softmax(cnn.model_pass(tf_x, params, is_training))
		top_k_predictions = tf.nn.top_k(predictions, 1)

	tf.train.Saver().restore(session, paths.model_path)


def cleanup_sign_detection():
	session.close()	


def detect_sign(frame):
	"""Detect whether a sign is in a frame.

	Arguments
	---------
		frame (array) : A numpy array representing an image.

	Returns
	-------
		If a sign is in the frame, returns a tuple: 
			1. string indicating which sign is in the frame
			2. estimated distance to sign 
		Otherwise, return None
	"""
	frame = frame[100:, 300:]
	if session is None or tf_x is None or top_k_predictions is None:
		raise ValueError("Tensorflow computation graph not initialized.")

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.medianBlur(gray, 5)
	rows = gray.shape[0]
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=7, maxRadius=50)

	boxframe = frame.copy()
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]: 
			p2 = (i[0] - i[2], i[1] + i[2])
			p2 = (i[0] + i[2], i[1] - i[2])
			try:
				cv2.rectangle(boxframe, p1, p2, (255, 0, 255), 3)
			except cv2.error as ce:
				pass

			# If color isn't great, ignore
			#if not check_color(frame, i):
			#	print 'check_color failed'
			#	continue
				

			# If we can't draw a bounding box, ignore
			resized_box = pad(frame, i)
			if resized_box is None:
				print 'padding is failing'
				continue
				

			X = np.array([resized_box])
			X, _ = tscd.preprocess_dataset(X)
			[p] = session.run([top_k_predictions], feed_dict = {tf_x : X})
			predictions = np.array(p)
			if predictions[0, 0, 0] > 0.70:
				sign_type = predictions[1, 0, 0]	
				# TODO sort of a hack until we start dealing w other sign types
				print sign_type
				if sign_type not in sign_name_key.keys():
					return (boxframe, None, None)

				return (boxframe, sign_name_key[sign_type], get_distance(2 * i[2], sign_type))
			else:
				print 'predictions[0, 0, 0] = %f' % predictions[0, 0, 0]


	return (boxframe, None, None)


def detect_sign_red_scan(frame):
	"""Look over entire frame for high concentration of red pixels.
	"""
	frame = frame[100:, 300:]
	if session is None or tf_x is None or top_k_predictions is None:
		raise ValueError("Tensorflow computation graph not initialized.")
	hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
	
	# lower mask
	lower_red = np.array([0,15,50])
	upper_red = np.array([10,255,255])
	mask0 = cv2.inRange(hsv, lower_red, upper_red)
	
	# upper mask (170-180)
	lower_red = np.array([170,15,50])
	upper_red = np.array([180,255,255])
	mask1 = cv2.inRange(hsv, lower_red, upper_red)

	return detect_sign_color_scan(frame, hsv, mask0 + mask1)	


def detect_sign_blue_scan(frame):	
	frame = frame[100:, 300:]
	if session is None or tf_x is None or top_k_predictions is None:
		raise ValueError("Tensorflow computation graph not initialized.")
	hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
	
	# lower mask
	lower_red = np.array([95,50,50])
	upper_red = np.array([135,255,255])
	mask0 = cv2.inRange(hsv, lower_red, upper_red)
	
	return detect_sign_color_scan(frame, hsv, mask0)	


def detect_sign_color_scan(frame, hsv, mask):

	# Test image based on mask	
	output_hsv = hsv.copy()
	output_hsv[np.where(mask==0)] = 0
	output_hsv = cv2.cvtColor(output_hsv, cv2.COLOR_BGR2GRAY)
	ratio = cv2.countNonZero(output_hsv) / (32 * 32)
	
	cv2.imshow("hsv filter", output_hsv)
	# bounding box
	output_hsv_copy = output_hsv.copy()
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
	dilated = cv2.dilate(output_hsv_copy, kernel)
	_, contours, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	correct = None

	boxframe = frame.copy()
	cnt_scores = []
	for cnt in contours:
		# Check if size is appropriate
		if cv2.contourArea(cnt) < 400 or cv2.contourArea(cnt) > 2500:
			continue

		(x, y, w, h) = cv2.boundingRect(cnt)

		# Check if shape is appropriate
		ratio = float(w) / h
		if ratio < 0.90 or ratio > 1.10:
			continue 

		cv2.rectangle(boxframe, (x, y), (x + w, y + h), (255, 0, 255), 2)
		
		# Feed to network
		#print(x, y, w, h)
		sign_box = frame[y:y + w, x:x + h]
		cv2.imshow('neural net', sign_box)
		sign_box = cv2.resize(sign_box, dsize=(32, 32))
		X = np.array([sign_box])
		X, _ = tscd.preprocess_dataset(X)
		[p] = session.run([top_k_predictions], feed_dict = {tf_x : X})
		predictions = np.array(p)
		if predictions[0, 0, 0] > 0.90:
			sign_type = predictions[1, 0, 0]	
			# TODO sort of a hack until we start dealing w other sign types
			print sign_type
			if sign_type not in sign_name_key.keys():
				result = (boxframe, None, None)

			else: 
				result = (boxframe, sign_name_key[sign_type], get_distance(w, sign_type))

			cnt_scores.append((predictions[0, 0, 0], result))

	if cnt_scores != []:
		return sorted(cnt_scores, key=lambda x: x[0])[-1][1]

	return (boxframe, None, None)

def check_color(frame, i):
	sign_frame = frame[(i[1] - i[2]):(i[1] + i[2]),(i[0] - i[2]):(i[0] + i[2])]
	try:
		resized_box = cv2.resize(sign_frame, dsize=(32, 32), interpolation=cv2.INTER_CUBIC)
	except cv2.error as e:
		return False
	
	hsv = cv2.cvtColor(resized_box, cv2.COLOR_BGR2HSV)
	
	# lower mask
	lower_red = np.array([0,50,50])
	upper_red = np.array([10,255,255])
	mask0 = cv2.inRange(hsv, lower_red, upper_red)
	
	# upper mask (170-180)
	lower_red = np.array([170,50,50])
	upper_red = np.array([180,255,255])
	mask1 = cv2.inRange(hsv, lower_red, upper_red)
	
	# Test how much of image is red
	mask = mask0 + mask1
	output_hsv = hsv.copy()
	output_hsv[np.where(mask==0)] = 0
	#cv2.imshow('output_hsv', output_hsv)
	#cv2.waitKey(1)
	output_hsv = cv2.cvtColor(output_hsv, cv2.COLOR_BGR2GRAY)
	ratio = cv2.countNonZero(output_hsv) / (32.0 * 32)
	#print ratio
	if ratio < 0.45:
		return False
	
	return True


def pad(frame, i):
	padding = [20, 15, 10, 7, 5, 2, 0]
	for p in padding:
		sign_box = frame[(i[1] - i[2] - p):(i[1] + i[2] + p),(i[0] - i[2] - p):(i[0] + i[2] + p)]
		try:
			resized_box = cv2.resize(sign_box, dsize=(32, 32))
		except cv2.error as e:
			resized_box = None
	return resized_box



def get_distance(perceived_width, sign_type):
	"""Returns the estimated distance to the sign.

	Arguments
	---------
		perceived_width (float) : The perceived width of the sign in the current 
			frame.
		sign_type (float) : The sign's class

	Returns
	-------
		A float giving the estimate distance.
	"""
	known_width = sign_size_key[sign_type]
	return (known_width * focal_length) / perceived_width


def sign_detector():
	rospy.init_node('sign_detector', anonymous=True)
	rospy.Subscriber('camera', Image, image_callback)

	initialize_sign_detection()
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		if frame is not None:
			#start_time = datetime.now()
			processed_frame, sign, dist = detect_sign_red_scan(frame)
			#prf_2, sign_2, dist_2 = detect_sign_blue_scan(frame)
			#end_time = datetime.now()
			#print 'elapsed: %s' % str(end_time - start_time)

			cv2.imshow("red sign detection", processed_frame)
			#cv2.imshow("blue sign detection", prf_2)
			cv2.waitKey(1)
			if sign and dist:
				print("{} at distance {}".format(sign, dist))
			#if sign_2 and dist_2:
			#	print("{} at distance {}".format(sign_2, dist_2))
			if not sign:
				print '-----------------------------------'
			

			new_ros_image1 = bridge.cv2_to_imgmsg(processed_frame, "bgr8")
			camera1_pub.publish(new_ros_image1)

		rate.sleep()
	cleanup_sign_detection()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	print("sign_detector")
	sign_detector()
