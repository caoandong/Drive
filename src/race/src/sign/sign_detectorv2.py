#!/usr/bin/env python
import time as time
from datetime import datetime

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import tensorflow as tf

import ts_classifier.data as tscd
import ts_classifier.cnn as cnn
import ts_classifier.util as util

session = None
tf_x = None
top_k_predictions = None

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
#    2.0: 'speed limit 50'    
}

# Width of signs, in mm
sign_size_key = {
    14.0: 85.0,
    35.0: 65.0,
    2.0: 88.0
}


def image_callback(ros_image):
    global frame
    frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")


def initialize_sign_detection():
    global session, tf_x, top_k_predictions

    # Initialisation routines: generate variable scope, create logger, note start time.
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


class SignClassifier(object):
    """Use color information and contours to detect and classify potential signs
    in an image.

    Arguments
    ---------
        window (4 tuple)    : Specify the boundary of our sign detection window relative
            to the camera frame.
        params (dictionary) : A dictionary of parameters used in the sign
            detect method. Current valid keys include:

                - mask (str): The color mask to apply

    """
    def __init__(self, window = (150, 275, 300, 640), params = None):
        self.w = window
        self.padding = 5
        self.params = params
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def detect(self, frame):
        # Shape detection first
        s_candidates, s_rects = self.detect_circle(frame)
        c_candidates, c_rects = self.detect_color(frame)
        candidates = np.array(s_candidates + c_candidates)
        rects = np.array(s_rects + c_rects)

        return candidates, rects 


    def detect_circle(self, frame):
        """Initial step in processsing pipeline. Gets a list of candidate images
        from an input frame. Uses circles to get list of candidates.

        Arguments
        ---------
            frame (np.array) : An image to scan for potential signs.

        Returns
        -------
            A tuple containing the following two elements:

                - candidates (np.array)     : Subframes of our original frame that may
                    contain signs
                - bounding_rects (np.array) : An array of tuples (x, y, w, h) that indicate
                    the locations of the candidates in the original frame -- x and y represent
                    the top left corner of the candidate, and w and h are the width and height.
        """
        cv2.imshow('original', frame)
        temp_frame = frame.copy()
        gray = cv2.cvtColor(self.preprocess_frame(frame), cv2.COLOR_BGR2GRAY)
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8,
                                   param1=110, param2=35,
                                   minRadius=5, maxRadius=30)
        candidates, bounding_rects = [], []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # TODO smarter way to pad, probably, but for now just add static amount
            p = 5
            for i in circles[0, :]:
                x, y = (i[0] - i[2], i[1] - i[2])
                w, h = 2 * i[2], 2 * i[2]

                cv2.rectangle(temp_frame, (self.w[2] + x, self.w[0] + y), (self.w[2] + x + w, self.w[0] + y + h), (255, 0, 255), 1)

                # General filtering based on box dimensions
                if not self.box_filter((x, y, w, h), size_b = (100, 3500)): 
                    continue

                sign_box = frame[self.w[0] + y - p : self.w[0] + y + h + p, self.w[2] + x - p : self.w[2] + x + w + p]
                try:
                    resized_box = cv2.resize(sign_box, dsize=(32, 32))
                except cv2.error as e:
                    continue

                candidates.append(resized_box)
                bounding_rects.append((self.w[2] + x, self.w[0] + y, w, h))

        #cv2.imshow('all bounding contours', temp_frame)
        return candidates, bounding_rects

    def detect_color(self, frame):
        """Initial step in processsing pipeline. Gets a list of candidate images
        from an input frame.

        Arguments
        ---------
            frame (np.array) : An image to scan for potential signs.

        Returns
        -------
            A tuple containing the following two elements:

                - candidates (np.array)     : Subframes of our original frame that may
                    contain signs
                - bounding_rects (np.array) : An array of tuples (x, y, w, h) that indicate
                    the locations of the candidates in the original frame -- x and y represent
                    the top left corner of the candidate, and w and h are the width and height.
        """
        preprocessed = self.preprocess_frame(frame)
        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)
        output_hsv = hsv.copy()

        mask = self.params['mask'](hsv)

        output_hsv[np.where(mask==0)] = 0
        output_hsv = cv2.cvtColor(output_hsv, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('mask', output_hsv)


        output_hsv_copy = output_hsv.copy()
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13,13))
        kernel = np.ones((9, 9), np.uint8)
        dilated = cv2.dilate(output_hsv_copy, kernel)
        dilated = cv2.dilate(dilated, kernel)
        dilated = cv2.dilate(dilated, kernel)
        _, contours, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Filter depending on supplied parameters
        candidates = []
        bounding_rects = []
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            x, y, w, h = x + self.padding, y + self.padding, w - 3*self.padding, h - 3*self.padding
            cv2.rectangle(output_hsv_copy, (x, y), (x + w, y + h), (255, 0, 255), 1)

            # General filtering based on box dimensions
            if not self.box_filter((x, y, w, h)): continue

            # Filtering based on color information
            color_sign_box = frame[self.w[0] + y : self.w[0] + y + h, self.w[2] + x : self.w[2] + x + w]
            thresh_sign_box = output_hsv[y : y + h, x : x + w]
            #thresh_sign_box[thresh_sign_box < 125] = 0
            color_ratio = cv2.countNonZero(output_hsv_copy) / float(w * h)
            if color_ratio < 0.10:
                #print("Non zero {}, {}, {}".format(cv2.countNonZero(output_hsv_copy[y : y + h - 5, x : x + w - 5]), h, w))
                #print("Bad color {}".format(color_ratio))
                continue

            cv2.rectangle(output_hsv, (x, y), (x + w, y + h), (255, 0, 255), 1)
            try:
                resized_box = cv2.resize(color_sign_box, dsize=(32, 32))
            except cv2.error as e:
                continue 

            candidates.append(resized_box)
            bounding_rects.append((self.w[2] + x, self.w[0] + y, w, h))

        return candidates, bounding_rects

    def box_filter(self, box, size_b = (300, 5000), ratio_b = (0.8, 1.2)):
        """Filter the candidate bounding boxes by basic size/dimension. 
        """
        x, y, w, h = box
        if (w * h) < size_b[0] or (w * h) > size_b[1]:
            #print("Too small {} {}".format(w, h))
            return False

        size_ratio = float(w) / h
        if size_ratio > ratio_b[1] or size_ratio < ratio_b[0]:
            #print("Bad dims {} {}".format(w, h))
            return False

        return True

    def classify(self, frame, candidates=None, rects=None):
        """Given several candidate bounding boxes within our frame, run the classifier
        on them to check if they actually contain signs.

        Arguments
        ---------
            frame (np.array) : The frame in which we are evaluating our candidates
            candidates (np.array) : Sub-frames in our frame that potentially
                contain signs.

        Returns
        -------
            A tuple containing the following:

                - A modified frame with a box drawn around any successfully classified
                signs.
                - The sign class.
        """
        processed_frame = frame.copy()
        if candidates is None:
            candidates, rects = self.detect(frame)

        if candidates.size == 0:
            return (frame, None, None)

        #candidates = self.preprocess_candidates(candidates)
        X, _ = tscd.preprocess_dataset(candidates)
        [p] = session.run([top_k_predictions], feed_dict = {tf_x : X})
        predictions = np.array(p)
        max_prediction = np.amax(predictions, axis=1)
        max_ind = np.argmax(np.argmax(predictions, axis=1))

        sign_class, sign_dist, rf = None, None, None 
        if max_prediction[0, 0] > 0.80:
            # TODO do smarter processing here, can maybe look for second most likely and
            # see if that is in our relevant sign keys. 
            if max_prediction[1, 0] not in sign_name_key.keys():
                return (frame, None, None)
            print(predictions[1, 0])
            rf = rects[max_ind]
            sign_class = max_prediction[1, 0]
            sign_dist = get_distance(rf[2], sign_class)

        if rf is not None:
            #print(rf)
            cv2.rectangle(
                processed_frame, (rf[0], rf[1]), (rf[0] + rf[2], rf[1] + rf[3]), (255, 0, 255), 2)

        
        camera1_pub.publish(bridge.cv2_to_imgmsg(processed_frame, "bgr8"))
        return (processed_frame, sign_name_key.get(sign_class, None), sign_dist)

    def preprocess_frame(self, frame):
        frame = frame[self.w[0] : self.w[1], self.w[2] : self.w[3]]
        frame = cv2.medianBlur(frame, 5)
        frame = self._apply_clahe(frame)
        return frame

    def _apply_clahe(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lab_planes = cv2.split(lab)
        lab_planes[0] = self.clahe.apply(lab_planes[0])
        lab = cv2.merge(lab_planes)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)



def red_classifier():
    # lower mask
    low_lower_red = np.array([0,100,20])
    low_upper_red = np.array([10,255,255])

    # upper mask (170-180)
    hi_lower_red = np.array([165,100,20])
    hi_upper_red = np.array([180,255,255])

    def mask(hsv):
        mask1 = cv2.inRange(hsv, low_lower_red, low_upper_red)
        mask0 = cv2.inRange(hsv, hi_lower_red, hi_upper_red)
        return mask0 + mask1

    params = {'mask': mask}
    return SignClassifier(params=params)

def dark_red_classifier():
    """Experimental: When the stop sign is in shadow, this color range
    more accurately reflects what the camera picks up. If the sign
    is silhouetted or the background is low contrast, it doesn't help
    much.
    """
    lower_dark_red = np.array([150, 10, 10])
    upper_dark_red = np.array([170, 150, 150])

    def mask(hsv):
        mask1 = cv2.inRange(hsv, lower_dark_red, upper_dark_red)
        return mask1

    params = {'mask': mask}
    return SignClassifier(params=params)


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
    if sign_type not in sign_size_key.keys():
        return 0
    known_width = sign_size_key[sign_type]
    return (known_width * focal_length) / perceived_width


def sign_detector():
    rospy.init_node('sign_detector', anonymous=True)
    rospy.Subscriber('camera', Image, image_callback)

    initialize_sign_detection()
    rate = rospy.Rate(30)
    stop_sign_detector = red_classifier()
    while not rospy.is_shutdown():
        if frame is not None:
            candidates, rects = stop_sign_detector.detect(frame)
            processed_frame, sign_cls, sign_dist = stop_sign_detector.classify(
                frame, np.array(candidates), np.array(rects))
            cv2.imshow("sign detection", processed_frame)
            cv2.waitKey(1)
            if sign_cls:
                print("Detected {} at distance {}".format(sign_cls, sign_dist))
            else:
                print '-----------------------------------'
            

            new_ros_image1 = bridge.cv2_to_imgmsg(processed_frame, "bgr8")
            camera1_pub.publish(new_ros_image1)

        rate.sleep()
    cleanup_sign_detection()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print("sign_detector")
    sign_detector()
