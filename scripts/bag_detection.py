#!/usr/bin/env python
import numpy as np
import rospy
import cv2

from std_msgs.msg import UInt16
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from cv2 import RANSAC

class bagFlipModule:

    STATUS            = 0 # Normal, Incoming, Aligned

    BAG_CAMERA_TOPIC  = rospy.get_param("bag_flip_detection/bag_camera_topic")
    TEST_CAMERA_TOPIC = rospy.get_param("bag_flip_detection/test_camera_topic")
    SERVO_TOPIC       = rospy.get_param("bag_flip_detection/servo_topic")
    
    CLOSE_ANGLE       = rospy.get_param("bag_flip_detection/close_angle")
    OPEN_ANGLE        = rospy.get_param("bag_flip_detection/open_angle")

    LOWER_COLOR       = rospy.get_param("bag_flip_detection/lower_color_range")
    UPPER_COLOR       = rospy.get_param("bag_flip_detection/upper_color_range")
    AREA              = rospy.get_param("bag_flip_detection/contour_area")

    TR                = rospy.get_param("bag_flip_detection/tr")
    TL                = rospy.get_param("bag_flip_detection/tl")
    BR                = rospy.get_param("bag_flip_detection/br")
    BL                = rospy.get_param("bag_flip_detection/bl")

    SIDE              = rospy.get_param("bag_flip_detection/side")

    def react_to_zed(self, data):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

    	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    	mask = cv2.inRange(hsv, np.array(self.LOWER_COLOR), np.array(self.UPPER_COLOR))

    	contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
	if len(contours) > 0:
	    big_contours = [i for i in contours if cv2.contourArea(i) > self.AREA]
	    sorted_contours = sorted(big_contours, key = lambda x: cv2.contourArea(x), reverse=True)
	    cv2.drawContours(cv_image, [sorted_contours[0]], -1, (0, 255, 0), 3)
	    top = False
	    bot = False
	    if big_contours:
		for contour in big_contours:
		    x, y, w, h = cv2.boundingRect(contour)
		    if x > self.bot_zone[0][0] and (x+w) < self.bot_zone[1][0] and y > self.bot_zone[0][1] and (y+h) < self.bot_zone[1][1]:
			bot = True 
		    if x > self.top_zone[0][0] and (x+w) < self.top_zone[1][0] and y > self.top_zone[0][1] and (y+h) < self.top_zone[1][1]:
			top = True


	self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))
	

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
	self.bag_camera_sub = rospy.Subscriber(self.BAG_CAMERA_TOPIC, Image, callback = self.react_to_zed)
	self.image_pub = rospy.Publisher(self.TEST_CAMERA_TOPIC, Image, queue_size=10)
	self.servo_pub = rospy.Publisher(self.SERVO_TOPIC, UInt16, queue_size=10)
	self.bridge = CvBridge()
	
	if self.SIDE == 1:
	    self.top_zone = self.TR
	    self.bot_zone = self.BR
	else:
            self.top_zone = self.TL
            self.bot_zone = self.BL
	
if __name__ == "__main__":
    rospy.init_node('bagFlipModule')
    zedModule = bagFlipModule()
    rospy.spin()
