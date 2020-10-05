#!/usr/bin/env python
import util

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

    BAG_CAMERA_TOPIC  = rospy.get_param("bag_detection/bag_camera_topic")
    TEST_CAMERA_TOPIC = rospy.get_param("bag_detection/test_camera_topic")
    BAG_POS_TOPIC     = rospy.get_param("bag_detection/bag_pos_topic")

    LOWER_COLOR       = rospy.get_param("bag_detection/lower_color_range")
    UPPER_COLOR       = rospy.get_param("bag_detection/upper_color_range")
    AREA              = rospy.get_param("bag_detection/contour_area")

    TR                = rospy.get_param("bag_detection/tr")
    TL                = rospy.get_param("bag_detection/tl")
    BR                = rospy.get_param("bag_detection/br")
    BL                = rospy.get_param("bag_detection/bl")

    SIDE              = rospy.get_param("bag_detection/side")

    def react_to_zed(self, data):
	try:
	    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	    print(e)

    	contours, hierarchy = util.get_contours(cv_image, self.LOWER_COLOR, self.UPPER_COLOR)

	if len(contours) > 0:
	    for contour in contours:
	        print('nut')
		    
	self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))
	

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
	self.bag_camera_sub = rospy.Subscriber(self.BAG_CAMERA_TOPIC, Image, callback = self.react_to_zed)
	self.image_pub = rospy.Publisher(self.TEST_CAMERA_TOPIC, Image, queue_size=10)
	self.bag_pos_pub = rospy.Publisher(self.BAG_POS_TOPIC, Image, queue_size=10)
	self.bridge = CvBridge()
	
	if self.SIDE == 1:
	    self.top_zone = self.TR
	    self.bot_zone = self.BR
	else:
            self.top_zone = self.TL
            self.bot_zone = self.BL
	print(self.top_zone)
	print(type(self.top_zone[0][0]))
	
if __name__ == "__main__":
    rospy.init_node('bagFlipModule')
    zedModule = bagFlipModule()
    rospy.spin()
