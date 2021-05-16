#!/usr/bin/env python3
import util

import numpy as np
import rospy
import cv2

import message_filters
from std_msgs.msg import UInt16
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from bag_detection.msg import PathPos

from cv_bridge import CvBridge, CvBridgeError
from cv2 import RANSAC

class bagPathModule:

    BAG_RGB_CAMERA_TOPIC   = rospy.get_param("bag_path_detection/path_rgb_camera_topic")
    BAG_DEPTH_CAMERA_TOPIC = rospy.get_param("bag_path_detection/path_depth_camera_topic")
    TEST_CAMERA_TOPIC      = rospy.get_param("bag_path_detection/test_path_camera_topic")
    BAG_POS_TOPIC          = rospy.get_param("bag_path_detection/path_pos_topic")
    MODE_TOPIC             = rospy.get_param("bag_path_detection/mode_topic")

    LOWER_COLOR            = rospy.get_param("bag_path_detection/lower_color_range")
    UPPER_COLOR            = rospy.get_param("bag_path_detection/upper_color_range")
    AREA                   = rospy.get_param("bag_path_detection/path_area")

    SIDE                   = rospy.get_param("bag_path_detection/side")
    MODE                   = 0

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.bag_rgb_camera_sub = message_filters.Subscriber(self.BAG_RGB_CAMERA_TOPIC, Image)
        self.bag_depth_camera_sub = message_filters.Subscriber(self.BAG_RGB_CAMERA_TOPIC, Image)
        self.ts = message_filters.TimeSynchronizer([self.bag_rgb_camera_sub, self.bag_depth_camera_sub], 10)
        self.ts.registerCallback(react_to_camera)

        self.mode_sub = rospy.Subscriber(self.MODE_TOPIC, UInt16, callback=self.update_mode)
        self.image_pub = rospy.Publisher(self.TEST_CAMERA_TOPIC, Image, queue_size=10)
        self.bag_pos_pub = rospy.Publisher(self.BAG_POS_TOPIC, PathPos, queue_size=10)
        self.first = True
        self.last_row = (0,0)

        self.bridge = CvBridge()
        self.h_matrix, self.status = self.makeHomography()

        self.gtf = util.get_gtf()
        if (self.SIDE == 1):
           self.corner = (0,720)
        else:
           self.corner = (1280,720) 


    def react_to_camera(self, rgb_data, depth_data):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
	
	bboxes = util.gtf_detect_objects(np.array(rgb_image))

        
        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))


    def update_mode(self, data):
        self.MODE = data.data


    def get_relative_pos(self, rect):
        bag_msg = PathPos()
        bag_msg.x = rect[0]
        bag_msg.y = rect[1]
        return bag_msg


    def makeHomography(self):
        return 1, 1
        

if __name__ == "__main__":
    rospy.init_node('bagPathModule')
    pathModule = bagPathModule()
    rospy.spin()
