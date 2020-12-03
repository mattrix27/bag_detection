#!/usr/bin/env python
import util

import numpy as np
import rospy
import cv2

from std_msgs.msg import UInt16
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from bag_detection.msg import PathPos

from cv_bridge import CvBridge, CvBridgeError
from cv2 import RANSAC

class bagPathModule:

    BAG_CAMERA_TOPIC  = rospy.get_param("bag_path_detection/path_camera_topic")
    TEST_CAMERA_TOPIC = rospy.get_param("bag_path_detection/test_path_camera_topic")
    BAG_POS_TOPIC     = rospy.get_param("bag_path_detection/path_pos_topic")
    MODE_TOPIC        = rospy.get_param("bag_path_detection/mode_topic")

    LOWER_COLOR       = rospy.get_param("bag_path_detection/lower_color_range")
    UPPER_COLOR       = rospy.get_param("bag_path_detection/upper_color_range")
    AREA              = rospy.get_param("bag_path_detection/path_area")

    SIDE              = rospy.get_param("bag_path_detection/side")

    MODE              = 0

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.bag_camera_sub = rospy.Subscriber(self.BAG_CAMERA_TOPIC, Image, callback=self.react_to_camera)
        self.mode_sub = rospy.Subscriber(self.MODE_TOPIC, UInt16, callback=self.update_mode)
        self.image_pub = rospy.Publisher(self.TEST_CAMERA_TOPIC, Image, queue_size=10)
        self.bag_pos_pub = rospy.Publisher(self.BAG_POS_TOPIC, PathPos, queue_size=10)

        self.bridge = CvBridge()
        self.h_matrix, self.status = self.makeHomography()


    def react_to_camera(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        rectangles = sorted(util.get_rectangles(cv_image, self.LOWER_COLOR, self.UPPER_COLOR, self.AREA), key=lambda x: x[1], reverse=True)

        if len(rectangles) > 0:
            point_rect = None
            for rect in rectangles:
                point_rect = rect

            bag_msg = self.get_relative_pos(point_rect)
            self.bag_pos_pub.publish(bag_msg)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))


    def update_mode(self, data):
        self.MODE = data


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
