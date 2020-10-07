#!/usr/bin/env python
import util

import numpy as np
import rospy
import cv2

from std_msgs.msg import UInt16
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from bag_detection.msg import FlipPos

from cv_bridge import CvBridge, CvBridgeError
from cv2 import RANSAC

class bagFlipModule:

    BAG_CAMERA_TOPIC  = rospy.get_param("bag_detection/flip_camera_topic")
    TEST_CAMERA_TOPIC = rospy.get_param("bag_detection/test_flip_camera_topic")
    BAG_POS_TOPIC     = rospy.get_param("bag_detection/flip_pos_topic")

    LOWER_COLOR       = rospy.get_param("bag_detection/lower_color_range")
    UPPER_COLOR       = rospy.get_param("bag_detection/upper_color_range")
    AREA              = rospy.get_param("bag_detection/contour_area")

    TR                = rospy.get_param("bag_detection/tr")
    TL                = rospy.get_param("bag_detection/tl")
    BR                = rospy.get_param("bag_detection/br")
    BL                = rospy.get_param("bag_detection/bl")

    SIDE              = rospy.get_param("bag_detection/side")


    def update_bag_message(bag_msg, rect):
        x, y, w, h = rect

        bot_x = (x+(w/2)) - ((self.bot_zone[0][0] + self.bot_zone[1][0])/2)
        bot_y = (y+(h/2)) - ((self.bot_zone[0][1] + self.bot_zone[1][1])/2)
        top_x = (x+(w/2)) - ((self.top_zone[0][0] + self.top_zone[1][0])/2)
        top_y = (y+(h/2)) - ((self.top_zone[0][1] + self.top_zone[1][1])/2)

        if ((bot_x**2)+(bot_y**2)**2)**0.5 < ((top_x**2)+(top_y**2)**2)**0.5:
            if abs(bot_x) < abs(bag_msg.bot_x) and abs(bot_y) < abs(bag_msg.bot_y):
                bag_msg.bot_x = bot_x
                bag_msg.bot_y = bot_y
        else:
            if abs(top_x) < abs(bag_msg.top_x) and abs(top_y) < abs(bag_msg.top_y):
                bag_msg.top_x = top_x
                bag_msg.top_y = top_y

        return bag_msg


    def react_to_zed(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv, np.array(self.LOWER_COLOR), np.array(self.UPPER_COLOR))
        # red_only = cv2.bitwise_and(hsv,hsv,mask = mask)
        
        center = cv_image.shape[1]/2
        if self.SIDE > 0:
            cropped_image = cv_image[:,:center]
        else:
            cropped_image = cv_image[:,center:]

        rectangles = util.get_rectangles(cropped_image, self.LOWER_COLOR, self.UPPER_COLOR, self.AREA, self.SIDE)

        top_color = (0,0,255)
        bot_color = (0,0,255)
        green = (0,255,0)
        if len(rectangles) > 0:
            bag_msg = util.create_flip_pos_msg()
            for rect in rectangles:
                #cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255,0,0), 2)
                #cv2.rectangle(red_only, (x,y), (x+w, y+h), (255,0,0), 2)
                bag_msg = update_bag_message(bag_msg, rect)
     
            if abs(bag_msg.bot_x) < ((self.bot_zone[1][0]-self.bot_zone[0][0])/2) and abs(bag_msg.bot_y) < ((self.bot_zone[1][1]-self.bot_zone[0][1])/2):
                bag_msg.bot = True
                bot_color = green
            if abs(bag_msg.top_x) < ((self.top_zone[1][0]-self.top_zone[0][0])/2) and abs(bag_msg.top_y) < ((self.top_zone[1][1]-self.top_zone[0][1])/2):
                bag_msg.top = True
                top_color = green

            self.bag_pos_pub.publish(bag_pos)

        #cv2.rectangle(cropped_image, tuple(self.bot_zone[0]), tuple(self.bot_zone[1]), bot_color, 3)
        #cv2.rectangle(cropped_image, tuple(self.top_zone[0]), tuple(self.top_zone[1]), top_color, 3)

        #cv2.rectangle(red_only, tuple(self.bot_zone[0]), tuple(self.bot_zone[1]), bot_color, 3)
        #cv2.rectangle(red_only, tuple(self.top_zone[0]), tuple(self.top_zone[1]), top_color, 3)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(red_only, encoding="passthrough"))


    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.bag_camera_sub = rospy.Subscriber(self.BAG_CAMERA_TOPIC, Image, callback = self.react_to_zed)
        self.image_pub = rospy.Publisher(self.TEST_CAMERA_TOPIC, Image, queue_size=10)
        self.bag_pos_pub = rospy.Publisher(self.BAG_POS_TOPIC, BagError, queue_size=10)
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
