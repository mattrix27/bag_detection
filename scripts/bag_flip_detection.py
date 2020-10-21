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
    MODE_TOPIC        = rospy.get_param("bag_detection/mode_topic")

    LOWER_COLOR       = rospy.get_param("bag_detection/lower_color_range")
    UPPER_COLOR       = rospy.get_param("bag_detection/upper_color_range")
    AREA              = rospy.get_param("bag_detection/flip_area")

    TR                = rospy.get_param("bag_detection/tr")
    TL                = rospy.get_param("bag_detection/tl")
    BR                = rospy.get_param("bag_detection/br")
    BL                = rospy.get_param("bag_detection/bl")

    WIDTH             = rospy.get_param("bag_detection/width")
    HEIGHT            = rospy.get_param("bag_detection/height")

    SIDE              = rospy.get_param("bag_detection/side")

    MODE              = 0

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.bag_camera_sub = rospy.Subscriber(self.BAG_CAMERA_TOPIC, Image, callback=self.react_to_camera)
        self.mode_sub = rospy.Subscriber(self.MODE_TOPIC, UInt16, callback=self.update_mode)
        self.image_pub = rospy.Publisher(self.TEST_CAMERA_TOPIC, Image, queue_size=10)
        self.bag_pos_pub = rospy.Publisher(self.BAG_POS_TOPIC, FlipPos, queue_size=10)
        self.bridge = CvBridge()


    def update_bag_message(self, bag_msg, rect):
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


    def get_bag_message(self, rectangles, cv_image=None):
        bag_msg = util.create_flip_pos_msg()
        for rect in rectangles:
	    if type(cv_image) != type(None):
            	cv2.rectangle(cv_image, (rect[0],rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (255,0,0), 2)
            bag_msg = self.update_bag_message(bag_msg, rect)
        return bag_msg


    def react_to_camera(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv, np.array(self.LOWER_COLOR), np.array(self.UPPER_COLOR))
        # red_only = cv2.bitwise_and(hsv,hsv,mask = mask)
        
	center=cv_image.shape[1]/2
        if (self.SIDE > 0) ^ (self.MODE != 2):
            cropped_image = cv_image[:,center:]
            self.bot_zone = self.BR
            self.top_zone = self.TR
        else:
            cropped_image = cv_image[:,:center]
            self.bot_zone = self.BL
            self.top_zone = self.TL
            
        rectangles = util.get_rectangles(cropped_image, self.LOWER_COLOR, self.UPPER_COLOR, self.AREA)
		
	bag_msg = None
        if len(rectangles) > 0:
            bag_msg = self.get_bag_message(rectangles, cropped_image)
     
            if abs(bag_msg.bot_x) < ((self.bot_zone[1][0]-self.bot_zone[0][0])/2) and abs(bag_msg.bot_y) < ((self.bot_zone[1][1]-self.bot_zone[0][1])/2):
                bag_msg.bot = True
            if abs(bag_msg.top_x) < ((self.top_zone[1][0]-self.top_zone[0][0])/2) and abs(bag_msg.top_y) < ((self.top_zone[1][1]-self.top_zone[0][1])/2):
                bag_msg.top = True
            #print("AHHHH")
            self.bag_pos_pub.publish(bag_msg)


        #FOR VIEWING

        top_color = (0,0,255)
        bot_color = (0,0,255)
        green = (0,255,0)
	
        if bag_msg:
            if bag_msg.top:
                top_color = green
            if bag_msg.bot:
                bot_color = green
        cv2.rectangle(cropped_image, tuple(self.bot_zone[0]), tuple(self.bot_zone[1]), bot_color, 3)
        cv2.rectangle(cropped_image, tuple(self.top_zone[0]), tuple(self.top_zone[1]), top_color, 3)

        #cv2.rectangle(red_only, tuple(self.bot_zone[0]), tuple(self.bot_zone[1]), bot_color, 3)
        #cv2.rectangle(red_only, tuple(self.top_zone[0]), tuple(self.top_zone[1]), top_color, 3)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cropped_image, encoding="passthrough"))


    def update_mode(self, data):
	print("UPDATE MODE: ", data)
        self.MODE = data.data

    
if __name__ == "__main__":
    rospy.init_node('bagFlipModule')
    zedModule = bagFlipModule()
    rospy.spin()
