#!/usr/bin/env python3
import util

import numpy as np
import rospy
import cv2

from std_msgs.msg import UInt16
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
#from bag_detection.msg import FlipPos

from cv_bridge import CvBridge, CvBridgeError
from cv2 import RANSAC

class bagFlipModule:

    BAG_CAMERA_TOPIC  = rospy.get_param("bag_flip_detection/flip_camera_topic")
    TEST_CAMERA_TOPIC = rospy.get_param("bag_flip_detection/test_flip_camera_topic")
    BAG_POS_TOPIC     = rospy.get_param("bag_flip_detection/flip_pos_topic")
    MODE_TOPIC        = rospy.get_param("bag_flip_detection/mode_topic")

    #LOWER_COLOR       = rospy.get_param("bag_flip_detection/lower_color_range")
    #UPPER_COLOR       = rospy.get_param("bag_flip_detection/upper_color_range")
    #AREA              = rospy.get_param("bag_flip_detection/flip_area")
    DILATION          = rospy.get_param("bag_flip_detection/dilation", 2)
    THRES_AREA_PROP   = rospy.get_param("bag_flip_detection/thres_area_prop", 0.05)
    SHEAR_FACTOR      = rospy.get_param("bag_flip_detection/shear_factor", 50)
    
    TL                = rospy.get_param("bag_flip_detection/tl")
    BR                = rospy.get_param("bag_flip_detection/br")
    VERTICAL          = rospy.get_param("bag_flip_detection/vertical", 1)
    DIRECTION         = rospy.get_param("bag_flip_detection/direction", 1)
    AREA_PROP         = rospy.get_param("bag_flip_detection/area_prop", 0.5)

    WIDTH             = rospy.get_param("bag_flip_detection/width")
    HEIGHT            = rospy.get_param("bag_flip_detection/height")

    SIDE              = rospy.get_param("bag_flip_detection/side")

    MODE              = 0

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.bag_camera_sub = rospy.Subscriber(self.BAG_CAMERA_TOPIC, Image, callback=self.react_to_camera)
        self.mode_sub = rospy.Subscriber(self.MODE_TOPIC, UInt16, callback=self.update_mode)
        self.image_pub = rospy.Publisher(self.TEST_CAMERA_TOPIC, Image, queue_size=10)
        self.edge_pub = rospy.Publisher("edgetest", Image, queue_size=10)
        self.blob_pub = rospy.Publisher("blobtest", Image, queue_size=10)
        self.bag_pos_pub = rospy.Publisher(self.BAG_POS_TOPIC, UInt16, queue_size=10)
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


    def react_to_camera(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        bag_msg = None

        top_color = (0,0,255)
        bot_color = (0,0,255)
        green = (0,255,0)
    
        if bag_msg:
            if bag_msg.top:
                top_color = green
            if bag_msg.bot:
                bot_color = green

        element = util.get_element(self.DILATION)
        edges = util.canny(cv_image)
        closed = util.dilate_bag_row(edges, element)
        closedS = util.directional_shear(closed, element, self.VERTICAL, self.SHEAR_FACTOR) 
        h, w = cv_image.shape[:2]
        threshold_area = self.THRES_AREA_PROP*h*w
        c_rects = util.get_rectangles(closedS, threshold_area)
        #c_rects = util.bag_rect_detection(cv_image, self.VERTICAL)
        inbound_area = 0
        error = 9999
        for rect in c_rects:
            top_l = (rect[0], rect[1])
            bot_r = (rect[0]+rect[2], rect[1]+rect[3])
            cv2.rectangle(cv_image, top_l, bot_r, (255,0,0), 5)
            if (self.VERTICAL):
                if ((top_l[1] > self.TL[1] and top_l[1] < self.BR[1]) and (bot_r[1] > self.TL[1] and bot_r[1] < self.BR[1])):
                    inbound_area += (bot_r[0] - top_l[0]) * (bot_r[1] - top_l[1])
                elif (top_l[1] < self.TL[1] and bot_r[1] > self.TL[1]):
                    # GO FORWARD
                    error = self.TL[1] - top_l[1]
               
            else:
                if ((top_l[0] > self.TL[0] and top_l[0] < self.BR[0]) and (bot_r[0] > self.TL[0] and bot_r[0] < self.BR[0])):
                    inbound_area += (bot_r[0] - top_l[0]) * (bot_r[1] - top_l[1])
                elif (top_l[0] < self.TL[0] and bot_r[0] > self.TL[0]):
                    # GO FORWARD
                    error = self.TL[0] - top_l[0]

        if (inbound_area >= self.AREA_PROP*((self.BR[0] - self.TL[0]) * (self.BR[1] - self.TL[1]))):
            error = 0

        

        self.bag_pos_pub.publish(error)
                 

        #FOR VIEWING
        cv2.rectangle(cv_image, tuple(self.TL), tuple(self.BR), (0,0,255), 10)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))
        self.edge_pub.publish(self.bridge.cv2_to_imgmsg(edges, encoding="passthrough"))
        self.blob_pub.publish(self.bridge.cv2_to_imgmsg(closed, encoding="passthrough"))


    def update_mode(self, data):
        print("UPDATE MODE: ", data)
        self.MODE = data.data

    
if __name__ == "__main__":
    rospy.init_node('bagFlipModule')
    flipModule = bagFlipModule()
    rospy.spin()
