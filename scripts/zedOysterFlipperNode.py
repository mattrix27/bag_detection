#!/usr/bin/env python
import util

import numpy as np
import rospy
import cv2
import message_filters

from std_msgs.msg import UInt16
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
from cv2 import RANSAC


class ZEDOysterFlipper():
    RGB_CAMERA_TOPIC   = rospy.get_param("zed_depth_flipper/rgb_camera_topic", 'nut')
    DEPTH_CAMERA_TOPIC = rospy.get_param("zed_depth_flipper/depth_camera_topic", 'nut')
    FLIP_START_TOPIC   = rospy.get_param("zed_depth_flipper/flip_topic", 'nut')
    TEST_CAMERA_TOPIC  = rospy.get_param("zed_depth_flipper/test_path_camera_topic", 'nut')

    DETECTION_MODEL    = rospy.get_param("zed_depth_flipper/detection_model_path", 'nut')

    GRAY_LOWER_RANGE   = rospy.get_param("zed_depth_flipper/gray_lower_range", (0,5,0))
    GRAY_UPPER_RANGE   = rospy.get_param("zed_depth_flipper/gray_upper_range", (100,10,100))
    BLACK_LOWER_RANGE  = rospy.get_param("zed_depth_flipper/black_lower_range", (0,0,0))
    BLACK_UPPER_RANGE  = rospy.get_param("zed_depth_flipper/black_lower_range", (360,255,40))
    AREA               = rospy.get_param("zed_depth_flipper/flip_area", 100)
    DISTANCE_THRESHOLD = rospy.get_param("zed_depth_flipper/distance_threshold", 10)

    CROP_Y             = rospy.get_param("zed_depth_flipper/crop_y", (300,720))
    CROP_X             = rospy.get_param("zed_depth_flipper/crop_x", (400,1100))

    # GRAY_LOWER_RANGE  = (0,5,0)
    # GRAY_UPPER_RANGE  = (100,10,100)
    # BLACK_LOWER_RANGE = (0,0,0)
    # BLACK_UPPER_RANGE = (360,255,40)
    # AREA              = 100


    def __init__(self):

        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber(self.RGB_CAMERA_TOPIC, Image)
        self.depth_sub = message_filters.Subscriber(self.DEPTH_CAMERA_TOPIC, Image)
        self.flip_pub = rospy.Publisher(self.FLIP_START_TOPIC, UInt16, queue_size=5)

        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.callback)
        self.detect_fn = util.get_tf2_detect_fn(self.DETECTION_MODEL)
        self.flipping = False


    def callback(self, data):
        print(data)
        rgb_image = None
        depth_image = None

        objects = util.detect_objects(self.detect_fn, rgb_image)

        flipper_box = None
        bag_box = None
        for obj in objects:
            if obj['class'] == 1.0:
                if bag_box == None or bag_box['box'][1][1] < obj[1][1]:
                    bag_box = obj['box']
            elif obj['class'] == 2.0:
                flipper_box = obj['box']

        bag_depth = self.get_object_depth(bag_box, rgb_image, depth_image, self.BLACK_LOWER_RANGE, self.BLACK_UPPER_RANGE)
        flipper_depth = self.get_object_depth(flipper_box, rgb_image, depth_image, self.GRAY_LOWER_RANGE, self.GRAY_UPPER_RANGE)

        counter = 0
        if bag_depth < flipper_depth and bag_depth+self.DISTANCE_THRESHOLD > flipper_depth:
            print("ALIGNED!")
            counter += 1
            if counter > 10:
                print("FLIPPPPPPPPPPINGGG")
                self.flipping = True
                counter = 0
        else:
            counter = 0
            if bag_depth < flipper_depth:
                print("TOOO CLOSEEEEEE!!!")
            else:
                pirnt("TOOOO FARRRRRRR!!!")


    def get_object_depth(self, bag_box, rgb_image, depth_image, lower, upper):
        x1, y1 = bag_box[0]
        x2, y2 = bag_box[1] 
        crop_rgb = rgb_image[y1:y2,x1:x2]
        crop_depth = depth_image[y1:y2,x1:x2]

        mask = util.color_segmentation(crop_rgb, lower, upper)
        pixels = util.get_mask_pixels(mask)

        return util.get_avg_depth(crop_depth, pixels)

    
if __name__ == "__main__":
    rospy.init_node('ZEDOysterFlipper')
    flipModule = ZEDOysterFlipper()
    rospy.spin()
