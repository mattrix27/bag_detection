#!/usr/bin/env python

import numpy as np
import cv2

from bag_detection.msg import FlipPos, PathPos

def get_rectangles(cv_image, lower_range, upper_range, threshold_area):
    """
    Extract defined color from image and return rectangles coordinates of large enough contours on given side
    Input: 
        cv_image: Image (BGR)
        lower_range: 1x3 tuple representing lower HSV for target color
        upper_range: 1x3 tuple representing upper HSV for target color
        threshold_area: int
        side: 1 for Right, -1 for Left
    Output:
        list of 1x4 tuples (x, y, w, h) of color blobs 
    """
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_range), np.array(upper_range))
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    rectangles = []
    for contour in contours:
        if cv2.contourArea(contour) > threshold_area:
            rect = cv2.boundingRect(contour)
            rectangles.append(rect)
    return rectangles

def get_contours(cv_image, lower_range, upper_range, threshold_area):
    """
    Extract defined color from image and return large contours (UNUSED)
    Input: 
        cv_image: Image (BGR)
        lower_range: 1x3 tuple representing lower HSV for target color
        upper_range: 1x3 tuple representing upper HSV for target color
        threshold_area: int
    Output:
        list of openCV contours 
    """
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_range), np.array(upper_range))
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    return [x for x in contours if cv2.contourArea(x) > threshold_area], hierarchy



def color_segmentation(image, lower, upper):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    return mask


def get_mask_pixels(mask):
    return np.transpose((mask>0).nonzero())


def get_avg_depth(depth_img, pixels, low_thres=0, high_thres=1000):
    avg_depth = 0
    i = 0
    for x,y in pixels:
        depth = depth_img[x][y]
        # print(depth)
        if depth > low_thres and depth < high_thres: 
            avg_depth += depth
            i += 1

    return avg_depth/i


def get_region_box(smask, area=100, side='bottom', image=None):
    left = mask.shape[1]
    right = 0
    top = mask.shape[0]
    bot = 0
    box = None

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > area:
            rect = cv2.boundingRect(contour)
            if image:
                tl = (rect[0], rect[1])
                br = (rect[0]+rect[2], rect[1]+rect[3])
                cv.rectangle(image, tl, br, (255,0,0), 2)
            if side == 'left':
                if rect[0] < left:
                    left = rect[0]
                    box = rect
            elif side == 'right':
                if rect[0] > right:
                    right = rect[0]
                    box = rect
            elif side == 'top':
                if rect[1] < top:
                    top = rect[1]
                    box = rect
            else:
                if rect[1] > bot:
                    bot = rect[1]
                    box = rect
    if image:
        cv.rectangle(image, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0,0,255), 2)
    return box

def create_flip_pos_msg(top=False, bot=False):

    msg = FlipPos()
    msg.top = top
    msg.bot = bot
    msg.top_x = float('inf')
    msg.top_y = float('inf')
    msg.bot_x = float('inf')
    msg.bot_y = float('inf')

    return msg
