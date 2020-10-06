#!/usr/bin/env python
import numpy as np
import cv2

def get_rectangles(cv_image, lower_range, upper_range, threshold_area, side):
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

    center = cv_image.shape[1]/2
    rectangles = []
    for contour in contours:
	if cv2.contourArea(contour) > threshold_area:
	    rect = cv2.boundingRect(contour)
	    if (side * (rect[0] - center) > 0) or (side * ((rect[0]+rect[2]) - center) > 0):
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

