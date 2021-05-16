#!/usr/bin/env python3

import numpy as np
import cv2
import tensorflow as tf

import sys
sys.path.append("/home/oyster/Tensorflow/Monk_Object_Detection/13_tf_obj_2/lib/")
from infer_detector_nano import Infer

from bag_detection.msg import FlipPos, PathPos


def get_rectangles(mask, threshold_area):
    """
    Extract defined color from image and return rectangles coordinates of large enough contours on given side
    Input: 
        mask: Binary Image
        threshold_area: int
    Output:
        list of 1x4 tuples (x, y, w, h) of color blobs 
    """
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    rectangles = []
    for contour in contours:
        if cv2.contourArea(contour) > threshold_area:
            rect = cv2.boundingRect(contour)
            rectangles.append(rect)
    return rectangles


def get_contours(mask, threshold_area):
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


def get_tf2_detect_fn(path):
    detect_fn=tf.saved_model.load(path)
    return detect_fn

def detect_objects(detect_fn, image, width=1280, height=720, min_score_thres=0.5):
    image_np = np.array(image)
    input_tensor=tf.convert_to_tensor(image_np)
    input_tensor=input_tensor[tf.newaxis, ...]
    detections=detect_fn(input_tensor)
    print(type(detections))

    # This is the way I'm getting my coordinates
    boxes = detections['detection_boxes'][0]
    # print(boxes)
    # get all boxes from an array
    max_boxes_to_draw = boxes.shape[0]
    # get scores to get a threshold
    scores = detections['detection_scores'][0]
    # print(scores)
    # this is set as a default but feel free to adjust it to your needs
  
    # iterate over all objects found
    objects = []
    for i in range(min(max_boxes_to_draw, boxes.shape[0])): 
        if scores is None or scores[i] > min_score_thresh:
            class_name = detections['detection_classes'][0][i].numpy()

            y_min, x_min, y_max, x_max = boxes[i].numpy()
            tl, br = ((int(x_min*width), int(y_min*height)), (int(x_max*width), int(y_max*height)))
            detection = {'class':class_name, 'box': (tl, br)}
            objects.append(detection)

    return objects


def get_gtf();
    gtf = Infer();
    print("GTFF INITIALIZEDDDDDDDDDDDDDDDDDDDDDDDDDDD")
    gtf.set_dataset_params(class_list_file = '/home/oyster/Tensorflow/oyster_bag/classes.txt')
    print("DATA SET PARAMMMS SETTTTTT")
    gtf.set_model_params(exported_model_dir = '/home/oyster/Tensorflow/trt_fp16_dir')

    return gtf

def gtf_detect_objects(gtf, image_np, min_score_thres=0.5, width=1280, height=720):
    input_tensor = tf.convert_to_tensor(image_np)
    input_tensor = input_tensor[tf.newaxis, ...]
    scores, bboxes, labels = gtf.infer_on_tensor(input_tensor, thresh=0.8);
    
    return bboxes


def create_flip_pos_msg(top=False, bot=False):

    msg = FlipPos()
    msg.top = top
    msg.bot = bot
    msg.top_x = float('inf')
    msg.top_y = float('inf')
    msg.bot_x = float('inf')
    msg.bot_y = float('inf')

    return msg
