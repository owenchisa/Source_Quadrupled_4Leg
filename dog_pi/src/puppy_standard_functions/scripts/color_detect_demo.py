#!/usr/bin/python3
# coding=utf8
# Date:2022/4/28
# Author:hiwonder
import sys
import cv2
import math
import rospy
import threading
import numpy as np
from threading import RLock, Timer
from std_srvs.srv import *
from sensor_msgs.msg import Image
from sensor.msg import Led

from puppy_pi import Misc


ROS_NODE_NAME = 'color_detect_demo'

color_range_list = {}
detect_color = 'None'
color_list = []




range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
draw_color = range_rgb["black"]

# Find out the contour with the maximum area
# The parameter is the list of the contour to be compared
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # traverse all the contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # Only when the area is greater than 50, the maximum contour is effective in order to avoid interference
                area_max_contour = c

    return area_max_contour, contour_area_max  # return the maximum contour
def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish
    size = (320, 240)

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert the image into LAB space

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    for i in color_range_list:
        if i in ['red', 'green', 'blue']:
            frame_mask = cv2.inRange(frame_lab,
                                            (color_range_list[i]['min'][0],
                                            color_range_list[i]['min'][1],
                                            color_range_list[i]['min'][2]),
                                            (color_range_list[i]['max'][0],
                                            color_range_list[i]['max'][1],
                                            color_range_list[i]['max'][2]))  #perform bit operation on the original image and the mask
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #Corrode
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #Dilate
            # if debug:
            #     cv2.imshow(i, dilated)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #Find out the contour
            areaMaxContour, area_max = getAreaMaxContour(contours)  #Find out the maximum contour
            if areaMaxContour is not None:
                if area_max > max_area:#Find out the maximum area
                    max_area = area_max
                    color_area_max = i
                    areaMaxContour_max = areaMaxContour
    if max_area > 200:  # The maximum area is found
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # Acquire the minimum circumscribed circle
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))            
        cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)#Draw circle

        if color_area_max == 'red':  #Red area is the largest
            color = 1
        elif color_area_max == 'green':  #Green area is the largest
            color = 2
        elif color_area_max == 'blue':  #Blue area is the largest
            color = 3
        else:
            color = 0
        color_list.append(color)

        if len(color_list) == 3:  #Judge for multiple times
            # Take the average
            color = int(round(np.mean(np.array(color_list))))
            color_list = []
            if color == 1:
                detect_color = 'red'
                draw_color = range_rgb["red"]
            elif color == 2:
                detect_color = 'green'
                draw_color = range_rgb["green"]
            elif color == 3:
                detect_color = 'blue'
                draw_color = range_rgb["blue"]
            else:
                detect_color = 'None'
                draw_color = range_rgb["black"]     
            print('detect_color is',detect_color)          
    else:
        detect_color = 'None'
        draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img

def image_callback(ros_image):

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # Convert the custom image data into image
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame

    frame_result = run(frame)
    cv2.imshow('Frame', frame_result)
    key = cv2.waitKey(1)
    # rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tobytes()
    # ros_image.data = rgb_image
    # image_pub.publish(ros_image)



if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)

    color_range_list = rospy.get_param('/lab_config_manager/color_range_list', {})
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
