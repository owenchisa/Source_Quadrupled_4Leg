#!/usr/bin/python3
# coding=utf8
# Date:2022/4/7
# Author:hiwonder
import sys
import cv2
import time
import math
import threading
import numpy as np
from enum import Enum

from puppy_pi import Misc

import rospy
from std_srvs.srv import *
from sensor_msgs.msg import Image
from sensor.msg import Led
from object_tracking.srv import *
from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName

ROS_NODE_NAME = 'negotiate_stairs_demo'

is_shutdown = False

color_range_list = rospy.get_param('/lab_config_manager/color_range_list')

PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

lock = threading.Lock()
debug = False
__isRunning = True
haved_detect = False

class PuppyStatus(Enum):
    LOOKING_FOR = 0 #bend down to search the stair

    FOUND_TARGET = 3 # the target stair is found
    DOWN_STAIRS = 4 # step down the stair

    STOP = 10
    END = 20            

puppyStatus = PuppyStatus.LOOKING_FOR
puppyStatusLast = PuppyStatus.END



target_centre_point = None # coordinate of the target center

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

color_list = []
detect_color = 'None'
action_finish = True
draw_color = range_rgb["black"]
__target_color = ('red',)


# Find out the contour with the maximum area
# The parameter is the list of the contour to be compared
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Traverse all the contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:  # Only when the area is greater than 300, the maximum contour is effective in order to avoid interference
                area_max_contour = c
    return area_max_contour, contour_area_max  # Return the maximum contour


def move():
    global detect_color
    global puppyStatus, puppyStatusLast, haved_detect, action_finish, target_centre_point, PuppyPose
    up_stairs_time = 0
    rospy.sleep(2)
    while True:
        rospy.sleep(0.01)

        # print(target_centre_point)

        while(puppyStatus == PuppyStatus.LOOKING_FOR) :
            if target_centre_point != None and target_centre_point[1] > 400:
                puppyStatus = PuppyStatus.FOUND_TARGET
                rospy.sleep(2.1) # continue to move forward
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))
                up_stairs_time = time.time()
                break
            
            PuppyVelocityPub.publish(x=10, y=0, yaw_rate = math.radians(0))
            # PuppyPose = PP['LookDown_20deg'].copy()
            # PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            #     ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
            rospy.sleep(0.01)
            break
        
        while(puppyStatus == PuppyStatus.FOUND_TARGET) :
            runActionGroup_srv('up_stairs_2cm.d6ac',True)
            if time.time() - up_stairs_time > 25:
                puppyStatus = PuppyStatus.DOWN_STAIRS
                PuppyPose = PP['Stand'].copy()
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
                rospy.sleep(0.5)
            break

        while(puppyStatus == PuppyStatus.DOWN_STAIRS) :
            PuppyVelocityPub.publish(x=14, y=0, yaw_rate = math.radians(0))
            rospy.sleep(0.1)
            break

        if puppyStatusLast != puppyStatus:
            print('puppyStatus',puppyStatus)
        puppyStatusLast = puppyStatus

        if is_shutdown:break

# Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
# th.start()


size = (320, 240)
# size = (640, 480)
def run(img):
    global line_centerx
    global __target_color, __isRunning, puppyStatus, target_centre_point, area_max
    
    # img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
            

    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert the image into LAB space

    for i in color_range_list:
        if i in __target_color:
            detect_color = i
            
            frame_mask = cv2.inRange(frame_lab,
                                            (color_range_list[detect_color]['min'][0],
                                            color_range_list[detect_color]['min'][1],
                                            color_range_list[detect_color]['min'][2]),
                                            (color_range_list[detect_color]['max'][0],
                                            color_range_list[detect_color]['max'][1],
                                            color_range_list[detect_color]['max'][2]))  # Perform bit operation on the original image and mask              
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
    #closed[:, 0:160] = 0
    #closed[:, 480:640] = 0        
    cnts = cv2.findContours(closed , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]# find out all the contours
    cnt_large, area_max = getAreaMaxContour(cnts)#The maximum area is found
    
    if cnt_large is not None:#if the contour isn't none
        rect = cv2.minAreaRect(cnt_large)#the smallest circumscribed rectangle
        box = np.int0(cv2.boxPoints(rect))#four vertices of the smallest circumscribed rectangle 
        
        centerX = rect[0][0]
        centerY = rect[0][1]
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        for i in range(4):
            box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
        for i in range(4):                
            box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
            
        cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#draw a rectangle composed of four points
        target_centre_point = [centerX, centerY]      
        cv2.circle(img, (int(centerX), int(centerY)), 5, (0,0,255), -1)#draw the center

        # #acquire the diagonal points of the rectangle
        # pt1_x, pt1_y = box[0, 0], box[0, 1]
        # pt3_x, pt3_y = box[2, 0], box[2, 1]            
        # center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#center
        # target_centre_point = [center_x, center_y]      
        # cv2.circle(img, (int(center_x), int(center_y)), 5, (0,0,255), -1)#draw the center
            
    return img


def image_callback(ros_image):
    # global lock
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # Convert the custom image data into image
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame
    # with lock:
    if __isRunning:
        frame_result = run(frame)
        if puppyStatus != PuppyStatus.FOUND_TARGET:
            cv2.imshow('Frame', frame_result)
            key = cv2.waitKey(1)
        rospy.sleep(0.001)

    # rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tobytes()
    # ros_image.data = rgb_image
    # image_pub.publish(ros_image)
def cleanup():
    global is_shutdown
    is_shutdown = True
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('is_shutdown')

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    rospy.on_shutdown(cleanup)

    PP = rospy.get_param('/puppy_control/PuppyPose')
    PuppyPose = PP['LookDown_10deg'].copy()
    PG = rospy.get_param('/puppy_control/GaitConfig')
    GaitConfig = PG['GaitConfigFast'].copy()

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    # image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher


    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)

    rospy.sleep(0.5)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    rospy.sleep(0.2)

    th.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()

