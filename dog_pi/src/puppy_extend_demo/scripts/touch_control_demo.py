#!/usr/bin/python3
#coding=utf8
import os
import sys
import time
import math
import rospy
import RPi.GPIO as GPIO
from sensor.msg import *
from std_msgs.msg import *
from puppy_control.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************function:touch control routine*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again.
----------------------------------------------------------
''')

# the touch sensor should be connected to IO22 and IO24 interface on expansion board

PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
# stance_x：refers to the distance in cm that front legs and hind legs move in the opposite direction on x axis
# stance_y：refers to the distance in cm that front legs and hind legs move in the opposite direction on y axis. As there is no servo controlling legs to move along Y axis, this parameter is useless.
# x_shift: the distance that 4 legs move in the same direction on x axis. The smaller the value, the greater PuppyPi leans forward. The greater the value, the greater PuppyPi leans backward. We can adjust x_shift to balance PuppyPi during it is walking
# height： refers to PuppyPi’s height that the perpendicular distance between foothold and the upper joint in cm
# pitch： PuppyPi’s pitch angle in degree


GaitConfig = {'overlap_time':0.2, 'swing_time':0.2, 'clearance_time':0.0, 'z_clearance':3}
# overlap_time: time all 4 legs touch the ground in s
# swing_time：time two legs lift ooff in s
# clearance_time：interval between the movements of front leg and hind leg in s
# z_clearance：the height that the leg should be raised in cm

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# close detection function
run_st = True
def Stop():
    global run_st
    run_st = False
    print('关闭中...')


if __name__ == '__main__':
    # initialize the node    
    rospy.init_node('touch_control_demo')
    rospy.on_shutdown(Stop)
    # subscribe puppy_control node. The path of codes：/home/pi/puppy_pi/src/puppy_control/scripts/puppy.py
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    rospy.sleep(0.5) # delay for a while
    # PuppyPi stands
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    
    GPIO.setup(22, GPIO.IN) #set the pin as the input mode
    click = 0
    squat = True
    while run_st:
        state = GPIO.input(22)  #read the pin number
        rospy.sleep(0.05)
        if not state:
            detect_time = time.time()+1
            while time.time() < detect_time:
                state = GPIO.input(22)  #read the pin number
                rospy.sleep(0.1)
                if not state:
                    click += 1
                    rospy.sleep(0.1)
            
            if click == 1:
                click = 0
                if squat:
                    # PuppyPi will squat
                    PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-6, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                            height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
                    rospy.sleep(1)
                    squat = False
                    
                elif not squat:
                    # PuppyPi stands
                    PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                            height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
                    rospy.sleep(1)
                    squat = True
                    
            elif click == 2:
                click = 0
                # the robot will shake
                for i in range(5):
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                        height=PuppyPose['height'], roll=math.radians(3.5), pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 50)
                    rospy.sleep(0.13)
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                        height=PuppyPose['height'], roll=math.radians(-3.5), pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 50)
                    rospy.sleep(0.13)
                
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                    height=PuppyPose['height'], roll=math.radians(0), pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 300)
                rospy.sleep(1)
       
        
    
