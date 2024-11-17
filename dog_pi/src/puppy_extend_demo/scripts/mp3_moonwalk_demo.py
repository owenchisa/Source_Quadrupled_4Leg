#!/usr/bin/env python3
# coding=utf8
import os
import sys
import math
import rospy
import MP3 as MP3
from sensor.msg import *
from std_msgs.msg import *
from puppy_control.srv import SetRunActionName
from puppy_control.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************Function:MP3 module routine*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again
----------------------------------------------------------
''')

PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
# stance_x：refers to the distance in cm that front legs and hind legs move in the opposite direction on x axis
# stance_y：refers to the distance in cm that front legs and hind legs move in the opposite direction on y axis. As there is no servo controlling legs to move along Y axis, this parameter is useless.
# x_shift: the distance that 4 legs move in the same direction on x axis. The smaller the value, the greater PuppyPi leans forward. The greater the value, the greater PuppyPi leans backward. We can adjust x_shift to balance PuppyPi during it is walking
# height： refers to PuppyPi’s height that the perpendicular distance between foothold and the upper joint in cm
# pitch： PuppyPi’s pitch angle in degree


GaitConfig = {'overlap_time':0.3, 'swing_time':0.2, 'clearance_time':0.0, 'z_clearance':5}
# overlap_time: time all 4 legs touch the ground in s
# swing_time：time two legs lift ooff in s
# clearance_time：interval between the movements of front leg and hind leg in s
# z_clearance：the height that the leg should be raised in cm

# multiple shafts integration
def linkage(times =1):
    # times: times of cycle
    for i in range(0,15,1):
        PuppyPose = {'roll':math.radians(i), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
            height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 30)
        rospy.sleep(0.03)
    for i in range(0,15,1):
        PuppyPose = {'roll':math.radians(15), 'pitch':math.radians(i), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
            height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 30)
        rospy.sleep(0.03)
        
    for s in range(times):
        for i in range(15,-15,-1):
            PuppyPose = {'roll':math.radians(i), 'pitch':math.radians(15), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 30)
            rospy.sleep(0.03)
        for i in range(15,-15,-1):
            PuppyPose = {'roll':math.radians(-15), 'pitch':math.radians(i), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 30)
            rospy.sleep(0.03)
             
        for i in range(-15,15,1):
            PuppyPose = {'roll':math.radians(i), 'pitch':math.radians(-15), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 30)
            rospy.sleep(0.03)
        for i in range(-15,15,1):
            PuppyPose = {'roll':math.radians(15), 'pitch':math.radians(i), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 30)
            rospy.sleep(0.03)
            

# close detection function
run_st = True
def Stop():
    global run_st
    run_st = False
    print('关闭中...')
    mp3.pause()
    runActionGroup_srv('stand.d6ac',True)
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    
            
if __name__ == "__main__":
    addr = 0x7b #sensor iic address
    mp3 = MP3.MP3(addr)
    mp3.volume(30) #set the volume as 20 before playing 
    mp3.playNum(25) #play song 3
    
    # initialize node    
    rospy.init_node('mp3_moonwalk_demo')
    rospy.on_shutdown(Stop)
    # subscribe puppy_control node. The path of code：/home/pi/puppy_pi/src/puppy_control/scripts/puppy.py
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
    rospy.sleep(0.3) # delay for a while
    # PuppyPi stands
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.5)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    
    # march on the spot
    PuppyVelocityPub.publish(x=0.01, y=0, yaw_rate=0) 
    rospy.sleep(3)
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    rospy.sleep(1)
    
    # multiple shafts integration
    linkage(2) 
    # PuppyPi stands 
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    rospy.sleep(0.5)
    
    # move forward
    PuppyVelocityPub.publish(x=5, y=0, yaw_rate=0) 
    rospy.sleep(3)
    
    PuppyVelocityPub.publish(x=-5, y=0, yaw_rate=0) 
    rospy.sleep(2)
    # move backward
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    rospy.sleep(1)
    
    # execute the moonwalk action group
    runActionGroup_srv('moonwalk.d6ac',True)
    runActionGroup_srv('moonwalk.d6ac',True)
    rospy.sleep(0.5)
    
    # PuppyPi stand
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    rospy.sleep(0.5)
    # move forward
    PuppyVelocityPub.publish(x=5, y=0, yaw_rate=0) 
    rospy.sleep(5)
    # march on the spot
    PuppyVelocityPub.publish(x=0.01, y=0, yaw_rate=0) 
    rospy.sleep(3)
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    rospy.sleep(1)
    
    
    
            