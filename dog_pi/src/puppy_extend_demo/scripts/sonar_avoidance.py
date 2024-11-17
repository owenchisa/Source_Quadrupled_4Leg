#!/usr/bin/env python3
import os
import sys
import math
import rospy
import Sonar as Sonar
from puppy_control.msg import Velocity, Pose, Gait

print('''
**********************************************************
********************function:ultrasonic avoidance routine**********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again.
----------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
# stance_x：refers to the distance in cm that front legs and hind legs move in the opposite direction on x axis
# stance_y：refers to the distance in cm that front legs and hind legs move in the opposite direction on y axis. As there is no servo controlling legs to move along Y axis, this parameter is useless.
# x_shift: the distance that 4 legs move in the same direction on x axis. The smaller the value, the greater PuppyPi leans forward. The greater the value, the greater PuppyPi leans backward. We can adjust x_shift to balance PuppyPi during it is walking
# height： refers to PuppyPi’s height that the perpendicular distance between foothold and the upper joint in cm
# pitch： PuppyPi’s pitch angle in degree


GaitConfig = {'overlap_time':0.15, 'swing_time':0.2, 'clearance_time':0.0, 'z_clearance':3}
# overlap_time: time all 4 legs touch the ground in s
# swing_time：time two legs lift ooff in s
# clearance_time：interval between the movements of front leg and hind leg in s
# z_clearance：the height that the leg should be raised in cm

# close detection function
run_st = True
def Stop():
    global run_st
    run_st = False
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('关闭中...')
    

if __name__ == '__main__':
    s = Sonar.Sonar()
    s.setRGBMode(0) # 0:colored light module, 1: breathing light mode
    s.setRGB(1, (0, 0, 0)) # close RGB light
    s.setRGB(0, (0, 0, 0))
    
    # initialize the node
    rospy.init_node('Sonar_avoidance')
    rospy.on_shutdown(Stop)
    # subscribe puppy_control node. The path of code：/home/pi/puppy_pi/src/puppy_control/scripts/puppy.py
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    rospy.sleep(0.2)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    
    forward = True
    
    while run_st:
        rospy.sleep(0.1)
        distance = s.getDistance() # acquire the detected distance
        print('distance: {}(mm)'.format(distance))
        if distance <= 300: # the distance is smaller than 300mm
            if not forward:
                forward = True
                s.setRGB(1, (255, 0, 0)) # set tthe light as red
                s.setRGB(0, (255, 0, 0))
                PuppyVelocityPub.publish(x=5, y=0, yaw_rate=0.3) # turn left
                rospy.sleep(6)
            
        else:
            if forward:
                forward = False
                s.setRGB(1, (0, 0, 255)) # set tthe light as blue
                s.setRGB(0, (0, 0, 255))
                PuppyVelocityPub.publish(x=15, y=0, yaw_rate=0) # move forward
            
    s.setRGB(1, (0, 0, 0))
    s.setRGB(0, (0, 0, 0))
