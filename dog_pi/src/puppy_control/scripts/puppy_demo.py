#!/usr/bin/env python3
# coding=utf8

import sys
import math
import rospy
from std_srvs.srv import SetBool
from puppy_control.msg import Velocity, Pose, Gait


ROS_NODE_NAME = 'puppy_demo'

PuppyMove = {'x':6, 'y':0, 'yaw_rate':0}
# x:The first parameter “x” is the speed in cm/s of moving straight. Moving forward is taken as the positive direction.
# y:The second parameter “y” is the speed in cm/s of moving sideways. PuppyPi cannot move sideways, hence this parameter is useless.
# yaw_rate：The third parameter “yaw_rate” is the turning speed in rad/s. Counterclockwise direction is taken ad positive direction.

PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.3, 'stance_x':0, 'stance_y':0}
# PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
# stance_x：refers to the distance in cm that front legs and hind legs move in the opposite direction on x axis.
# stance_y：refers to the distance in cm that front legs and hind legs move in the opposite direction on y axis. 
# x_shift: the distance that 4 legs move in the same direction on x axis. The smaller the value, the greater PuppyPi leans forward. The greater the value, the greater PuppyPi leans backward. We can adjust x_shift to balance PuppyPi during it is walking.
# height： “height” refers to PuppyPi’s height that the perpendicular distance between foothold and the upper joint in cm
# pitch： PuppyPi’s pitch angle in degree


gait = 'Trot'
# overlap_time:the time all 4 legs touch the ground in second.
# swing_time：time single leg lifts off in second
# clearance_time：the phase interval between the legs at the digobal opposite ends of the body
# z_clearance：the height that the toe should be raised in cm

if gait == 'Trot':
    GaitConfig = {'overlap_time':0.2, 'swing_time':0.3, 'clearance_time':0.0, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.6
    # Trot clearance_time = 0

elif gait == 'Amble':
    GaitConfig = {'overlap_time':0.1, 'swing_time':0.2, 'clearance_time':0.1, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.9
    # Amble 0 ＜ clearance_time ＜ swing_time
    
elif gait == 'Walk':
    GaitConfig = {'overlap_time':0.1, 'swing_time':0.2, 'clearance_time':0.3, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.65
    # Walk swing_time ≤ clearance_time

def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('is_shutdown')

if __name__ == '__main__':

    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)
    # march on the spot

    rospy.sleep(0.2)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    rospy.sleep(0.2)

    PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])

    set_mark_time_srv(False)
    ## If PuppyPi still moves forward or backward during marching on the spot, the center of gravity of PuppyPi needs to be adjusted, and we just adjust PuppyPose['x_shift'] 


    while True:
        try:
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit(0)
        except :
            sys.exit(0)

