#!/usr/bin/python3
#coding=utf8
import os
import sys
import rospy
from sensor.msg import *
from std_msgs.msg import *

# control buzzer

print('''
**********************************************************
******************Function:buzzer control routine***********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again.
----------------------------------------------------------
''')

if __name__ == '__main__':
    # Initialize the node   
    rospy.init_node('buzzer_control_demo')
    # buzzer node. The path of the codes:/home/pi/puppy_pi/src/sensor/scripts/sensor_control.py
    buzzer_pub = rospy.Publisher("/sensor/buzzer", Float32, queue_size=1)
    rospy.sleep(0.5) # delay for a while
     
    buzzer_pub.publish(2) # buzzer will sound for 2s
    rospy.sleep(3)
    buzzer_pub.publish(0.5) # buzzer will sound for 0.5s      
        
    
