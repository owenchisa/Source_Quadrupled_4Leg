#!/usr/bin/python3
#coding=utf8
import os
import sys
import math
import rospy
import RPi.GPIO as GPIO
from sensor.msg import *
from std_msgs.msg import *


print('''
**********************************************************
*******************function:touch detection routine************************
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

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# close detection function
run_st = True
def Stop():
    global run_st
    run_st = False
    print('关闭中...')
    buzzer_pub.publish(0)

if __name__ == '__main__':
    # initialize the node    
    rospy.init_node('buzzer_control_demo')
    rospy.on_shutdown(Stop)
    # buzzer node. The path of codes:/home/pi/puppy_pi/src/sensor/scripts/sensor_control.py
    buzzer_pub = rospy.Publisher("/sensor/buzzer", Float32, queue_size=1)
    rospy.sleep(0.5) # delay for a while
    
    GPIO.setup(22, GPIO.IN) #set the pin as the input mode
    st = 0
    while run_st:
        state = GPIO.input(22)  #read the pin number 
        if not state:
            if st :             #judge to avoid repetitive sounding
                st = 0
                buzzer_pub.publish(0.5) # buzzer sounds for 0.5s
                rospy.sleep(1)
        else:
            st = 1
            
        
    
