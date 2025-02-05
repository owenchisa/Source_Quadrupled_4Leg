#!/usr/bin/python3
#coding=utf8
import os
import sys
import rospy
from sensor.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Imu

# posture detection

print('''
**********************************************************
******************function:posture detection routine*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again.
----------------------------------------------------------
''')

rgb_color = 'None'
def run(msg):
    global rgb_color
    
    x = msg.linear_acceleration.x
    y = msg.linear_acceleration.y
   
    if x >= 3: # robot tilts to left
        if rgb_color != 'red_r':
            print('机器人左倾斜')
            rgb_color = 'red_r'
            led = Led()
            led.rgb.r = 255
            led.rgb.g = 0
            led.rgb.b = 0
            led.index = 0
            rgb_pub.publish(led)
        
    elif x <= -3: # robot tilts to right
        if rgb_color != 'red_l':
            print('机器人右倾斜')
            rgb_color = 'red_l'
            led = Led()
            led.rgb.r = 255
            led.rgb.g = 0
            led.rgb.b = 0
            led.index = 1
            rgb_pub.publish(led)
    
    elif y >= 3: # robot tilts forward
        if rgb_color != 'red':
            print('机器人前倾斜')
            rgb_color = 'red'
            led = Led()
            led.rgb.r = 255
            led.rgb.g = 0
            led.rgb.b = 0
            led.index = 0
            rgb_pub.publish(led)
            rospy.sleep(0.01)
            led.index = 1
            rgb_pub.publish(led)
    
    elif y <= -3: # robot tilts backward
        if rgb_color != 'blue':
            print('机器人后倾斜')
            rgb_color = 'blue'
            led = Led()
            led.rgb.r = 0
            led.rgb.g = 0
            led.rgb.b = 255
            led.index = 0
            rgb_pub.publish(led)
            rospy.sleep(0.01)
            led.index = 1
            rgb_pub.publish(led)
    
    else:
        if rgb_color != 'None':
            print('机器人姿态正常')
            rgb_color = 'None'
            led = Led()
            led.rgb.r = 0
            led.rgb.g = 0
            led.rgb.b = 0
            led.index = 0
            rgb_pub.publish(led)
            rospy.sleep(0.01)
            led.index = 1
            rgb_pub.publish(led)

# close detection function
run_st = True
def Stop():
    global run_st
    run_st = False
    turn_off_rgb()
    print('关闭中...')

if __name__ == '__main__':
    # initialize nodes  
    rospy.init_node('posture_detect_demo')
    # RGB light node. The path of code:/home/pi/puppy_pi/src/sensor/scripts/sensor_control.py
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    # subscribe imu node. The path of code:/home/pi/puppy_pi/src/puppy_control/scripts/mpu6050Node.py
    imu_sub = rospy.Subscriber('/imu', Imu, run)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        
    
