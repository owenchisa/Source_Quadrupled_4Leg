#!/usr/bin/python3
#coding=utf8
import os
import sys
import rospy
import pigpio
from sensor.msg import *
from std_msgs.msg import *

# key control

print('''
**********************************************************
*********************function:key control routine**********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again.
----------------------------------------------------------
''')

# close RGB light
def turn_off_rgb():
    led = Led()
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    led.index = 0
    rgb_pub.publish(led)
    rospy.sleep(0.01)
    led.index = 1
    rgb_pub.publish(led)

# set RGB light display
def set_rgb_show(r,g,b):
    led = Led()
    led.rgb.r = r
    led.rgb.g = g
    led.rgb.b = b
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
    # initialized key configuration
    key1_pin = 25
    key2_pin = 23
    pi = pigpio.pi()
    pi.set_mode(key1_pin, pigpio.INPUT)
    pi.set_mode(key2_pin, pigpio.INPUT)
    pi.set_pull_up_down(key1_pin, pigpio.PUD_UP)
    pi.set_pull_up_down(key2_pin, pigpio.PUD_UP)
    
    # initialize node    
    rospy.init_node('button_control_demo')
    rospy.on_shutdown(Stop)
    # RGB light node. The path of the code:/home/pi/puppy_pi/src/sensor/scripts/sensor_control.py
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    # buzzer node. The path of the code:/home/pi/puppy_pi/src/sensor/scripts/sensor_control.py
    buzzer_pub = rospy.Publisher("/sensor/buzzer", Float32, queue_size=1)
    rospy.sleep(0.5) # delay for a while
    
    button_press = False # the key is pressed
    while run_st:
        if pi.read(key1_pin) == 0: # detect the key is pressed once
            rospy.sleep(0.05) # delay and detect again
            if pi.read(key1_pin) == 0:
                if not button_press:
                    button_press = True
                    r,g,b = 0,255,0
                    set_rgb_show(r,g,b) #green
                    buzzer_pub.publish(0.5) # buzzer sounds for 0.5s
        
        if pi.read(key2_pin) == 0: # detect the key is pressed twice
            rospy.sleep(0.05) # delay and detect again
            if pi.read(key2_pin) == 0:
                if not button_press:
                    button_press = True
                    r,g,b = 0,0,255
                    set_rgb_show(r,g,b) #blue
                    buzzer_pub.publish(0.5) # buzzer sounds for 0.5s
        else:
            if button_press:
                button_press = False
            rospy.sleep(0.05)
        
    
