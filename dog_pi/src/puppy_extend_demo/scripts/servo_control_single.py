#!/usr/bin/env python3
# coding=utf8
import sys
import time
import signal
HomePath = '/home/pi'
sys.path.append(HomePath + '/PuppyPi_PC_Software')
from PWMServoControl import *

# control single servo

print('''
**********************************************************
*****************function:control single servo routine***********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again.
----------------------------------------------------------
''')

# close detection function
run_st = True
def Stop(signum, frame):
    global run_st
    run_st = False
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    servo = PWMServo()
    
    while run_st:
        servo.setPulse(1,1000,2000) # drive NO.1 servo
        time.sleep(2) # delay
        servo.setPulse(1,2000,2000) # drive NO.1 servo
        time.sleep(2) # delay
    
    servo.setPulse(1,1500,2000)
    
