#!/usr/bin/env python3
import os
import sys
import time
import signal
import Sonar as Sonar

print('''
**********************************************************
*******************Function:ultrasonic control routine**********************
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


# close detection function
run_st = True
def Stop(signum, frame):
    global run_st
    run_st = False
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    s = Sonar.Sonar()
    s.setRGBMode(0) # 0:colored light module,1:breathing light mode
    s.setRGB(1, (0, 0, 0)) # close RGB light
    s.setRGB(0, (0, 0, 0))
    while run_st:
        time.sleep(0.1)
        distance = s.getDistance() # acquire the detection distance
        print('distance: {}(mm)'.format(distance))
        if distance <= 300: # distance is smaller than 300mm
            s.setRGB(1, (255, 0, 0)) # set the light as red
            s.setRGB(0, (255, 0, 0))
            
        elif 300 < distance < 500: 
            s.setRGB(1, (0, 255, 0)) # set the light as green
            s.setRGB(0, (0, 255, 0))
            
        else:
            s.setRGB(1, (0, 0, 255)) # set the light as blue
            s.setRGB(0, (0, 0, 255))
            
    s.setRGB(1, (0, 0, 0))
    s.setRGB(0, (0, 0, 0))
    
