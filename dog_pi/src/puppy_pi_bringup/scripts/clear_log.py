#!/usr/bin/python3 
import os
import sys
import time
import signal

# detect the log every 30 minutes, if it is greater than 10m, then clear all the logs.
# large log will results in hugh CPU usage and running exception
# this function and then the system will restart：sudo systemctl disable clear_log.service

running = True
def handler(signum, frame):
    global running

    running = False
    print('exit')
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

ros_log_path = '/home/pi/.ros/log'

log_size = 0
while running:
    try:
        for root, dirs, files in os.walk(ros_log_path):
            for f in files:
                log_size += os.path.getsize(os.path.join(root, f))
        log_size /= float(1024*1024)
        #print('the current size of the logs:{}m'.format(log_size))
        if log_size > 5:
            os.system('sudo rm -rf {}/*'.format(ros_log_path))
        time.sleep(30*60)
    except BaseException as e:
        print(e)
