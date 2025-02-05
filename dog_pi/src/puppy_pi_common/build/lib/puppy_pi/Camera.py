#!/usr/bin/env python3
# encoding:utf-8
import sys
import cv2
import time
import threading
import numpy as np

#This is a package library that uses Opencv to get the screen of the usb camera

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.opened = False
        
        # obtain the image with child thread
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self): # open
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30) # frame rate
            #self.cap.set(cv2.CAP_PROP_SATURATION, 40) # Saturation
            self.opened = True
        except Exception as e:
            print('打开摄像头失败:', e)

    def camera_close(self): # close
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print('关闭摄像头失败:', e)

    def camera_task(self): # acquire the thread of the camera screen
        while True:
            try:
                if self.opened and self.cap.isOpened(): # judge whether the camera opens
                    ret, frame_tmp = self.cap.read() # acquire the image
                    if ret:
                        self.frame = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST) # scale                     
                    else:
                        # If you fail to get the screen, try restarting the camera
                        self.frame = None
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap              
                else:
                    time.sleep(0.01)
            except Exception as e:
                print('获取摄像头画面出错:', e)
                time.sleep(0.01)

if __name__ == '__main__':
    # use the routine
    my_camera = Camera()
    my_camera.camera_open()
    print('摄像头原始画面，未做畸变校正')
    while True:
        img = my_camera.frame
        if img is not None:
            cv2.imshow('img', img)
            key = cv2.waitKey(1)
            if key == 27:
                break  
    my_camera.camera_close()
    cv2.destroyAllWindows()
