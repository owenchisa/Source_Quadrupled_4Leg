import os
import time

# detect whether the camera connection works every 1s
# if the camera cinnection cuts off, restart the camera service after next plugging in.

while True:
    try:
        rostopic_result = os.popen('rostopic list').read()
        camera_result = os.popen('rosservice info /usb_cam/start_capture').read()
        video_result = os.popen('ls /dev/video0_usb_cam').read()
        #print(rostopic_result, camera_result, video_result)
        if 'rosout' in rostopic_result and ('usb_cam' not in rostopic_result or 'Type' not in camera_result) and '/dev/video0_usb_cam' in video_result:
            os.system('sudo udevadm trigger')
            os.system('sudo systemctl restart start_camera.service')
            time.sleep(4)
            print('restart camera')
    except BaseException as e:
        print('error', e)
    time.sleep(1)
