#!/usr/bin/python3
# coding=utf8
# Author:hiwonder

import os, sys, time
import numpy as np


HomePath = '/home/pi'
sys.path.append(HomePath + '/PuppyPi_PC_Software')
from ServoCmd import setServoPulse
from HiwonderPuppy import HiwonderPuppy, PWMServoParams


puppy = HiwonderPuppy(setServoPulse = setServoPulse, servoParams = PWMServoParams(), dof = '8')
                            # FR    FL    BR     BL
foot_locations = np.array([ [ -1.,  -1.,  -1.,   -1.], # X
                            [ 0.,    0.,   0.,    0.], # Y
                            [-10,   -10,  -10,   -10,] # Z
                            ])
# corresponds to the coordinate of four legs in cm


foot_locations = foot_locations/100 # covert the unit into meter

joint_angles = puppy.fourLegsRelativeCoordControl(foot_locations)
# input the coordinate and the angle of each servo will be calculated through inverse kinematics
print(joint_angles*57.3)

puppy.servo_force_run()
# force the servo to rotate. Sometimes the servo will not rotate without the execution of this code

puppy.sendServoAngle(joint_angles, time = 500)
#send the servo angle to the servo


while True:
    time.sleep(0.001)