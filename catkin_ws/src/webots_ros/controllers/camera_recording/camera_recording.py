#!/usr/bin/env python
from controller import Robot, Camera

TIME_STEP = 32

robot = Robot()

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)
path="../../src/change_pkg/ROI"

while robot.step(TIME_STEP) != -1:
    roi=""
    with open('{}/ROI.txt'.format(path), 'w') as f:
        f.write(roi)
