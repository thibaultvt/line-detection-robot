import cv2 as cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

import laneDetectionModule as ld
import motorModule as mm



#   PiCam initialization
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

#   Give the camera some time
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #   Converts frame to numpy array
    frame = frame.array

    #   Flip horizontally and vertically
    frame = cv2.flip(frame, -1)

    #   Use laneDetectionModule to find the steeringAngle
    thresholdFrame = ld.getThresholdedFrame(frame)
    croppedFrame = ld.regionOfInterest(thresholdFrame, 75)
    contours = ld.getContours(croppedFrame)
    hull = ld.getHullArray(contours)
    extremesList = ld.findExtremes(hull, frame)

    steeringAngle = ld.getSteeringAngle(extremesList, frame)

    #   Use motorModule to control the direction of the robot
    mm.steeringWheel(steeringAngle)

    # ld.contourDraw(hull, frame, color=(255, 0, 0))
    # ld.drawExtremeRectangle(extremesList, frame)
    # ld.drawSteeringPoint(steeringAngle, frame)
    rawCapture.truncate(0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
