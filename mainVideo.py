import cv2 as cv2
import laneDetectionModule as ld
import time

videoPath = './resources/lineVideo.h264' 
cap = cap = cv2.VideoCapture(videoPath)


while(True):
    ret, frame = cap.read()
    #   Flip horizontally and vertically
    frame = cv2.flip(frame, -1)
    
    thresholdFrame = ld.getThresholdedFrame(frame)
    croppedFrame = ld.regionOfInterest(thresholdFrame, 75)
    contours = ld.getContours(croppedFrame)
    hull = ld.getHullArray(contours)
    extremesList = ld.findExtremes(hull, frame)

    steeringAngle = ld.getSteeringAngle(extremesList, frame)
    print(steeringAngle)


    ld.contourDraw(hull, frame, color=(255, 0, 0))
    ld.drawExtremeRectangle(extremesList, frame)
    ld.drawSteeringPoint(steeringAngle, frame)

    #   Display the edited frame
    cv2.imshow("Hull on the frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    #   Only need this if the video is too fast
    time.sleep(0.025)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
