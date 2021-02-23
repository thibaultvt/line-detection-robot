import cv2 as cv2
import laneDetectionModule as ld

#   Reads the image you want to test the Lane Detection module on
imagePath = './resources/line.png'
frame = cv2.imread(imagePath)

thresholdFrame = ld.getThresholdedFrame(frame)
croppedFrame = ld.regionOfInterest(thresholdFrame, 75)
contours = ld.getContours(croppedFrame)

hull = ld.getHullArray(contours)
extremesList = ld.findExtremes(hull, frame)

steeringAngle = ld.getSteeringAngle(extremesList, frame)


ld.contourDraw(hull, frame, color=(255, 0, 0))
ld.drawExtremeRectangle(extremesList, frame)
ld.drawSteeringPoint(steeringAngle, frame)

cv2.imshow("Hull on the frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
