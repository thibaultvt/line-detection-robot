import cv2 as cv2
import numpy as np



#   Filter out a color based on the HSV pixel values, the values
#   in the arrays need to be adjusted when detecting another color.

#   Returns a threshold image with pixel values of 255 if it's within
#   boundaries otherwise it's set to 0.


def getThresholdedFrame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #   Set minimum and maximum HSV values to display, OpenCV uses a Hue scale between 0-179!
    lower_white = np.array([101, 66, 151])
    upper_white = np.array([136, 255, 255])
    thresholdedFrame = cv2.inRange(hsv, lower_white, upper_white)

    return thresholdedFrame


#   Crop a frame to n pixels down from the middle
#   Returns a frame


def regionOfInterest(frame, nPixels):
    height, width = frame.shape

    #   Creates an all black frame with the same size as the given frame
    mask = np.zeros_like(frame)

    #   Specify four points to "draw" a rectangle over the nPixels top of the bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, (height * 1 / 2) + nPixels),
        (0, (height * 1 / 2) + nPixels),
    ]], np.int32)

    #   Fill the created polygon(rectangle) with white
    cv2.fillPoly(mask, polygon, 255)

    #   Returns only the white part of the mask on the frame
    croppedFrame = cv2.bitwise_and(frame, mask)

    return croppedFrame


#   The parameter croppedFrame should come from the function regionOfInterest
#   Returns the contours coordinates


def getContours(croppedFrame):
    #   findContours finds contours in a binary image
    contours, _ = cv2.findContours(
        croppedFrame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours


#   Best explanation of convexHull => https://learnopencv.com/convex-hull-using-opencv-in-python-and-c/
#   Returns an array of hull coordinates


def getHullArray(contours):
    #   A Convex object is one with no interior angles greater than 180 degrees.
    #   Hull means the exterior or the shape of the object.
    hull = []
    #   Calculate points for each contour
    for i in range(len(contours)):
        #   Creating convex hull object for each contour
        hull.append(cv2.convexHull(contours[i], False))

    return hull


#   Draws the a contour using the hull points


def contourDraw(hull, frame, color=(255, 0, 0)):
    for i in range(len(hull)):
        cv2.drawContours(frame, hull, i, color, 1, 8)


#   Returns a list with the four most extreme points, format => [Xleft, Xright, Ytop, Ybottom]


def findExtremes(hull, frame):
    hullLength = len(hull)
    Xlist = []
    Ylist = []

    extremesList = [0, 0, 0, 0]

    for counter in range(hullLength-0):
        #   Create a separate list for X and Y coordinates
        for i in hull[counter]:
            Xlist.append(i[0][0])
            Ylist.append(i[0][1])

    #   We use this to not get the max of an empty list
    if Xlist:
        extremesList[0] = min(Xlist)  # Xleft
        extremesList[1] = max(Xlist)  # Xright
        extremesList[2] = min(Ylist)  # Ytop
        extremesList[3] = max(Ylist)  # Ybottom

    return extremesList


#   Draw a rectangle using the extreme points, first you need to find the extremes with the function 'findExtremes()'


def drawExtremeRectangle(extremesList, frame):
    Xleft, Ytop, Xright, Ybottom = extremesList[0], extremesList[2], extremesList[1], extremesList[3]
    #   Don't draw the rectangle if Ybottom = 0, because it can never be 0 unless we set it to 0
    if Ybottom != 0:
        cv2.rectangle(frame, (Xleft, Ytop), (Xright, Ybottom), (0, 255, 0), 3)


#   Returns the a steering angle between 0 and 180


def getSteeringAngle(extremesList, frame):
    Xleft, Xright, Ybottom = extremesList[0], extremesList[1], extremesList[3]
    _, width, _ = frame.shape
    steeringAngle = 90  # Default value is 90 => "Go straight"

    #   Don't calculate new steeringAngle if Ybottom = 0, because it can never be 0 unless we set it to 0
    if Ybottom != 0:
        centerCoord = round((Xright - Xleft) * 1/2)
        steeringCoord = Xleft + centerCoord
        #   Convert center of line coordinate to an angle between 0 and 180
        steeringAngle = round((steeringCoord / width) * 180)

    return steeringAngle


#   Visualizes the steering angle as a line from the bottom to the vertical half


def drawSteeringPoint(steeringAngle, frame):
    _, width, _ = frame.shape
    steeringCoord = round((width * steeringAngle) / 180)
    height, width, _ = frame.shape
    cv2.line(frame, (steeringCoord, round(height * 1/2)),
             (round(width * 1/2), height), (78, 89, 237), 5)
