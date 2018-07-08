#!/usr/bin/env python

# subscribes to image topics from a camera module;
# it then detects the targets and computes the coordinates of their center


import rospy
import message_filters
import numpy as np
import cv2
import matplotlib.pyplot as plt

from sensor_msgs.msg import CompressedImage

class PixelPos():
    def __init__(self, u, v):
        self.u = u
        self.v = v


class CamModuleSub:

    def __init__(self, moduleName):
        # subscribers for left, right image topics
        self.subLeft  = message_filters.Subscriber("/" + moduleName + "/left_camera/image_raw_left/compressed", CompressedImage)
        self.subRight = message_filters.Subscriber("/" + moduleName + "/right_camera/image_raw_right/compressed", CompressedImage)

        # save BGR images
        self.leftImg  = None
        self.rightImg = None

        # ensure that both images arrive at the same time
        timeSync = message_filters.TimeSynchronizer([self.subLeft, self.subRight],10)
        timeSync.registerCallback(self.callback)

    def callback(self, image_left, image_right):
        # convert left image from string array to BGR format
        np_array     = np.fromstring(image_left.data, np.uint8)
        self.leftImg = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        # convert right image from string array to BGR format
        np_array     = np.fromstring(image_left.data, np.uint8)
        self.leftImg = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

    def detectTarget(self, bgrImage):
        # Color detection
        # convert BGR image to HSV format
        hsvImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2HSV)
        # set lower and upper limits for red color, HSV format
        lower = [0, 100, 100]
        upper = [10, 255, 255]
        lowerRange = np.array(lower, dtype="uint8")
        upperRange = np.array(upper, dtype="uint8")
        # create mask
        mask = cv2.inRange(hsvImage, lowerRange, upperRange)
        # bitwise and between mask and video capture will select the objects with red color
        redObjects = cv2.bitwise_and(bgrImage, bgrImage, mask = mask)
        # dilate resulting image for filling holes
        redObjects = cv2.dilate(redObjects, None, iterations=2)
        # TODO: detect round objects

        # detect contours of red circles in result image
        resultGray = cv2.cvtColor(redObjects, cv2.COLOR_BGR2GRAY)
        # if a bit has a value lower than 25, set it to black; if not, set it to white
        thresh = cv2.threshold(resultGray, 30, 255, cv2.THRESH_BINARY)[1]
        # dilate the image to fill holes; implicit using 3x3 matrix
        threshDilated = cv2.dilate(thresh, None, iterations=2)
        pixelPos = None
        (_, contours, _) = cv2.findContours(threshDilated.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # initalize centroid position (the method will return (0,0) if no target has been detected)
        (center_x, center_y) = (0,0)
        # loop over the contours
        pixelPos = []
        for cntr in contours:
            # compute the bounding circle for the contour, draw it on the current image
            (center_x,center_y), radius = cv2.minEnclosingCircle(cntr)
            center_x = int(center_x)
            center_y = int(center_y)
            pixelPos.append(PixelPos(center_x, center_y))
            cv2.circle(bgrImage, (center_x, center_y), int(radius), (0, 255, 0), 1)

        if pixelPos == []:
            pixelPos = None

        plt.imshow(cv2.cvtColor(bgrImage, cv2.COLOR_BGR2RGB))
        plt.pause(0.05)
        plt.show()
        return pixelPos

def main():

    # initialize ROS node
    rospy.init_node("detect_target")

    #
    baseName  = "camera"
    indexName = 0

    # verify which camera module is activated;
    moduleName = baseName + str(indexName)

    # create module subscriber
    test = CamModuleSub(moduleName)

    # set up loop rate
    loop_rate = rospy.Rate(1000)
    plt.ion()
    # wait until both cameras sent image message to topics
    while (test.leftImg is None) and (test.rightImg is None):
        pass

    while not rospy.is_shutdown():
        print test.detectTarget(test.leftImg)
        loop_rate.sleep()

if __name__ == '__main__':
    main()
