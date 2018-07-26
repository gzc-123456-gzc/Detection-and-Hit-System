#!/usr/bin/env python

# subscribes to image topics from a camera module;
# it then detects the targets and computes the coordinates of their center


import rospy
import message_filters
import numpy as np
import cv2
import matplotlib.pyplot as plt

from sensor_msgs.msg import CompressedImage


def detect_target(bgr_image):
    # Color detection
    # convert BGR image to HSV format
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    # set lower and upper limits for red color, HSV format
    lower = [0, 100, 100]
    upper = [10, 255, 255]
    lower_range = np.array(lower, dtype="uint8")
    upper_range = np.array(upper, dtype="uint8")
    # create mask
    mask = cv2.inRange(hsv_image, lower_range, upper_range)
    # bitwise and between mask and video capture will select the objects with red color
    red_objects_img = cv2.bitwise_and(bgr_image, bgr_image, mask = mask)
    # dilate resulting image for filling holes
    red_objects_img = cv2.dilate(red_objects_img, None, iterations=2)
    # TODO: detect round objects

    # detect contours of red circles in result image
    result_gray = cv2.cvtColor(red_objects_img, cv2.COLOR_BGR2GRAY)
    # if a bit has a value lower than 25, set it to black; if not, set it to white
    thresh = cv2.threshold(result_gray, 30, 255, cv2.THRESH_BINARY)[1]
    # dilate the image to fill holes; implicit using 3x3 matrix
    thresh_dilated = cv2.dilate(thresh, None, iterations=2)
    pixelPos = None
    (_, contours, _) = cv2.findContours(thresh_dilated.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # initalize centroid position (the method will return (0,0) if no target has been detected)
    (center_x,center_y) = (0,0)
    # loop over the contours

    pixelPos  = []
    radiusVec = []
    for cntr in contours:
        # compute the bounding circle for the contour, draw it on the current image
        (center_x,center_y), radius = cv2.minEnclosingCircle(cntr)
        center_x = int(center_x)
        center_y = int(center_y)

        # if the radius is too small, target is ignored

        pixelPos.append(PixelPos(center_x, center_y))
        radiusVec.append(radius)
        #cv2.circle(bgr_image, (center_x, center_y), int(radius), (0, 255, 0), 1)
        #cv2.putText(bgr_image, "id: " + str(target_id), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)

    if pixelPos == []:
        pixelPos = None

    return pixelPos, radiusVec, bgr_image

# select a pixel window from an image
def select_pixel_window(bgr_image, pixelCoord, offset):
    # TODO treat particular cases: index out of range
    center_x = pixelCoord.u
    center_y = pixelCoord.v
    row_start = center_x - offset
    row_end   = center_x + offset
    col_start = center_y - offset
    col_end   = center_y + offset
    return bgr_image[col_start:col_end+1, row_start:row_end+1]
    # TODO: this pixel window will be used for target detection and will be updated every iteration;
    #       ensure that pixel coordinates of target are translated into original pixel coord

class PixelPos:
    def __init__(self, u, v):
        self.u = u
        self.v = v

    def print_pixel_pos(self, which_cam):
        print "Pixel Coord(x,y) " + which_cam + ": (" + str(self.u) + ',' + str(self.v) + ")"

class TargetTracker:
    def __init__(self, pixelPosL, pixelPosR, id):
        self.coord_left  = pixelPosL
        self.coord_right = pixelPosR
        self.id = id

class TargetDetector:
    def __init__(self, pixelPosL, pixelPosR, radiusL, radiusR, id):
        # detection on the whole image for dealing with new targets;
        # detect the current position of target in a window of pixels, instead of the whole image;
        self.coord_right   = pixelPosR
        self.coord_left    = pixelPosL
        self.radius_left   = radiusL
        self.radius_right  = radiusR
        self.radius_thresh = 5
        self.id            = id
        self.status        = "active" # becomes active when a target has been detected;
                                        # becomes inactive when target detected is too small;
        # window dimensions in pixels, depends on radius;
        # default: 10x10
        self.window_offset_left  = 2*int(radiusL)
        self.window_offset_right = 2*int(radiusR)

    def update_coord(self, image_left, image_right):

        # input: pixel window for each camera
        pixelPos_L, radius_L, image_L = detect_target(image_left)
        pixelPos_R, radius_R, image_R = detect_target(image_right)

        # transform in original image coordinates
        if (pixelPos_L is not None) and (pixelPos_R is not None):
            new_u_L = pixelPos_L[0].u - self.window_offset_left + self.coord_left.u
            new_v_L = pixelPos_L[0].v - self.window_offset_left + self.coord_left.v

            new_u_R = pixelPos_R[0].u - self.window_offset_right + self.coord_right.u
            new_v_R = pixelPos_R[0].v - self.window_offset_right + self.coord_right.v

            # update coordinates and radius
            self.coord_left.u = new_u_L
            self.coord_left.v = new_v_L

            self.coord_right.u = new_u_R
            self.coord_right.v = new_v_R

            self.radius_left  = radius_L[0]
            self.radius_right = radius_R[0]

            # check if a target should be ignored
            if radius_L[0] < self.radius_thresh or radius_R[0] < self.radius_thresh:
                self.status = "inactive"


class CamModuleSub:

    def __init__(self, module_name):
        # subscribers for left, right image topics
        self.subLeft  = message_filters.Subscriber("/" + module_name + "/left_camera/image_raw_left/compressed", CompressedImage)
        self.subRight = message_filters.Subscriber("/" + module_name + "/right_camera/image_raw_right/compressed", CompressedImage)

        # save BGR images
        self.left_img  = None
        self.right_img = None

        # ensure that both images arrive at the same time
        timeSync = message_filters.TimeSynchronizer([self.subLeft, self.subRight],10)
        timeSync.registerCallback(self.callback)

    def callback(self, image_left, image_right):
        # convert left image from string array to BGR format
        np_array      = np.fromstring(image_left.data, np.uint8)
        self.left_img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        # convert right image from string array to BGR format
        np_array       = np.fromstring(image_right.data, np.uint8)
        self.right_img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)



def main():
    # global list of classes TargetDetector
    _detectorVect  = []
    _sampling_time = 5 # seconds
    _target_id     = 0
    #############################
    current_state = "Detect state - whole image"
    next_state    = "Detect state - whole image"
    # initialize ROS node
    rospy.init_node("detect_target")

    #
    base_name  = "camera"
    index_name = 0

    # verify which camera module is activated;
    module_name = base_name + str(index_name)

    # create module subscriber
    test = CamModuleSub(module_name)

    # set up loop rate
    loop_rate = rospy.Rate(100)\
    #############################

    ######### LOG INFO
    print "Waiting camera data..."
    #########

    # wait until both cameras sent image message to topics
    while (test.left_img is None) and (test.right_img is None):
        pass

    # initial step:
    # initial target detection
    pixelPos_L, radius_L, image_L = detect_target(test.left_img)
    pixelPos_R, radius_R, image_R = detect_target(test.right_img)

    ######### LOG INFO
    print "Waiting target detection..."
    #########

    # if a target has been detected
    while (pixelPos_L is None) or (pixelPos_R is None):
        # get new data until a target has been detected by both cameras
        pixelPos_L, radius_L, image_L = detect_target(test.left_img)
        pixelPos_R, radius_R, image_R = detect_target(test.right_img)

    ######### LOG INFO
    print str(len(pixelPos_L)) + " target(s) detected..."
    #########

    # append tracker for each target detected in the initial step
    for index in range(len(pixelPos_L)):
        newDetector = TargetDetector(pixelPos_L[index], pixelPos_R[index], radius_L[index], radius_R[index], _target_id)
        _target_id += 1
        _detectorVect.append(newDetector)

    # set current state to pixel window since we have the targets data
    current_state = "Detect state - pixel window"
    # save current time;
    detection_sampling_time = rospy.get_rostime().secs

    while not rospy.is_shutdown():

        # wait first detection then change state
        # TODO
        if current_state == "Detect state - whole image":
            ######### LOG INFO
            print "Whole image detection MODE."
            #########


        # after a target has been detected, in the next frame, detection will
        # occur in a region around the target coordinates for efficiency
        elif current_state == "Detect state - pixel window":
            ######### LOG INFO
            print "Pixel window detection MODE."
            #########
            # detect targets in a block of pixels around the center point detected in the last frame
            for detector in _detectorVect:
                left_img  = select_pixel_window(
                    test.left_img,
                    detector.coord_left,
                    detector.window_offset_left
                )

                right_img = select_pixel_window(
                    test.right_img,
                    detector.coord_right,
                    detector.window_offset_right
                )

                detector.update_coord(left_img, right_img)

                # check if current detector is labeled as inactive:
                if detector.status == "inactive":
                    _detectorVect.remove(detector)

            
            # if a time period EGT _sampling_time has passed, change state
            current_time = rospy.get_rostime().secs
            if current_time - detection_sampling_time >= _sampling_time:
                detection_sampling_time = current_time
                next_state = "Detect state - whole image"
                ######### LOG INFO
                print str(_sampling_time) + " seconds have passed. Change state. Check for new targets."
                #########
            else:
                next_state = current_state


        current_state = next_state
        loop_rate.sleep()

if __name__ == '__main__':
    main()
