#!/usr/bin/env python

# subscribes to image topics from a camera module;
# it then detects the targets and computes the coordinates of their center

import sys
import rospy
import message_filters
import numpy as np
import cv2

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
    red_objects_img = cv2.bitwise_and(bgr_image, bgr_image, mask=mask)
    # dilate resulting image for filling holes
    red_objects_img = cv2.dilate(red_objects_img, None, iterations=2)
    # TODO: detect round objects

    # detect contours of red circles in result image
    result_gray = cv2.cvtColor(red_objects_img, cv2.COLOR_BGR2GRAY)
    # if a bit has a value lower than 25, set it to black; if not, set it to white
    thresh = cv2.threshold(result_gray, 30, 255, cv2.THRESH_BINARY)[1]
    # dilate the image to fill holes; implicit using 3x3 matrix
    thresh_dilated = cv2.dilate(thresh, None, iterations=2)

    (_, contours, _) = cv2.findContours(thresh_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # initialize centroid position (the method will return (0,0) if no target has been detected)

    # loop over the contours

    pixel_pos = []
    radius_vec = []
    for contour in contours:
        # compute the bounding circle for the contour, draw it on the current image
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        center_x = int(center_x)
        center_y = int(center_y)

        # if the radius is too small, target is ignored

        pixel_pos.append(PixelPos(center_x, center_y))
        radius_vec.append(radius)

        # cv2.circle(bgr_image, (center_x, center_y), int(radius), (0, 255, 0), 1)
        # cv2.putText(bgr_image, "id: " + str(target_id), (center_x, center_y),
        # cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)

    if not pixel_pos:
        pixel_pos = None

    return pixel_pos, radius_vec, bgr_image

# select a pixel window from an image


def select_pixel_window(bgr_image, pixel_coord, offset):
    # TODO treat particular cases: index out of range
    center_x = pixel_coord.u
    center_y = pixel_coord.v
    row_start = center_x - offset
    row_end = center_x + offset
    col_start = center_y - offset
    col_end = center_y + offset
    return bgr_image[col_start:col_end+1, row_start:row_end+1]


def is_pixel_in_area(area_start, area_end, pixel_coord):
    # check if a pair of pixel coordinates is in an area
    # input parameters given as PixelPos instances;
    # if pixel coord in area, then return True
    if (area_start.u <= pixel_coord.u <= area_end.u) and (area_start.v <= pixel_coord.v <= area_end.v):
        return True
    else:
        return False


class PixelPos:

    def __init__(self, u, v):
        self.u = u
        self.v = v

    def print_pixel_pos(self, which_cam):
        print "Pixel Coord(x,y) " + which_cam + ": (" + str(self.u) + ',' + str(self.v) + ")"


class TargetTracker:

    def __init__(self, pixel_pos_left, pixel_pos_right, target_id):
        self.coord_left = pixel_pos_left
        self.coord_right = pixel_pos_right
        self.id = target_id


class TargetDetector:

    def __init__(self, pixel_pos_left, pixel_pos_right, radius_left, radius_right, target_id):
        # detection on the whole image for dealing with new targets;
        # detect the current position of target in a window of pixels, instead of the whole image;
        self.coord_right = pixel_pos_right
        self.coord_left = pixel_pos_left
        self.radius_left = radius_left
        self.radius_right = radius_right
        self.radius_thresh = 7
        self.target_id = target_id
        # becomes active when a target has been detected;
        # becomes inactive when target detected is too small;
        self.status = "active"
        # window dimensions in pixels, depends on radius;
        # default: 10x10
        self.window_offset_left = 2*int(radius_left)
        self.window_offset_right = 2*int(radius_right)

    def update_coord(self, image_left, image_right):

        # input: pixel window for each camera
        pixel_pos_left, radius_left, image_left = detect_target(image_left)
        pixel_pos_right, radius_right, image_right = detect_target(image_right)

        # transform in original image coordinates
        if (pixel_pos_left is not None) and (pixel_pos_right is not None):
            new_u_left = pixel_pos_left[0].u - self.window_offset_left + self.coord_left.u
            new_v_left = pixel_pos_left[0].v - self.window_offset_left + self.coord_left.v

            new_u_right = pixel_pos_right[0].u - self.window_offset_right + self.coord_right.u
            new_v_right = pixel_pos_right[0].v - self.window_offset_right + self.coord_right.v

            # update coordinates and radius
            self.coord_left.u = new_u_left
            self.coord_left.v = new_v_left

            self.coord_right.u = new_u_right
            self.coord_right.v = new_v_right

            self.radius_left = radius_left[0]
            self.radius_right = radius_right[0]

    def define_area(self):
        area_start = PixelPos(self.coord_left.u - self.window_offset_left,
                              self.coord_left.v - self.window_offset_left)
        area_end = PixelPos(self.coord_left.u + self.window_offset_left,
                            self.coord_left.v + self.window_offset_left)
        return area_start, area_end

    def print_info(self):
        sys.stdout.write("-T_id " + str(self.target_id) + "L:(" + str(self.coord_left.u) + ',' + str(self.coord_left.v) +
               "); R:(" + str(self.coord_right.u) + ',' + str(self.coord_right.v) + ")" + "\r")

    def set_inactive_status(self):
        # check if a target should be ignored
        if (int(self.radius_left) < self.radius_thresh) and (int(self.radius_right) < self.radius_thresh):
            self.status = "inactive"
            return True
        else:
            return False

class CamModuleSub:

    def __init__(self, module_name):
        # subscribers for left, right image topics
        self.subLeft = message_filters.Subscriber("/" + module_name + "/left_camera/image_raw_left/compressed",
                                                  CompressedImage)
        self.subRight = message_filters.Subscriber("/" + module_name + "/right_camera/image_raw_right/compressed",
                                                   CompressedImage)

        # save BGR images
        self.left_img = None
        self.right_img = None

        # ensure that both images arrive at the same time
        time_sync = message_filters.TimeSynchronizer([self.subLeft, self.subRight], 10)
        time_sync.registerCallback(self.callback)

    def callback(self, image_left, image_right):
        # convert left image from string array to BGR format
        np_array = np.fromstring(image_left.data, np.uint8)
        self.left_img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        # convert right image from string array to BGR format
        np_array = np.fromstring(image_right.data, np.uint8)
        self.right_img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)


# global list of classes TargetDetector
_detector_vect = []
_sampling_time = 5  # seconds
_target_id = 0
_current_target_nr = 0
_current_state = "Detect state - whole image"
_next_state = "Detect state - whole image"


def main():
    global _detector_vect
    global _sampling_time
    global _target_id
    global _current_target_nr
    global _current_state
    global _next_state
    # initialize ROS node
    rospy.init_node("detect_target")

    #
    base_name = "camera"
    index_name = 0

    # verify which camera module is activated;
    module_name = base_name + str(index_name)

    # create module subscriber
    test = CamModuleSub(module_name)

    # set up loop rate
    loop_rate = rospy.Rate(100)

    # LOG INFO
    print "Waiting camera data..."
    # -------

    # wait until both cameras sent image message to topics
    while (test.left_img is None) and (test.right_img is None):
        pass

    # initial step:
    # initial target detection
    pixel_pos_left, radius_left, image_left = detect_target(test.left_img)
    pixel_pos_right, radius_right, image_right = detect_target(test.right_img)

    # LOG INFO
    print "Waiting target detection..."
    # -------

    # if a target has been detected
    while (pixel_pos_left is None) or (pixel_pos_right is None):
        # get new data until a target has been detected by both cameras
        pixel_pos_left, radius_left, image_left = detect_target(test.left_img)
        pixel_pos_right, radius_right, image_right = detect_target(test.right_img)

    _current_target_nr = len(pixel_pos_left)
    
    # LOG INFO
    print str(_current_target_nr) + " target(s) detected..."
    # -------

    # append tracker for each target detected in the initial step
    for index in range(len(pixel_pos_left)):
        new_detector = TargetDetector(pixel_pos_left[index],
                                      pixel_pos_right[index],
                                      radius_left[index],
                                      radius_right[index],
                                      _target_id)
        _target_id += 1
        _detector_vect.append(new_detector)
        # LOG INFO
        new_detector.print_info()
        # -------

    # set current state to pixel window since we have the targets data
    _current_state = "Detect state - pixel window"

    # LOG INFO
    print "-----------------------\nPixel window detection MODE."
    # -------

    # save current time;
    detection_sampling_time = rospy.get_rostime().secs
    loop = 0
    while not rospy.is_shutdown():

        if _current_state == "Detect state - whole image":
            #  TODO

            # get targets' positions in pixel coordinates for both cameras
            # only new targets should be added to the target vector
            pixel_pos_left, radius_left, image_left = detect_target(test.left_img)
            pixel_pos_right, radius_right, image_right = detect_target(test.right_img)

            # if target is detected on both images
            if (pixel_pos_left is not None) and (pixel_pos_right is not None):

                detected_target_nr = len(pixel_pos_left)
                # if the number of pixel coordinates is greater than current target nr,
                # then a new target has been detected (or more than 1)
                if detected_target_nr > _current_target_nr:
                    # LOG INFO
                    print "\n" + str(detected_target_nr - _current_target_nr) + " new target(s) detected."
                    # -------
                    # check if there are targets detected;
                    # _detector_vect contains past detected targets; if there are none, then all the targets will be
                    # added to the list
                    if not _detector_vect:
                        for index in range(len(pixel_pos_left)):

                                new_detector = TargetDetector(pixel_pos_left[index],
                                                              pixel_pos_right[index],
                                                              radius_left[index],
                                                              radius_right[index],
                                                              _target_id)
                                # if the detected target is too small, ignore it
                                if not new_detector.set_inactive_status():
                                    _detector_vect.append(new_detector)
                                    # new target detected; increase the number of targets and future target id
                                    _target_id += 1
                                    _current_target_nr += 1
                    # if there are >= 1 targets in target list, then determine if the target is new/past;
                    else:
                        # for each pair of pixels, check if they are within an area given by current targets;
                        # if a target is outside of all the available areas, then it's a new target
                        for index in range(len(pixel_pos_left)):
                            for detector in _detector_vect:
                                # for each detector instance (target instance), define area
                                area_start, area_end = detector.define_area()
                                if not is_pixel_in_area(area_start, area_end, pixel_pos_left[index]):
                                    # target given by index is a new target;
                                    new_detector = TargetDetector(pixel_pos_left[index],
                                                                  pixel_pos_right[index],
                                                                  radius_left[index],
                                                                  radius_right[index],
                                                                  _target_id)
                                    _detector_vect.append(new_detector)
                                    # new target detected; increase the number of targets and future target id
                                    _target_id += 1
                                    _current_target_nr += 1
                                    # LOG INFO
                                    print "\nA new target has been detected."
                                    new_detector.print_info()
                                    # -------
                                    break  # exit second for loop; check a new pair of coordinates

                else:
                    # LOG INFO
                    print "\nNo detection of a new target."
                    # -------

            _next_state = "Detect state - pixel window"
            # LOG INFO
            print "\n-----------------------\nPixel window detection MODE."
            # -------

        # after a target has been detected, in the next frame, detection will
        # occur in a region around the target coordinates for efficiency
        # detect targets in a block of pixels around the center point detected in the last frame
        elif _current_state == "Detect state - pixel window":
            if not _detector_vect:
                # LOG INFO
                if loop == 0:
                    sys.stdout.write(".\r")
                    loop += 1
                elif loop == 1:
                    sys.stdout.write("..\r")
                    loop += 1
                elif loop == 2:
                    sys.stdout.write("...\r")
                    loop = 0
                # -------
            else:
                for detector in _detector_vect:
                    left_img = select_pixel_window(test.left_img,
                                                   detector.coord_left,
                                                   detector.window_offset_left)

                    right_img = select_pixel_window(test.right_img,
                                                    detector.coord_right,
                                                    detector.window_offset_right)

                    detector.update_coord(left_img, right_img)
                    detector.set_inactive_status()
                    detector.print_info()

                    # check if current detector is labeled as inactive:
                    if detector.status == "inactive":
                        _detector_vect.remove(detector)
                        # decrement number of currently detected targets
                        _current_target_nr -= 1
                        # LOG INFO
                        print "\nTarget too small. Removed."
                        # -------

            # if a time period EGT _sampling_time has passed, change state
            current_time = rospy.get_rostime().secs
            if current_time - detection_sampling_time >= _sampling_time:
                detection_sampling_time = current_time
                _next_state = "Detect state - whole image"
                # LOG INFO
                print "\n" + str(_sampling_time) + " seconds have passed. Change state. Check for new targets."
                print "\n---------------------\nWhole image detection MODE."
                # -------
            else:
                _next_state = _current_state

        _current_state = _next_state
        loop_rate.sleep()


if __name__ == '__main__':
    main()
