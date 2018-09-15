#!/usr/bin/env python

# subscribes to image topics from a camera module;
# it then detects the targets and computes the coordinates of their center

import tf
import cv2
import time
import rospy
import numpy as np
import message_filters
import matplotlib.pyplot as plt

from math import cos
from math import sin
from math import atan2
from kalman_filter import CustomKalmanFilter

from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage
from gazebo_msgs.srv import GetModelState
from simulator_gazebo.srv import GetCamModuleInfo
from detect_target.msg import TargetState


def detect_target(bgr_image, nr_targets):
    """
    Detects a number of targets equal to nr_targets.

    :param bgr_image: input BGR camera image from Gazebo sim.
    :param nr_targets: number of targets to be detected
    :return: PixelPos instance containing the corresponding pixel coordinates of the target and the radius of the target
    (in pixels).
    """

    if bgr_image is None:
        return None, None, None
    else:
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
        cv2.imwrite("/home/marius/Desktop/masca_hsv.png", mask)
        # bitwise and between mask and video capture will select the objects with red color
        red_objects_img = cv2.bitwise_and(bgr_image, bgr_image, mask=mask)
        cv2.imwrite("/home/marius/Desktop/obiecte_rosii.png", red_objects_img)
        # dilate resulting image for filling holes
        red_objects_img = cv2.dilate(red_objects_img, None, iterations=2)
        # TODO: detect round objects

        # detect contours of red circles in result image
        result_gray = cv2.cvtColor(red_objects_img, cv2.COLOR_BGR2GRAY)
        # if a bit has a value lower than 25, set it to black; if not, set it to white
        thresh = cv2.threshold(result_gray, 30, 255, cv2.THRESH_BINARY)[1]
        # dilate the image to fill holes; implicit using 3x3 matrix
        thresh_dilated = cv2.dilate(thresh, None, iterations=2)
        cv2.imwrite("/home/marius/Desktop/thresh_image.png", thresh_dilated)

        (_, contours, _) = cv2.findContours(thresh_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # initialize centroid position (the method will return (0,0) if no target has been detected)

        pixel_pos = []
        radius_vec = []
        if len(contours) != 0:
            # if the nr of targets to be detected is one, return that data
            if nr_targets == 1:
                (center_x, center_y), radius = cv2.minEnclosingCircle(contours[0])
                center_x = int(center_x)
                center_y = int(center_y)
                # draw center point
                cv2.circle(bgr_image, (center_x, center_y), int(radius), color=(0, 255, 0), thickness=2)
                cv2.imwrite("/home/marius/Desktop/detect_tinta.png", bgr_image)
                # cv2.imshow("Target detection ", bgr_image)

                # cv2.waitKey(10)
                return PixelPos(center_x, center_y), radius

            else:
                # if more than one target are required for detection;
                # append data in a vector
                for contour in contours:
                    # compute the bounding circle for the contour, draw it on the current image
                    (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
                    center_x = int(center_x)
                    center_y = int(center_y)

                    pixel_pos.append(PixelPos(center_x, center_y))
                    radius_vec.append(radius)

                return pixel_pos, radius_vec
        else:
            return None, None


def convert_quaternion_euler(quaternion):
    """
    Converts a quaternion to its Euler angles equivalent.

    :param quaternion:
    :return: y,p,r in radians
    """
    # convert orientation from Quaternion message type to euler angles
    tuple_quat = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    euler_tuple = tf.transformations.euler_from_quaternion(tuple_quat)
    return euler_tuple[2], euler_tuple[1], euler_tuple[0]


def cam_module_pos(cam_info):
    """
    This function determines the real world coordinates of a camera module. Cam module contains two cameras:
    left and right. Cam_info contains the coordinates of the left camera. The right camera has an offset on Y-axis,
    so its coordinates can also be determined.
    Furthermore, if the module has been rotated, the initial coordinates will change according to a rotation matrix
    applied to the initial coordinate vector for each camera. Cam_info contains the orientation of the module expressed
    as a quaternion. Convert the quaternion to Euler angles and determine the rotation matrix around Z-axis, then apply
    this matrix to the right_cam pos vector to obtain real world coordinates.

    :param cam_info: camera module info (position, orientation, camera characteristics)
    :return: two objects of class type CamPosWorldCoord containing real world camera coordinates
    """
    # real world coordinates of cameras;
    # a camera module has two cameras: left and right;
    # the coordinates for left camera and the distance between cameras are known;
    # using quaternion data, we can determine the real world position of the right camera

    # calculate right camera position
    # the difference between L and R cameras' coordinates is an offset on Y-axis, alongside a rotation around Z-axis
    # convert quaternion data into Euler angles

    left_cam = CamPosWorldCoord(cam_info.module_pose.position)
    y_offset = cam_info.right_pose.position.y

    yaw, pitch, _ = convert_quaternion_euler(cam_info.module_pose.orientation)  # working OK - VERIFIED
    # determine right cam position (just yaw angle needed)
    right_cam_pos = Pose().position
    right_cam_pos.z = left_cam.z
    right_cam_pos.x = cam_info.module_pose.position.x - y_offset*sin(yaw)
    right_cam_pos.y = cam_info.module_pose.position.y + y_offset*cos(yaw)
    right_cam = CamPosWorldCoord(right_cam_pos)

    # determine halfway point coordinates
    halfway_offset = y_offset/2
    halfway_pos = Pose().position
    halfway_pos.z = left_cam.z
    halfway_pos.x = left_cam.x - halfway_offset*sin(yaw)
    halfway_pos.y = left_cam.y + halfway_offset*cos(yaw)
    halfway_coord = CamPosWorldCoord(halfway_pos)
    return left_cam, right_cam, halfway_coord, yaw, pitch


class CamPosWorldCoord:

    def __init__(self, position=Pose().position):
        """
        Init 3D cordinates.
        :param position: 3D coordinates
        """
        self.x = position.x
        self.y = position.y
        self.z = position.z

    def print_coord(self, cam_type):
        """
        Prints position data.
        :param cam_type: string used at the beginning of the print
        :return:
        """
        print str(cam_type) + " Coordinates(x,y,z): " \
              + "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")."

    def return_pos_array(self):
        """
        Determines position vector as np array.
        :return: np array position
        """
        return np.array([self.x, self.y, self.z])


class CamModuleSub:

    def __init__(self, module_name):
        """
        Creates subscribers for camera module images.
        :param module_name: string used to identify the topic to subscribe to
        """
        # subscribers for left, right image topics
        self.subLeft = message_filters.Subscriber("/" + module_name + "/left_camera/image_raw_left/compressed",
                                                  CompressedImage)
        self.subRight = message_filters.Subscriber("/" + module_name + "/right_camera/image_raw_right/compressed",
                                                   CompressedImage)
        self.left_img = None
        self.right_img = None

        # ensure that both images arrive at the same time
        time_sync = message_filters.TimeSynchronizer([self.subLeft, self.subRight], 10)
        time_sync.registerCallback(self.callback)

    def callback(self, image_left, image_right):
        """
        Called when both topics get new image data.

        :param image_left: data from first topic
        :param image_right: data from second topic
        """
        # convert left image from string array to BGR format
        np_array = np.fromstring(image_left.data, np.uint8)
        self.left_img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        # convert right image from string array to BGR format
        np_array = np.fromstring(image_right.data, np.uint8)
        self.right_img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)


class PixelPos:
    """
    Class used for saving pixel coordinate pairs.
    """
    def __init__(self, u, v):
        self.u = u
        self.v = v

    def print_pixel_pos(self, which_cam):
        """
        Print pixel coordinates.
        :param which_cam: string to identify the camera.
        :return:
        """
        print "Pixel Coord(x,y) " + which_cam + ": (" + str(self.u) + ',' + str(self.v) + ")"

    def is_pixel_in_area(self, area_start, area_end):
        # check if a pair of pixel coordinates is in an area
        # input parameters given as PixelPos instances;
        # if pixel coord in area, then return True
        if (area_start.u <= self.u <= area_end.u) and (area_start.v <= self.v <= area_end.v):
            return True
        else:
            return False


_nr_targets = 1


def main():
    global _nr_targets
    # initialize ROS node
    rospy.init_node("detect_target")
    # create publisher
    state_publisher = rospy.Publisher("detect_target/target_state", TargetState, queue_size=1)
    #
    base_name = "camera"
    index_name = 0

    # verify which camera module is activated;
    module_name = base_name + str(index_name)

    # create module subscriber
    cam_sub = CamModuleSub(module_name)

    # LOG INFO
    print "Waiting camera data..."
    # -------
    rospy.wait_for_service("simulator_gazebo/get_cam_module_info")
    rospy.wait_for_service("gazebo/get_model_state")
    # create ros service proxy for get cam module info service
    cam_info_srv = rospy.ServiceProxy("simulator_gazebo/get_cam_module_info", GetCamModuleInfo)
    # create ros service proxy for get_model_state service
    get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
    # save camera data required for calculating real world coordinates
    try:
        cam_srv_response = cam_info_srv()
    except rospy.service.ServiceException as e:
        print e
    # get camera info (there's only one camera module, so select the first and only element in the array)
    cam_info = cam_srv_response.module_info[0]
    # LOG INFO
    print "Camera info:\n"
    print cam_info
    # -------
    # determine real world coordinates of camera module
    # determine coordinates of the halfway point between the two cameras
    left_cam, right_cam, halfway_pos, yaw, pitch = cam_module_pos(cam_info)
    # make sure pitch angle has the right sign
    pitch = -pitch
    left_cam.print_coord("Left")
    right_cam.print_coord("Right")
    halfway_pos.print_coord("Halfway")

    # wait until both cameras sent image message to topics
    while (cam_sub.left_img is None) or (cam_sub.right_img is None):
        pass
    # LOG INFO
    print "Data acquired."
    # -------

    # Pose instance used to store approx coordinates
    new_pose = Pose()
    error_pose = Pose()
    filter_error_pose = Pose()
    filter_pos = Pose()
    # list storing coordinates
    measure_error_list = list()
    # list storing filtered coordinates
    filter_error_list = list()
    # list storing real state
    real_state_list = list()
    # list storing measurement state
    measure_state_list = list()
    # list storing filtered state
    filter_state_list = list()

    # set up loop rate
    rate_value = 20
    loop_rate = rospy.Rate(rate_value)

    # Kalman Filter init - create parameters
    state_dim = 6  # nr. of states
    measure_dim = 3  # nr. of outputs

    n = state_dim
    m = measure_dim
    init_state = np.zeros(n)
    transition_matrix = np.eye(state_dim) + np.diag(1/float(rate_value)*np.ones(measure_dim), k=measure_dim)
    cov_matrix = np.eye(n)
    measure_state_matrix = np.concatenate((np.identity(m), np.zeros((m, m))), axis=1)
    # measure_noise_cov = np.identity(m)
    measure_noise_cov = np.eye(m)
    # initialize Kalman filter
    kf = CustomKalmanFilter(init_state, transition_matrix, cov_matrix, measure_state_matrix, measure_noise_cov)
    # target_state
    target_state = TargetState()
    # plot
    plot_var = "Erro"

    while not rospy.is_shutdown():

        # start loop time
        start_time = time.time()
        try:

            pos_left, radius_left = detect_target(cam_sub.left_img, _nr_targets)
            pos_right, radius_right = detect_target(cam_sub.right_img, _nr_targets)

            if _nr_targets == 1:
                # if the target has been detected by both cameras
                both_cam_detection = (pos_left is not None) and (pos_right is not None)
                if both_cam_detection:
                    # convert pixel position to real world coordinates
                    # (u,v) -------> (x,y,z)
                    # TODO logging file
                    # C is a point on the line between the two cameras and T is target coordinate; C is not constant
                    # need to determine the distance CT
                    dist_cam_prj_to_target = cam_info.focal_length*cam_info.dist_cameras / abs(pos_left.u-pos_right.u)

                    # determine angle between optical axis and CT (in radians) - for LEFT cam
                    angle_alpha_left = atan2(cam_info.height/2 - pos_left.v, cam_info.focal_length)
                    # determine angle between optical axis and width angle - for LEFT CAM
                    angle_beta_left = atan2(cam_info.width/2 - pos_left.u, cam_info.focal_length)

                    # determine angle between optical axis and CT (in radians) - for RIGHT cam
                    angle_alpha_right = atan2(cam_info.height/2 - pos_right.v, cam_info.focal_length)
                    # determine angle between optical axis and width angle - for RIGHT CAM
                    angle_beta_right = atan2(cam_info.width/2 - pos_right.u, cam_info.focal_length)

                    # CP is the distance between camera and target
                    c_p = dist_cam_prj_to_target / (cos(angle_alpha_left)*cos(angle_beta_left))
                    # determine 3d coordinates with respect to left cam
                    x_new_l = left_cam.x + c_p*cos(angle_alpha_left + pitch)*cos(yaw + angle_beta_left)
                    y_new_l = left_cam.y + c_p*cos(angle_alpha_left + pitch)*sin(yaw + angle_beta_left)
                    z_new_l = left_cam.z + c_p*sin(angle_alpha_left + pitch)
                    c_p = dist_cam_prj_to_target / (cos(angle_alpha_right) * cos(angle_beta_right))
                    # determine 3d coordinates with respect to right cam
                    x_new_r = right_cam.x + c_p*cos(angle_alpha_right + pitch)*cos(yaw + angle_beta_right)
                    y_new_r = right_cam.y + c_p*cos(angle_alpha_right + pitch)*sin(yaw + angle_beta_right)
                    z_new_r = right_cam.z + c_p*sin(angle_alpha_right + pitch)

                    # approx. 3d coordinate will be the average between the two coordinate points
                    new_pose.position.x = (x_new_l + x_new_r)/2
                    new_pose.position.y = (y_new_l + y_new_r)/2
                    new_pose.position.z = (z_new_l + z_new_r)/2
                    # the measurement
                    target_coord_measurement = CamPosWorldCoord(new_pose.position)

                    # get target model state
                    try:
                        real_target_state = get_model_state("target0", "")
                    except rospy.service.ServiceException as e:
                        print e
                    # real target coordinates (simulation)
                    target_coord = CamPosWorldCoord(real_target_state.pose.position)

                    # data filtering using KF
                    # predict current state
                    kf.predict()
                    # get target position as numpy array
                    coord_measurement = target_coord_measurement.return_pos_array()
                    # update current state
                    kf.correct(coord_measurement)

                    # get position data
                    filtered_data = kf.state
                    # fill TargetState instance for publishing
                    target_state.x = filtered_data[0]  # x position
                    target_state.y = filtered_data[1]  # y position
                    target_state.z = filtered_data[2]  # z position
                    target_state.vx = filtered_data[3]  # x velocity
                    target_state.vy = filtered_data[4]  # y velocity
                    target_state.vz = filtered_data[5]  # z velocity
                    # publish TargetState message
                    state_publisher.publish(target_state)

                    # PLOT --------------------------------------------------------------
                    filter_pos.position.x = filtered_data[0]
                    filter_pos.position.y = filtered_data[1]
                    filter_pos.position.z = filtered_data[2]

                    # PLOT DATA - real pos state, measurement and filtered pos state
                    real_state_list.append(target_coord)
                    measure_state_list.append(target_coord_measurement)
                    filter_state_list.append(CamPosWorldCoord(filter_pos.position))

                    # PLOT DATA - filtered data
                    filter_error_pose.position.x = abs(filtered_data[0] - target_coord.x)
                    filter_error_pose.position.y = abs(filtered_data[1] - target_coord.y)
                    filter_error_pose.position.z = abs(filtered_data[2] - target_coord.z)
                    filter_error = CamPosWorldCoord(filter_error_pose.position)
                    filter_error.print_coord("Flt ")
                    # append current error to the list for plotting
                    filter_error_list.append(filter_error)
                    # --------

                    # PLOT DATA - unfiltered data
                    # error values for each axis
                    error_pose.position.x = abs(target_coord.x - target_coord_measurement.x)
                    error_pose.position.y = abs(target_coord.y - target_coord_measurement.y)
                    error_pose.position.z = abs(target_coord.z - target_coord_measurement.z)
                    pos_error = CamPosWorldCoord(error_pose.position)
                    # append current error to the list for plotting
                    measure_error_list.append(pos_error)
                    # --------

            loop_rate.sleep()

        except rospy.ROSException as e:
            if plot_var == "Error":
                # test only
                x_error = list()
                y_error = list()
                z_error = list()
                sample_nr = list()
                # filter
                flt_x_error = list()
                flt_y_error = list()
                flt_z_error = list()

                # unfiltered data
                i = 0
                for coord_instance in measure_error_list:

                    sample_nr.append(i)
                    x_error.append(coord_instance.x)
                    y_error.append(coord_instance.y)
                    z_error.append(coord_instance.z)
                    i += 1

                # filtered data
                for coord_instance in filter_error_list:
                    flt_x_error.append(coord_instance.x)
                    flt_y_error.append(coord_instance.y)
                    flt_z_error.append(coord_instance.z)

                # plot errors
                plt.figure(1)
                plt.plot([1, 2, 3])
                plt.subplot(1, 2, 1)
                plt.plot(sample_nr, x_error, "r", label="Date nefiltrate")
                plt.ylim((0, 1))
                plt.xlabel("Esantioane")
                plt.ylabel("Eroarea")
                #plt.title("Eroarea dintre pozitia reala si cea aproximata - axa X")
                plt.legend()

                plt.subplot(1, 2, 2)
                plt.plot(sample_nr, flt_x_error, "b", label="Date filtrate")
                plt.ylim((0, 1))
                plt.xlabel("Esantioane")
                plt.ylabel("Eroarea")
                #plt.title("Eroarea dintre pozitia reala si cea aproximata - axa X")
                plt.legend()

                plt.savefig("eroarex.png")
                plt.show()

                plt.figure(2)
                plt.plot([1, 2, 3])
                plt.subplot(1, 2, 1)
                plt.plot(sample_nr, y_error, "r", label="Date nefiltrate")
                plt.ylim((0, 1))
                plt.xlabel("Esantioane")
                plt.ylabel("Eroarea")
                #plt.title("Eroarea dintre pozitia reala si cea aproximata - axa Y")
                plt.legend()

                plt.subplot(1, 2, 2)
                plt.plot(sample_nr, flt_y_error, "b", label="Date filtrate")
                plt.ylim((0, 1))
                plt.xlabel("Esantioane")
                plt.ylabel("Eroarea")
                #plt.title("Eroarea dintre pozitia reala si cea aproximata - axa Y")
                plt.legend()

                plt.savefig("eroarey.png")
                plt.show()

                plt.figure(3)
                plt.plot([1, 2, 3])
                plt.subplot(1, 2, 1)
                plt.ylim((0, 1))
                plt.plot(sample_nr, z_error, "r", label="Date nefiltrate")
                plt.xlabel("Esantioane")
                plt.ylabel("Eroarea")
                #plt.title("Eroarea dintre pozitia reala si cea aproximata - axa Z")
                plt.legend()

                plt.subplot(1, 2, 2)
                plt.plot(sample_nr, flt_z_error, "b", label="Date filtrate")
                plt.ylim((0, 1))
                plt.xlabel("Esantioane")
                plt.ylabel("Eroarea")
                #plt.title("Eroarea dintre pozitia reala si cea aproximata - axa Z")
                plt.legend()

                plt.savefig("eroarez.png")
                plt.show()

            # real pos state, measurement and filtered pos state plot
            if plot_var == "Pos":
                x_pos = list()
                y_pos = list()
                z_pos = list()
                nr_samples = list()
                i = 0

                for real_pos in real_state_list:
                    nr_samples.append(i)
                    x_pos.append(real_pos.x)
                    y_pos.append(real_pos.y)
                    z_pos.append(real_pos.z)
                    i += 1

                x_measure_pos = list()
                y_measure_pos = list()
                z_measure_pos = list()

                for measure_pos in measure_state_list:
                    x_measure_pos.append(measure_pos.x)
                    y_measure_pos.append(measure_pos.y)
                    z_measure_pos.append(measure_pos.z)

                x_filter_pos = list()
                y_filter_pos = list()
                z_filter_pos = list()

                for filter_pos in filter_state_list:
                    x_filter_pos.append(filter_pos.x)
                    y_filter_pos.append(filter_pos.y)
                    z_filter_pos.append(filter_pos.z)

                # plot errors
                plt.figure(1)
                plt.plot(nr_samples, x_pos, "r", label="Pozitia x reala")
                plt.plot(nr_samples, x_measure_pos, "b", label="Pozitia x masurata")
                plt.plot(nr_samples, x_filter_pos, "g", label="Pozitia x filtrata")
                plt.xlabel("Esantioane")
                plt.ylabel("Pozitia x")
                plt.legend()
                plt.savefig("pozx.png")
                plt.show()

                plt.figure(2)
                plt.plot(nr_samples, y_pos, "r", label="Pozitia y reala")
                plt.plot(nr_samples, y_measure_pos, "b", label="Pozitia y masurata")
                plt.plot(nr_samples, y_filter_pos, "g", label="Pozitia y filtrata")
                plt.xlabel("Esantioane")
                plt.ylabel("Pozitia y")
                plt.legend()
                plt.savefig("pozy.png")
                plt.show()

                plt.figure(3)
                plt.plot(nr_samples, z_pos, "r", label="Pozitia z reala")
                plt.plot(nr_samples, z_measure_pos, "b", label="Pozitia z masurata")
                plt.plot(nr_samples, z_filter_pos, "g", label="Pozitia z filtrata")
                plt.xlabel("Esantioane")
                plt.ylabel("Pozitia z")
                plt.legend()
                plt.savefig("pozz.png")
                plt.show()

            print e

        end_time = time.time()
        # print end_time - start_time


if __name__ == '__main__':
    main()
