#!/usr/bin/env python
# -------------------

import rospy

from simulator_gazebo.msg import CamModuleInfo
from simulator_gazebo.srv import GetCamModuleInfo, GetCamModuleInfoResponse
from gazebo_msgs.srv import GetModelState, GetLinkState

from classes.cam_classes import CameraInfoSub


def start_cam_server():

    # init ROS node and wait for the specific services
    rospy.init_node("camera_info_srv")
    rospy.wait_for_service("gazebo/get_model_state")
    rospy.wait_for_service("gazebo/get_link_state")

    # create service proxy for both services
    get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
    get_link_state = rospy.ServiceProxy("gazebo/get_link_state", GetLinkState)

    # the base string for model names
    base_name = "camera"
    index_name = 0

    # create model name and call get model state service
    model_name = base_name + str(index_name)
    model_state = get_model_state(model_name, "")

    # save data for each camera module in an array of classes
    dict_data = []

    # camera models are indexed starting with 0;
    # when there's no module with current index, exit loop;
    while model_state.success:
        # create object instance of camera module class
        data = CamModuleInfo()
        data.module_pose = model_state.pose
        # write model name and pose
        data.model_name = model_name
        # get link state of the left link to get distance between cams
        left_link_state  = get_link_state(model_name + "::left", model_name)
        right_link_state = get_link_state(model_name + "::right", model_name)
        if left_link_state.success and right_link_state.success:
            # distance between cameras is given by the difference between y pos of each camera
            # different pose for camera module is given by rotating and translating
            data.dist_cameras = abs(left_link_state.link_state.pose.position.y -
                                    right_link_state.link_state.pose.position.y)

        data.left_pose = left_link_state.link_state.pose
        data.right_pose = right_link_state.link_state.pose
        # create object instance for subcriber class
        data_sub = CameraInfoSub(model_name)
        # subscribed to camera info topic; wait until one message has arrived
        # in order to get focal length and resolution data
        while not data_sub.sub_done:
            pass

        # after a message has arrived, transfer the data to camera module instance
        data.focal_length = data_sub.focal_length
        data.height = data_sub.height
        data.width = data_sub.width

        # append data message to array
        dict_data.append(data)

        # after appending message to the array, update model name and get new model state
        index_name += 1
        model_name = base_name + str(index_name)
        model_state = get_model_state(model_name, "")

    def callback_srv_req(req):
        return GetCamModuleInfoResponse(dict_data)

    service = rospy.Service("simulator_gazebo/get_cam_module_info", GetCamModuleInfo, callback_srv_req)
    rospy.spin()


if __name__ == '__main__':
    start_cam_server()
