#!/usr/bin/env python
# -------------------

import rospy
import sys

from simulator_gazebo.msg import CamModuleInfo
from simulator_gazebo.srv import GetCamModuleInfo, GetCamModuleInfoResponse
from gazebo_msgs.srv      import GetModelState, GetLinkState

from classes.cam_classes  import CameraInfoSub

def main():

    # init ROS node and wait for the specific services
    rospy.init_node("camera_info_srv")
    rospy.wait_for_service("gazebo/get_model_state")
    rospy.wait_for_service("gazebo/get_link_state")

    # create service proxy for both services
    getModelState = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
    getLinkState  = rospy.ServiceProxy("gazebo/get_link_state", GetLinkState)

    # the base string for model names
    baseName  = "camera"
    indexName = 0

    # create model name and call get model state service
    modelName  = baseName + str(indexName)
    modelState = getModelState(modelName, "")

    # save data for each camera module in an array of classes
    dictData = []

    # camera models are indexed starting with 0;
    # when there's no module with current index, exit loop;
    while modelState.success:
        # create object instance of camera module class
        data = CamModuleInfo()

        # write model name and pose
        data.model_name = modelName
        data.pose = modelState.pose

        # get link state of the left link to get distance between cams
        leftLinkState  = getLinkState(modelName + "::left", modelName)
        rightLinkState = getLinkState(modelName + "::right", modelName)
        if leftLinkState.success and rightLinkState.success:
            data.dist_cameras = abs(leftLinkState.link_state.pose.position.y -
                                   rightLinkState.link_state.pose.position.y)

        # create object instance for subcriber class
        dataSub = CameraInfoSub(modelName)

        # subscribed to camera info topic; wait until one message has arrived
        # in order to get focal length and resolution data
        while not dataSub.subDone:
            pass

        # after a message has arrived, transfer the data to camera module instance
        data.focal_length = dataSub.focalLength
        data.height       = dataSub.height
        data.width        = dataSub.width

        # append data message to array
        dictData.append(data)

        # after appending message to the array, update model name and get new model state
        indexName += 1
        modelName  = baseName + str(indexName)
        modelState = getModelState(modelName, "")

    def callbackSrvReq(req):
        return GetCamModuleInfoResponse(dictData)

    service = rospy.Service("simulator_gazebo/get_cam_module_info", GetCamModuleInfo, callbackSrvReq)
    rospy.spin()

if __name__ == '__main__':
    main()
