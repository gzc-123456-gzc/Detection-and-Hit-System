#!/usr/bin/env python

import rospy

from sensor_msgs.msg import CameraInfo

class CameraInfoSub:

    def __init__(self, modelName = "camera0"):
        self.model_name   = modelName
        self.focal_length = 0
        self.height       = 0
        self.width        = 0
        self.sub_done     = False
        self.subscriber   = rospy.Subscriber("/" + self.model_name + "/left_camera/camera_info", CameraInfo, self.camInfoCallback, queue_size=1)

    def camInfoCallback(self, ros_data):
        self.focal_length = ros_data.K[0]
        self.height       = ros_data.height
        self.width        = ros_data.width
        self.sub_done     = True
