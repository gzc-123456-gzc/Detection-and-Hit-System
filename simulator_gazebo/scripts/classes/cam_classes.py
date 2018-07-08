#!/usr/bin/env python

import rospy

from sensor_msgs.msg import CameraInfo

class CameraInfoSub:

    def __init__(self, modelName = "camera0"):
        self.modelName   = modelName
        self.focalLength = 0
        self.height      = 0
        self.width       = 0
        self.subDone     = False
        self.subscriber  = rospy.Subscriber("/" + self.modelName + "/left_camera/camera_info", CameraInfo, self.camInfoCallback, queue_size=1)

    def camInfoCallback(self, rosData):
        self.focalLength = rosData.K[0]
        self.height      = rosData.height
        self.width       = rosData.width
        self.subDone     = True
