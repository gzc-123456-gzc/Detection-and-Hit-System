#!/usr/bin/env python

import random
import rospy
import os

from geometry_msgs.msg    import Pose, Point
from simulator_gazebo.msg import TargetSpawn
from simulator_gazebo.srv import SetVelocity, SetVelocityRequest
from gazebo_msgs.srv      import SpawnModel, DeleteModel


# parameters needed by spawn model service
# ---------------------------------------
# name of the model to be spawned must be unique in the simulation
base_name = "target"
# path to SDF file for the model
model_path = "/home/marius/Desktop/licenta2_ws/src/simulator_gazebo/models/target.sdf"
# upload the xml file
file = open(model_path,'r')
model_xml = file.read()
file.close()
# the namespace will be the inertial frame (world frame)
robot_namespace = ""
# initial pose of the spawned model
init_pose = Pose()
init_pose.position = Point(0,0,0)
# the model will be spawned at the pose given by init_pose with reference to reference_frame
reference_frame = "world"
# ---------------------------------------
# ---------------------------------------

def main():

    # initialize ROS node
    rospy.init_node("spawn_targets")

    # wait for services
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    # create service proxy for spawn model and delete model
    spawn_model  = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # start initial spawn time; initialize target index to 0
    start_spawn  = 0
    target_index = 0
    # modify model_name and init_pose; generate random initial position
    model_name = base_name + str(target_index)
    # spawn model
    spawn_model(model_name, model_xml, robot_namespace, init_pose, reference_frame)
    rospy.sleep(0.5)

    # generate velocity>
    vel_x = 1.5
    vel_y = 1
    vel_z = 0.5
    velocity = Point(vel_x,vel_y,vel_z)

    # create service proxy for SetVelocity server used for target plugin;
    set_vel = rospy.ServiceProxy(model_name + "/set_velocity", SetVelocity)
    # call service
    set_vel(SetVelocityRequest(velocity))

if __name__ == '__main__':
    main()
