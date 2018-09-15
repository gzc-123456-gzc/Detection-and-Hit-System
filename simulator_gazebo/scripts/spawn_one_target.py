#!/usr/bin/env python

import rospy
import argparse

from geometry_msgs.msg import Pose, Point

from simulator_gazebo.srv import SetVelocity, SetVelocityRequest
from gazebo_msgs.srv import SpawnModel

# create parser for command line arguments
parser = argparse.ArgumentParser(description="Set target velocity.")
parser.add_argument("vel", metavar="VEL", type=float, nargs="*",
                    help="velocity list [x,y,z]")
parser.add_argument("target_index", metavar="INDEX", type=int, nargs="+",
                    help="target index")
args = parser.parse_args()
print args
# parameters needed by spawn model service
# ---------------------------------------
# name of the model to be spawned must be unique in the simulation
_base_name = "target"
# path to SDF file for the model
_model_path = "/home/marius/Desktop/licenta2_ws/src/simulator_gazebo/models/target.sdf"
# upload the xml file
_file = open(_model_path, 'r')
_model_xml = _file.read()
_file.close()
# the namespace will be the inertial frame (world frame)
_robot_namespace = ""
# initial pose of the spawned model
_init_pose = Pose()
_init_pose.position = Point(0, 0, 0)
# the model will be spawned at the pose given by init_pose with reference to reference_frame
_reference_frame = "world"
# ---------------------------------------
# ---------------------------------------


def spawn_one_target():
    global args
    # initialize ROS node
    rospy.init_node("spawn_targets")

    # wait for services
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    # create service proxy for spawn model and delete model
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    target_index = args.target_index[0]
    # modify model_name and init_pose; generate random initial position
    model_name = _base_name + str(target_index)
    # spawn model
    spawn_model(model_name, _model_xml, _robot_namespace, _init_pose, _reference_frame)
    rospy.sleep(0.5)

    # generate velocity>
    vel_x = args.vel[0]
    vel_y = args.vel[1]
    vel_z = args.vel[2]
    velocity = Point(vel_x, vel_y, vel_z)

    # create service proxy for SetVelocity server used for target plugin;
    set_vel = rospy.ServiceProxy(model_name + "/set_velocity", SetVelocity)
    # call service
    set_vel(SetVelocityRequest(velocity))


if __name__ == '__main__':
    spawn_one_target()
