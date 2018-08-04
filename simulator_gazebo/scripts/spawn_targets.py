#!/usr/bin/env python

import random
import rospy

from geometry_msgs.msg import Pose, Point
from simulator_gazebo.msg import TargetSpawn

from simulator_gazebo.srv import SetVelocity, SetVelocityRequest
from gazebo_msgs.srv import SpawnModel, DeleteModel


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

# parameters for spawning time interval
# ---------------------------------------
_spawn_interval = 10
_delete_interval = 20
# ---------------------------------------

# parameters for publisher; create publisher; publish once a new target is spawned
# ---------------------------------------
_publish_topic = "/simulator_gazebo/set_target_movement"
_movement_publisher = rospy.Publisher(_publish_topic, TargetSpawn, queue_size=1)
# set velocity range
_vel_x_min = 0
_vel_x_max = 3
_vel_y_min = 0
_vel_y_max = 3
_vel_z_min = 1
_vel_z_max = 3

# ---------------------------------------


def main():

    # initialize ROS node
    rospy.init_node("spawn_targets")

    # wait for services
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")

    # create service proxy for spawn model and delete model
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    # start initial spawn time; initialize target index to 0
    start_spawn = 0
    target_index = 0
    # delete initial spawn time;
    start_delete = rospy.get_rostime().secs
    delete_index = 0
    # set loop rate
    once = False
    loop_rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        # check numbers of seconds since the last spawn
        if (rospy.get_rostime().secs - start_spawn) > _spawn_interval:
            # modify model_name and init_pose; generate random initial position
            model_name = _base_name + str(target_index)
            # spawn model
            spawn_model(model_name, _model_xml, _robot_namespace, _init_pose, _reference_frame)
            rospy.sleep(0.5)

            # generate velocity
            vel_x = random.randint(_vel_x_min, _vel_x_max)
            vel_y = random.randint(_vel_y_min, _vel_y_max)
            vel_z = random.randint(_vel_z_min, _vel_z_max)
            velocity = Point(vel_x, vel_y, vel_z)

            # create service proxy for SetVelocity server used for target plugin;
            set_vel = rospy.ServiceProxy(model_name + "/set_velocity", SetVelocity)
            # call service
            set_vel(SetVelocityRequest(velocity))

            # update start spawn variable and increment target index
            target_index += 1
            start_spawn = rospy.get_rostime().secs
            if not once:
                once = True
                start_delete = rospy.get_rostime().secs

        if (rospy.get_rostime().secs - start_delete) > _delete_interval:
            model_del_name = _base_name + str(delete_index)
            delete_model(model_del_name)
            delete_index += 1
            start_delete = rospy.get_rostime().secs

        loop_rate.sleep()


if __name__ == '__main__':
    main()
