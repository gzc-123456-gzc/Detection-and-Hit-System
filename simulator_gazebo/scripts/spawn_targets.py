#!/usr/bin/env python

import random
import rospy
import os

from geometry_msgs.msg    import Pose, Point
from simulator_gazebo.msg import TargetSpawn
from simulator_gazebo.srv import SetVelocity, SetVelocityRequest
from gazebo_msgs.srv import SpawnModel, DeleteModel


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

# parameters for spawning time interval
# ---------------------------------------
spawn_interval  = 10
delete_interval = 20
# ---------------------------------------

# parameters for publisher; create publisher; publish once a new target is spawned
# ---------------------------------------
publish_topic = "/simulator_gazebo/set_target_movement"
movement_publisher = rospy.Publisher(publish_topic, TargetSpawn, queue_size = 1)
# set velocity range
vel_x_min = 0
vel_x_max = 3
vel_y_min = 0
vel_y_max = 3
vel_z_min = 1
vel_z_max = 3

# ---------------------------------------

def main():

    # initialize ROS node
    rospy.init_node("spawn_targets")

    # wait for services
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")

    # create service proxy for spawn model and delete model
    spawnModel  = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    deleteModel = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)



    # start initial spawn time; initialize target index to 0
    start_spawn  = 0
    target_index = 0
    # delete initial spawn time;
    start_delete = rospy.get_rostime().secs
    delete_index = 0
    # set loop rate
    once = False
    loop_rate = rospy.Rate(1000)
    while not rospy.is_shutdown():

        # check numbers of seconds since the last spawn
        if (rospy.get_rostime().secs - start_spawn) > spawn_interval:
            # modify model_name and init_pose; generate random initial position
            model_name = base_name + str(target_index)
            # spawn model
            spawnModel(model_name, model_xml, robot_namespace, init_pose, reference_frame)
            rospy.sleep(0.5)

            # generate velocity
            vel_x = random.randint(vel_x_min, vel_x_max)
            vel_y = random.randint(vel_y_min, vel_y_max)
            vel_z = random.randint(vel_z_min, vel_z_max)
            velocity = Point(vel_x,vel_y,vel_z)

            # create service proxy for SetVelocity server used for target plugin;
            setVel = rospy.ServiceProxy(model_name + "/set_velocity", SetVelocity)
            # call service
            setVel(SetVelocityRequest(velocity))

            # update start spawn variable and increment target index
            target_index += 1
            start_spawn = rospy.get_rostime().secs
            if not once:
                once = True
                start_delete = rospy.get_rostime().secs

        if (rospy.get_rostime().secs - start_delete) > delete_interval:
            model_del_name = base_name + str(delete_index)
            deleteModel(model_del_name)
            delete_index += 1
            start_delete = rospy.get_rostime().secs

        loop_rate.sleep()


if __name__ == '__main__':
    main()
