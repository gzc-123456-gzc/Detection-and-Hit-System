#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from detect_target.msg import TargetState
from geometry_msgs.msg import Point

from sympy.core.symbol import symbols
from sympy import sin, cos
from sympy import solve


class InterceptSubscriber:

    def __init__(self):
        """ Creates subscriber for TargetState messages. """
        self.sub = rospy.Subscriber("detect_target/target_state", TargetState, callback=self.callback)
        self.position = Pose().position
        self.velocity = Pose().position
        self.is_ready_to_launch = False
        self.iter = 0
        self.__launch_iter = 75

    def callback(self, target_state):
        """
        Updates class attributes using target_state data from ROS topic.

        """
        self.position.x = target_state.x
        self.position.y = target_state.y
        self.position.z = target_state.z
        self.velocity.x = target_state.vx
        self.velocity.y = target_state.vy
        self.velocity.z = target_state.vz
        self.iter += 1
        self.__nr_of_iter()

    def print_data(self):
        """ Prints attributes. """
        print "Position: " + str(self.position)
        print "Velocity: " + str(self.velocity)

    def __nr_of_iter(self):
        """ if callback function has been called 50 times, set ready to launch variable to True """
        if self.iter > self.__launch_iter:
            self.is_ready_to_launch = True


class LauncherData:

    def __init__(self, velocity_mag, launcher_pos, intercept_time, launch_time):
        """ Initialize launcher data: magnitude of the projectile's velocity and launcher position """
        self.projectile_vel_mag = velocity_mag
        self.position = launcher_pos
        self.launch_vel = Point()
        self.theta = None
        self.phi = None
        self.intercept_time = intercept_time
        self.launch_time = launch_time
        self.__g_acc = 9.8

    def return_pos_arr(self):
        """ Returns position as np array type. """
        return np.array([self.position.x, self.position.y, self.position.z])

    def print_data(self):
        """ Prints instance data. """
        print "Launcher data."
        print "Projectile magnitude vel: " + str(self.projectile_vel_mag)
        print "Launcher position (x, y, z): (" \
              + str(self.position.x) + ", " \
              + str(self.position.y) + ", " \
              + str(self.position.z) + ")"

    def can_hit_target(self, target_pos, target_vel):
        """
        Based on target_data, determines if the target can be hit.
        Determine angles theta and phi for launching.
        """
        elapsed_time = self.intercept_time - self.launch_time
        self.launch_vel.x = (target_vel.x*self.intercept_time + target_pos.x - self.position.x)/elapsed_time
        self.launch_vel.y = (target_vel.y*self.intercept_time + target_pos.y - self.position.y)/elapsed_time
        self.launch_vel.z = (self.__g_acc*elapsed_time/2) \
                          + (target_vel.z*self.intercept_time + target_pos.z - self.position.z)/elapsed_time

        # print self.launch_vel.z / self.projectile_vel_mag
        return True


def intercept_target():
    """
    Using position and velocity data from subscribing to detect_target/target_state topic, this function determines:
    - time of launch
    - time of intercept
    - azimuth and elevation angles for the projectile

    :return:
    """
    set_vel_pub = rospy.Publisher("launchProjectile/set_projectile_vel", Point, queue_size=1)
    # constant variables
    # Point instance for launching
    launch_vel = Point()
    # velocity magnitude
    velocity_mag = 20
    # launcher projectile pos
    launcher_pos = Pose().position
    launcher_pos.x = 1
    launcher_pos.y = 1
    launcher_pos.z = 0
    # intercept time and launch time
    intercept_time = 4
    launch_time = 3
    # launcher instance
    launcher = LauncherData(velocity_mag, launcher_pos, intercept_time, launch_time)
    launcher.print_data()

    # init ROS node named "intercept_target"
    rospy.init_node("intercept_target")
    # create ROS subscriber
    target_intercepter = InterceptSubscriber()

    # set loop frequency
    loop_rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            # target_intercepter.print_data()

            # if the launcher is ready to launch
            if target_intercepter.is_ready_to_launch is True:
                # check if launcher can hit the target
                print "Launcher ready."
                launcher_status = launcher.can_hit_target(target_intercepter.position, target_intercepter.velocity)
                if launcher_status is True:
                    set_vel_pub.publish(launcher.launch_vel)
                    break

        except rospy.ROSException as e:
            print e

        loop_rate.sleep()


if __name__ == "__main__":
    intercept_target()