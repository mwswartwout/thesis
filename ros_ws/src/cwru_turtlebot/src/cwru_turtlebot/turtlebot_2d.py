#!/usr/bin/env python

import rospy
import helpers as h
import math
import random
import subprocess

from geometry_msgs.msg import Twist, Pose2D, PoseWithCovarianceStamped
from turtlebot import TurtleBot
from cwru_turtlebot.msg import ScanWithVarianceStamped
from nav_msgs.msg import Odometry


class TurtleBot2D(TurtleBot, object):
    """
    A class for a TurtleBot 2 to move between two other static robots which act as helpful landmarks.
    These landmarks have laser scanners like the moving robot, so they feed their own estimates of the moving robot's
    position through an EKF or UKF node that accurately predicts the position of the moving robot.
    author: Shaun Howard and Matt Swartwout
    """
    def __init__(self, speed=.2):
        super(TurtleBot2D, self).__init__()

        rospy.on_shutdown(self.stop)

        # initialize all variable values

        self.odom_pose = None
        self.speed = speed
        self.initialize_subscribers()
        self.initialize_publishers()

    def initialize_publishers(self):
        super(TurtleBot2D, self).initialize_publishers()

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist,
                                           queue_size=1)

    def initialize_subscribers(self):
        super(TurtleBot2D, self).initialize_subscribers()

    def scan_callback(self, scan_msg):
        super(TurtleBot2D, self).scan_callback(scan_msg)

    # TODO make this a 2D movement
    def move(self, amount, lower_bound=-1, upper_bound=1):
        # Employs dead-reckoning to move a turtle bot
        goal_x = self.odom_pose.x + amount

        within_bounds = h.check_bounds(goal_x, lower_bound, upper_bound)
        if within_bounds:
            move_cmd = Twist()
            # use opposite sign if direction is reversed
            if amount < 0:
                move_cmd.linear.x = -self.speed
            else:
                move_cmd.linear.x = self.speed
            rospy.logdebug('Robot is heading to x: %s', str(goal_x))

            # calculate the distance to the nearest goal from the robot's odom
            dist_to_goal = math.fabs(goal_x - self.odom_pose.x)
            # move closer to the goal until within a .1 meter tolerance
            while dist_to_goal > 0.1:
                self.cmd_vel_pub.publish(move_cmd)
                self.rate.sleep()
                # decrease and update distance to goal
                dist_to_goal = math.fabs(goal_x - self.odom_pose.x)
            rospy.logdebug('Robot reached x: %s', str(goal_x))
        else:
            rospy.logwarn('Goal received out of bounds')

    def stop(self):
        rospy.logdebug('%s has stopped', self.namespace)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


def main():
    # create a movable turtle bot object
    robot = TurtleBot2D()

    # Wait for everything else in Gazebo world to be ready
    #rospy.sleep(7)

    # Once everything is ready we need to reset our filters
    # because they could have gotten erroneous readings
    #rospy.loginfo("Calling service reset")
    #subprocess.Popen(["rosservice", "call", "set_pose_continuous", "{}"])
    #subprocess.Popen(["rosservice", "call", "set_pose_discrete","{}"])
    #rospy.loginfo("Poses reset")
    #rospy.sleep(3)

    # move the robot back and forth randomly until process killed with ctrl-c
    while not rospy.is_shutdown():
        #robot.move(amount=random.uniform(-1, 1))
        rospy.sleep(1)

if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        print "exiting turtle"
        pass
