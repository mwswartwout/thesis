#!/usr/bin/env python

import rospy
import helpers as h
import math
import random
import subprocess

from geometry_msgs.msg import Twist, Pose2D, PoseWithCovarianceStamped
from turtlebot import TurtleBot
from project1.msg import ScanWithVarianceStamped
from nav_msgs.msg import Odometry


class TurtleBot1D(TurtleBot, object):
    """
    A class for a TurtleBot 2 to move between two other static robots which act as helpful landmarks.
    These landmarks have laser scanners like the moving robot, so they feed their own estimates of the moving robot's
    position through an EKF or UKF node that accurately predicts the position of the moving robot.
    author: Shaun Howard and Matt Swartwout
    """
    def __init__(self, speed=.2):
        super(TurtleBot1D, self).__init__()

        rospy.on_shutdown(self.stop)

        # initialize all variable values
        self.robot_1_distance = None
        self.robot_3_distance = None
        self.robot_1_position = None
        self.robot_3_position = None
        self.pose11 = PoseWithCovarianceStamped()
        self.pose33 = PoseWithCovarianceStamped()
        self.pose32 = PoseWithCovarianceStamped()
        self.covariance11 = [0] * 36
        self.covariance33 = [0] * 36
        self.covariance32 = [0] * 36
        self.odom_pose = None
        self.speed = speed
        self.initialize_subscribers()
        self.initialize_publishers()

    def initialize_publishers(self):
        super(TurtleBot1D, self).initialize_publishers()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist,
                                           queue_size=1)

        self.pose_wrt_1_from_1 = rospy.Publisher('pose11',
                                                 PoseWithCovarianceStamped,
                                                 queue_size=1)

        self.pose_wrt_3_from_3 = rospy.Publisher('pose33',
                                                 PoseWithCovarianceStamped,
                                                 queue_size=1)

        self.pose_wrt_3_from_2 = rospy.Publisher('pose32',
                                                 PoseWithCovarianceStamped,
                                                 queue_size=1)

    def initialize_subscribers(self):
        super(TurtleBot1D, self).initialize_subscribers()
        self.robot_1_dist = rospy.Subscriber('/turtlebot1/processed_scan',
                                             ScanWithVarianceStamped,
                                             self.robot_1_dist_cb)

        self.robot_3_dist = rospy.Subscriber('/turtlebot3/processed_scan',
                                             ScanWithVarianceStamped,
                                             self.robot_3_dist_cb)

        self.robot_1_pos = rospy.Subscriber('/turtlebot1/position', Pose2D,
                                            self.robot_1_pos_cb)

        self.robot_3_pos = rospy.Subscriber('/turtlebot3/position', Pose2D,
                                            self.robot_3_pos_cb)

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)

    def scan_callback(self, scan_msg):
        super(TurtleBot1D, self).scan_callback(scan_msg)
        self.update_pose_32()

    # static robots 1 and 3 have distance callbacks from their processed scan to the moving robot

    def robot_1_dist_cb(self, distance):
        rospy.logdebug("Updated robot 1 distance")
        self.robot_1_distance = distance
        self.update_pose_11()

    def robot_3_dist_cb(self, distance):
        rospy.logdebug("Updated robot 3 distance")
        self.robot_3_distance = distance
        self.update_pose_33()

    # all the robots have odometry callbacks below

    def robot_1_pos_cb(self, position):
        self.robot_1_position = position

    def robot_3_pos_cb(self, position):
        self.robot_3_position = position

    def odom_cb(self, odom):
        self.odom_pose = odom.pose.pose.position

    def update_pose_11(self):
        # update the pose estimate for the first robot to the moving robot
        scan = self.robot_1_distance.scan
        if not h.scan_has_none_check(scan):
            self.pose11.pose.pose.position.x = self.robot_1_position.x + scan.median - self.pose.x
            self.covariance11[0] = self.robot_1_distance.scan.variance
            self.pose11.pose.covariance = self.covariance11
            self.pose11.header.stamp = rospy.get_rostime()
            self.pose11.header.frame_id = self.namespace + 'odom'
            self.pose_wrt_1_from_1.publish(self.pose11)
        else:
            rospy.logdebug("robot 1 scan items are none, cannot yet acquire feedback.")

    def update_pose_33(self):
        # update the pose estimate for the third robot to the moving robot
        scan = self.robot_3_distance.scan
        if not h.scan_has_none_check(scan):
            self.pose33.pose.pose.position.x = self.robot_3_position.x - scan.median - self.pose.x
            self.covariance33[0] = self.robot_3_distance.scan.variance
            self.pose33.pose.covariance = self.covariance33
            self.pose33.header.stamp = rospy.get_rostime()
            self.pose33.header.frame_id = self.namespace + 'odom'
            self.pose_wrt_3_from_3.publish(self.pose33)
        else:
            rospy.logdebug("robot 3 scan items are none, cannot yet acquire feedback.")

    def update_pose_32(self):
        # update the pose estimate for the moving robot to the third, static robot
        if self.processed_scan is not None and self.robot_3_position is not None:
            self.pose32.pose.pose.position.x = self.robot_3_position.x - self.processed_scan.scan.median - self.pose.x
            # Have to include this since robot_localization forces us to fuse y and yaw on at least one sensor
            self.pose32.pose.pose.orientation.w = 1
            self.covariance32[0] = self.processed_scan.scan.variance
            self.pose32.pose.covariance = self.covariance32
            self.pose32.header.stamp = rospy.get_rostime()
            self.pose32.header.frame_id = self.namespace + 'odom'
            self.pose_wrt_3_from_2.publish(self.pose32)

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
    robot = TurtleBot1D()

    # Wait for everything else in Gazebo world to be ready
    rospy.sleep(7)

    # Once everything is ready we need to reset our filters
    # because they could have gotten erroneous readings
    rospy.loginfo("Calling service reset")
    subprocess.Popen(["rosservice", "call", "set_pose_distributed", "{}"])
    subprocess.Popen(["rosservice", "call", "set_pose_self","{}"])
    rospy.loginfo("Poses reset")
    rospy.sleep(3)

    # move the robot back and forth randomly until process killed with ctrl-c
    while not rospy.is_shutdown():
        robot.move(amount=random.uniform(-1, 1))
        rospy.sleep(1)

if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        print "exiting turtle"
        pass
