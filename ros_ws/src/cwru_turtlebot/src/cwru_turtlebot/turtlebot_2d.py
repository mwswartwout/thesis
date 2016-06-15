#!/usr/bin/env python

import rospy
import math
import random

from geometry_msgs.msg import Twist
from turtlebot import TurtleBot


# TODO add ROS debug statements
# TODO add something to correct odom drift

class TurtleBot2D(TurtleBot, object):

    def __init__(self, linear_speed=.2, angular_speed=.2):
        super(TurtleBot2D, self).__init__()

        rospy.on_shutdown(self.stop)

        # initialize all variable values

        self.linear_speed = linear_speed  # expressed in m/s
        self.angular_speed = angular_speed  # expressed in rad/s

    def initialize_publishers(self):
        super(TurtleBot2D, self).initialize_publishers()

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist,
                                           queue_size=1)

    def move(self, distance, yaw, x_lower_bound=-1, x_upper_bound=1, y_lower_bound=-1, y_upper_bound=1):
        # Employs dead-reckoning to move TurtleBot via a desired heading and move distance

        # First check if desired location is within our set bounds
        goal_x = self.current_continuous_pose.x + math.cos(yaw)*distance
        goal_y = self.current_continuous_pose.y + math.sin(yaw)*distance

        within_bounds_x = self.check_move_bounds(goal_x, x_lower_bound, x_upper_bound)
        within_bounds_y = self.check_move_bounds(goal_y, y_lower_bound, y_upper_bound)

        if within_bounds_x and within_bounds_y:
            # First execute rotation to desired heading
            self.rotate(yaw)

            # Next translate to desired location
            self.translate(distance)
        else:
            rospy.loginfo('Goal received out of bounds' + str(goal_x) + ',' + str(goal_y))
            rospy.loginfo('Current pose is ' + str(self.current_continuous_pose.x) + ',' + str(self.current_continuous_pose.y))

    def rotate(self, yaw):
        yaw = self.correct_angle(yaw)
        move_cmd = Twist()
        if self.current_continuous_pose.theta < yaw:  # Positive rotation needed
            move_cmd.angular.z = self.angular_speed
        else:  # Negative rotation needed
            move_cmd.angular.z = -self.angular_speed

        distance_to_goal = math.fabs(self.current_continuous_pose.theta - yaw)
        move_time = distance_to_goal / self.angular_speed
        move_steps = move_time * self.rate_frequency
        while move_steps > 0 and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()
            move_steps -= 1

    def translate(self, distance):
        move_cmd = Twist()
        if distance < 0:
            rospy.logwarn("Distance for movement should not be negative! Aborting translation...")
        else:
            move_cmd.linear.x = self.linear_speed

            move_time = distance / self.linear_speed
            move_steps = move_time * self.rate_frequency
            while move_steps > 0 and not rospy.is_shutdown():
                self.cmd_vel_pub.publish(move_cmd)
                self.rate.sleep()
                # decrease and update distance to goal
                move_steps -= 1

    def stop(self):
        rospy.logdebug('%s has stopped', self.namespace)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    @staticmethod
    def check_move_bounds(val, lower, upper):
        # check that the val is between lower and upper values
        return lower < val < upper

    @staticmethod
    def correct_angle(yaw):
        # Correct yaws to smallest possible value, accounting for periodicity
        while yaw > 2 * math.pi and not rospy.is_shutdown():
            yaw -= 2 * math.pi

        while yaw < -2 * math.pi and not rospy.is_shutdown():
            yaw += 2 * math.pi

        # Make sure we're using the shortest rotation
        if math.fabs(yaw - 2*math.pi) < yaw:
            yaw -= 2*math.pi

        return yaw


def main():
    # create a movable turtle bot object
    robot = TurtleBot2D()

    # Wait for everything else in Gazebo world to be ready
    robot.wait_for_clients()

    # Once everything is ready we need to reset our filters
    # because they could have gotten erroneous readings
    robot.reset_filters()

    x_upper = rospy.get_param('/x_upper')
    x_lower = rospy.get_param('/x_lower')
    y_upper = rospy.get_param('/y_upper')
    y_lower = rospy.get_param('/y_lower')

    # move the robot back and forth randomly until process killed with ctrl-c
    while not rospy.is_shutdown():
        robot.move(distance=random.uniform(0, 2),
                   yaw=random.uniform(0, 2*math.pi),
                   x_lower_bound=x_lower,
                   x_upper_bound=x_upper,
                   y_lower_bound=y_lower,
                   y_upper_bound=y_upper)

if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        pass
