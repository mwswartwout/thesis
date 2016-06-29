#!/usr/bin/env python

import rospy
import math
import random
import tf

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

    def move(self, goal_x, goal_y, x_lower_bound=-1, x_upper_bound=1, y_lower_bound=-1, y_upper_bound=1):
        # Employs dead-reckoning to move TurtleBot via a desired heading and move distance
        current_x = self.current_continuous_pose.pose.pose.position.x
        current_y = self.current_continuous_pose.pose.pose.position.y

        # To prevent robot getting stuck or drifting towards just one specific area over course of experiment
        # Randomly send to middle of allowable area with 1% probability
        rand = random.uniform(0, 100)
        if rand == 1:
            goal_x = (x_lower_bound + x_upper_bound) / 2
            goal_y = (y_lower_bound + y_upper_bound) / 2

        # Check if desired location is within our set bounds
        within_bounds_x = self.check_move_bounds(goal_x, x_lower_bound, x_upper_bound)
        within_bounds_y = self.check_move_bounds(goal_y, y_lower_bound, y_upper_bound)
        rospy.logdebug(self.namespace + ': Requesting move from (' + str(current_x) + ', ' + str(current_y) +') to (' +
                       str(goal_x) + ', ' + str(goal_y) + ')')
        if within_bounds_x and within_bounds_y:
            rospy.logdebug(self.namespace + ': Move was in bounds and accepted')
            # Calculate required yaw of rotation and distance of translation
            distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
            yaw = math.atan2(goal_y - current_y, goal_x - current_x)

            # First execute rotation to desired heading
            self.rotate(yaw)

            # Next translate to desired location
            if not self.lidar_alarm:
                self.translate(distance)
                current_x = self.current_continuous_pose.pose.pose.position.x
                current_y = self.current_continuous_pose.pose.pose.position.y
                rospy.logdebug('Final location after move is (' + str(current_x) + ', ' + str(current_y) + ')' +
                               'requested location was (' + str(goal_x) + ', ' + str(goal_y) + ')')
            else:
                rospy.logwarn('Not executing translation because lidar alarm is triggered')
        else:
            rospy.logdebug('Move rejected because goal is out of bounds' + str(goal_x) + ',' + str(goal_y))

    def rotate(self, yaw):
        yaw = self.correct_angle(yaw)
        move_cmd = Twist()
        current_yaw = self.convert_quaternion_to_yaw(self.current_continuous_pose.pose.pose.orientation)
        rospy.logdebug(self.namespace + ': Rotation requested to yaw of ' + str(yaw) +
                       ', current yaw is ' + str(current_yaw))
        if current_yaw < yaw:  # Positive rotation needed
            move_cmd.angular.z = self.angular_speed
        else:  # Negative rotation needed
            move_cmd.angular.z = -self.angular_speed

        distance_to_goal = math.fabs(current_yaw - yaw)
        move_time = distance_to_goal / self.angular_speed
        move_steps = move_time * self.rate_frequency
        while move_steps > 0 and not rospy.is_shutdown():
            try:
                self.cmd_vel_pub.publish(move_cmd)
                self.rate.sleep()
                move_steps -= 1
            except rospy.ROSException as e:
                rospy.logwarn('Unable to publish rotation command')
                rospy.logwarn(e.message)
        self.stop()  # Stop once move is complete
        current_yaw = self.convert_quaternion_to_yaw(self.current_continuous_pose.pose.pose.orientation)
        rospy.logdebug(self.namespace + ': Yaw after rotation is ' + str(current_yaw) +
                       ', requested yaw was ' + str(yaw))

    def translate(self, distance):
        move_cmd = Twist()
        if distance < 0:
            rospy.logwarn("Distance for movement should not be negative! Aborting translation...")
        else:
            rospy.logdebug(self.namespace + ': Translation requested of distance ' + str(distance) + ' meters')
            move_cmd.linear.x = self.linear_speed

            move_time = distance / self.linear_speed
            move_steps = move_time * self.rate_frequency
            while move_steps > 0 and not rospy.is_shutdown() and not self.lidar_alarm:
                try:
                    self.cmd_vel_pub.publish(move_cmd)
                    self.rate.sleep()
                    # decrease and update distance to goal
                    move_steps -= 1
                except rospy.ROSException as e:
                    rospy.logwarn(self.namespace + ': Unable to publish translation command - ' + e.message)
            self.stop()  # Stop once move is complete

    def stop(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': Unable to publish stop command - ' + e.message)

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

        # TODO add in calculation to rotate in shortest direction
        # Make sure we're using the shortest rotation
        #if yaw > math.pi:
        #    yaw = -1 * (2*math.pi - yaw)

        return yaw


def main():
    # Wait for gazebo to be fully initialized before starting our robot
    rospy.wait_for_service('/gazebo/set_physics_properties')

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
        robot.move(goal_x=random.uniform(x_lower, x_upper),
                   goal_y=random.uniform(y_lower, y_upper),
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
