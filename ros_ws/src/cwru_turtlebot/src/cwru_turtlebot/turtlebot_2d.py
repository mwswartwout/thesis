#!/usr/bin/env python

import rospy
import math
import random

from geometry_msgs.msg import Twist, PoseStamped
from turtlebot import TurtleBot
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from helpers import convert_quaternion_to_yaw

# TODO add ROS debug statements
# TODO add something to correct odom drift
class TurtleBot2D(TurtleBot, object):

    def __init__(self, linear_speed=.2, angular_speed=.2):
        super(TurtleBot2D, self).__init__()

        rospy.on_shutdown(self.stop)

        # initialize all variable values

        self.linear_speed = linear_speed  # expressed in m/s
        self.angular_speed = angular_speed  # expressed in rad/s

    def move(self, goal_x, goal_y, x_lower_bound=-1, x_upper_bound=1, y_lower_bound=-1, y_upper_bound=1):
        # To prevent robot getting stuck or drifting towards just one specific area over course of experiment
        # Randomly send to middle of allowable area with 1% probability
        rand = random.uniform(0, 100)
        if rand == 1:
            goal_x = (x_lower_bound + x_upper_bound) / 2
            goal_y = (y_lower_bound + y_upper_bound) / 2

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation = self.current_continuous_pose.pose.pose.orientation  # Always keep our orientation the same

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        try:
            self.move_base_client.wait_for_server()
            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()
            success = self.move_base_client.get_result()
            if not success:
                rospy.logwarn(self.namespace + ': move_base was not able to successfully complete the request action')
            else:
                current_x = self.current_continuous_pose.pose.pose.position.x
                current_y = self.current_continuous_pose.pose.pose.position.y
                rospy.logdebug(self.namespace + ': requested move to (' + str(goal_x) + ', ' + str(goal_y) + ') ' +
                              'completed, current position according to continuous odom is (' + str(current_x) + ', ' +
                              str(current_y) + ')')
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': caught exception while sending move_base a movement goal - ' + e.message)

    def stop(self):
        try:
            self.move_base_client.cancel_all_goals()
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': Unable to send movement goal cancel command - ' + e.message)

    def spin(self):
        rospy.loginfo("Entering spin mode")
        cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        command = Twist()
        command.angular.z = 0.1
        while not rospy.is_shutdown():
            cmd_vel_pub.publish(command)
            rospy.loginfo(self.namespace + ': Current yaw is ' + str(convert_quaternion_to_yaw(self.current_gazebo_pose.pose.pose.orientation)))
            rospy.sleep(0.1)

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
