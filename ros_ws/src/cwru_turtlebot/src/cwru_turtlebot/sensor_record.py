#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Odometry

continuous_data = None
discrete_data = None
gazebo_data = None


def continuous_odom_callback(new_odom):
    global continuous_data

    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_covariance = new_odom.pose.covariance
    twist_covariance = new_odom.twist.covariance
    continuous_data = [pose_x, pose_y, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]


def discrete_odom_callback(new_odom):
    global discrete_data

    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_covariance = new_odom.pose.covariance
    twist_covariance = new_odom.twist.covariance
    discrete_data = [pose_x, pose_y, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]


def gazebo_odom_callback(new_odom):
    global gazebo_data

    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_covariance = new_odom.pose.covariance
    twist_covariance = new_odom.twist.covariance
    gazebo_data = [pose_x, pose_y, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]


def write_to_files(event):
    global continuous_data
    global discrete_data
    global gazebo_data

    namespace = rospy.get_namespace()[1:-1]

    if None not in (continuous_data, discrete_data, gazebo_data):
        filename = '/home/matt/' + namespace + '_continuous_odometry_filtered.csv'
        with open(filename, 'a+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(continuous_data)

        filename = '/home/matt/' + namespace + '_discrete_odometry_filtered.csv'
        with open(filename, 'a+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(discrete_data)

        filename = '/home/matt/' + namespace + '_gazebo_odometry_filtered.csv'
        with open(filename, 'a+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(gazebo_data)


def main():
    rospy.init_node('sensor_record')
    continuous_odom_subscriber = rospy.Subscriber('odometry/filtered_continuous', Odometry, continuous_odom_callback)
    discrete_odom_subscriber = rospy.Subscriber('odometry/filtered_discrete', Odometry, discrete_odom_callback)
    gazebo_odom_subscriber = rospy.Subscriber('odom', Odometry, gazebo_odom_callback)

    timer = rospy.Timer(rospy.Duration(.1), write_to_files)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
