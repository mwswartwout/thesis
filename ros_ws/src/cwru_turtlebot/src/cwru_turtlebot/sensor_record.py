#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64

continuous_data = None
discrete_data = None
gazebo_data = None
external_count = None
namespace = None


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


def external_pose_count_callback(count):
    global external_count
    external_count = count.data


def write_headers():
    filename = '/home/matt/' + namespace + '_continuous_odometry_filtered.csv'
    with open(filename, 'a+') as pose_file:
        writer = csv.writer(pose_file)
        writer.writerow('x_position, y_position, pose_covariance, twist_covariance')

    filename = '/home/matt/' + namespace + '_discrete_odometry_filtered.csv'
    with open(filename, 'a+') as pose_file:
        writer = csv.writer(pose_file)
        writer.writerow('x_position, y_position, pose_covariance, twist_covariance')

    filename = '/home/matt/' + namespace + '_gazebo_odometry_filtered.csv'
    with open(filename, 'a+') as pose_file:
        writer = csv.writer(pose_file)
        writer.writerow('x_position, y_position, pose_covariance, twist_covariance')

    filename = '/home/matt/' + namespace + '_external_pose_count.csv'
    with open(filename, 'a+') as count_file:
        writer = csv.writer(count_file)
        writer.writerow('count')


def write_to_files(event):
    global continuous_data
    global discrete_data
    global gazebo_data
    global external_count
    global namespace

    if None not in (continuous_data, discrete_data, gazebo_data, external_count):
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

        filename = '/home/matt/' + namespace + '_external_pose_count.csv'
        with open(filename, 'a+') as count_file:
            writer = csv.writer(count_file)
            writer.writerow(external_count)


def main():
    rospy.init_node('sensor_record')

    global namespace
    namespace = rospy.get_namespace()[1:-1]

    continuous_odom_subscriber = rospy.Subscriber('odometry/filtered_continuous', Odometry, continuous_odom_callback)
    discrete_odom_subscriber = rospy.Subscriber('odometry/filtered_discrete', Odometry, discrete_odom_callback)
    gazebo_odom_subscriber = rospy.Subscriber('odom', Odometry, gazebo_odom_callback)
    external_pose_count_subscriber = rospy.Subscriber('external_poses_count', UInt64, external_pose_count_callback)

    write_headers()

    timer = rospy.Timer(rospy.Duration(.1), write_to_files)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
