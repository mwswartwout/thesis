#!/usr/bin/env python

import rospy
import csv
import os
import errno
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64
from geometry_msgs.msg import PoseWithCovarianceStamped

initial_x = None
initial_y = None
continuous_data = None
discrete_data = None
gazebo_data = None
external_count = None
namespace = None
# TODO add protections so that a lack of starting or trailing slashes does not affect prefix and file writing
prefix = None


def initial_position_callback(position):
    # Record the initial position of the robot so that we can convert the odometry values from the robot's odom frame
    # to the map frame
    global initial_x
    global initial_y
    initial_x = position.pose.pose.position.x
    initial_y = position.pose.pose.position.y


def continuous_odom_callback(new_odom):
    global continuous_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y):
        # Must add in the initial pose values to convert from odom frame to map frame
        pose_x = new_odom.pose.pose.position.x + initial_x
        pose_y = new_odom.pose.pose.position.y + initial_y
        # pose_covariance = new_odom.pose.covariance
        # twist_covariance = new_odom.twist.covariance
        # TODO see if we can split covariance values into their own items
        continuous_data = [pose_x, pose_y]#, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]
        rospy.logdebug(namespace + ': sensor_record received new continuous data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def discrete_odom_callback(new_odom):
    global discrete_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y):
        # Measurement is already in map frame so no need to convert by adding initial position
        pose_x = new_odom.pose.pose.position.x
        pose_y = new_odom.pose.pose.position.y
        # pose_covariance = new_odom.pose.covariance
        # twist_covariance = new_odom.twist.covariance
        discrete_data = [pose_x, pose_y]#, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]
        rospy.logdebug(namespace + ': sensor_record received new discrete data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def gazebo_odom_callback(new_odom):
    global gazebo_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y):
        # Must add in the initial pose values to convert from odom frame to map frame
        pose_x = new_odom.pose.pose.position.x + initial_x
        pose_y = new_odom.pose.pose.position.y + initial_y
        # pose_covariance = new_odom.pose.covariance
        # twist_covariance = new_odom.twist.covariance
        gazebo_data = [pose_x, pose_y]#, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]
        rospy.logdebug(namespace + ': sensor_record received new gazebo data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def external_pose_count_callback(count):
    global external_count
    global namespace

    rospy.logdebug(namespace + ': sensor_record received new external_count value of ' + str(count.data))
    external_count = [count.data]


def make_sure_path_exists(path):
    global namespace

    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            rospy.logwarn(namespace + ': Error when checking file path - ' + e.message)


def write_headers():
    global namespace
    global prefix

    # Make sure our path exists before creating our files
    make_sure_path_exists(prefix)

    if None not in (prefix, namespace):
        rospy.logdebug(namespace + ': Writing headers to data files')
        filename = prefix + namespace + '_continuous_odometry_filtered.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position'])  # , 'pose_covariance', 'twist_covariance'])

        filename = prefix + namespace + '_discrete_odometry_filtered.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position'])  # , 'pose_covariance', 'twist_covariance'])

        filename = prefix + namespace + '_gazebo_odometry_filtered.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position'])  # , 'pose_covariance', 'twist_covariance'])

        filename = prefix + namespace + '_external_pose_count.csv'
        with open(filename, 'w+') as count_file:
            writer = csv.writer(count_file)
            writer.writerow(['count'])
    else:
        if namespace is None:
            rospy.logdebug('Could not write headers because namespace was not initialized')
        else:
            if prefix is None:
                rospy.logdebug(namespace + ': Could not write headers because file prefix was not initialized')


# Must accept event as argument due to use with timer
def write_to_files(event):
    global continuous_data
    global discrete_data
    global gazebo_data
    global external_count
    global namespace
    global prefix

    if None not in (continuous_data, discrete_data, gazebo_data, external_count, namespace, prefix):
        rospy.logdebug(namespace + ': Writing to sensor data to files')

        filename = prefix + namespace + '_continuous_odometry_filtered.csv'
        with open(filename, 'a+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(continuous_data)

        filename = prefix + namespace + '_discrete_odometry_filtered.csv'
        with open(filename, 'a+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(discrete_data)

        filename = prefix + namespace + '_gazebo_odometry_filtered.csv'
        with open(filename, 'a+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(gazebo_data)

        filename = prefix + namespace + '_external_pose_count.csv'
        with open(filename, 'a+') as count_file:
            writer = csv.writer(count_file)
            writer.writerow(external_count)
    else:
        if namespace is None:
            rospy.logdebug('Could not write data because namespace was not initialized')
        else:
            if continuous_data is None:
                rospy.logdebug(namespace + ': Could not write data because continuous data was not initialized')
            if discrete_data is None:
                rospy.logdebug(namespace + ': Could not write data because discrete data was not initialized')
            if gazebo_data is None:
                rospy.logdebug(namespace + ': Could not write data because gazebo data was not initialized')
            if external_count is None:
                rospy.logdebug(namespace + ': Could not write data because external count was not initialized')
            if prefix is None:
                rospy.logdebug(namespace + ': Could not write data because file prefix was not initialized')


def main():
    debug = rospy.get_param('/debug')
    if debug:
        rospy.init_node('sensor_record', log_level=rospy.DEBUG)
    else:
        rospy.init_node('sensor_record')

    global namespace
    namespace = rospy.get_namespace()[1:-1]

    global prefix
    prefix = rospy.get_param('/save_file_prefix')

    initial_position_subscriber = rospy.Subscriber('initial_position', PoseWithCovarianceStamped, initial_position_callback)
    continuous_odom_subscriber = rospy.Subscriber('odometry/filtered_continuous', Odometry, continuous_odom_callback)
    discrete_odom_subscriber = rospy.Subscriber('odometry/filtered_discrete', Odometry, discrete_odom_callback)
    gazebo_odom_subscriber = rospy.Subscriber('odom', Odometry, gazebo_odom_callback)
    external_pose_count_subscriber = rospy.Subscriber('external_poses_count', UInt64, external_pose_count_callback)

    write_headers()

    # Must account for single robot simulation where there are no external poses
    global external_count
    if rospy.get_param('/number_of_robots') == 1:
        external_count = [0]

    timer = rospy.Timer(rospy.Duration(.1), write_to_files)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
