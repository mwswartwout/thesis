#!/usr/bin/env python

import rospy
import csv
import os
import errno
from helpers import convert_quaternion_to_yaw, correct_angle

from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

initial_x = None
initial_y = None
initial_yaw = None
continuous_data = None
discrete_data = None
gazebo_data = None
external_count = None
namespace = None
# TODO add protections so that a lack of starting or trailing slashes does not affect prefix and file writing
prefix = None
imu_data = None


def initial_position_callback(position):
    # Record the initial position of the robot so that we can convert the odometry values from the robot's odom frame
    # to the map frame
    global initial_x
    global initial_y
    global initial_yaw

    initial_x = position.pose.pose.position.x
    initial_y = position.pose.pose.position.y
    initial_yaw = convert_quaternion_to_yaw(position.pose.pose.orientation)


def continuous_odom_callback(new_odom):
    global continuous_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y, initial_yaw):
        # Must add in the initial pose values to convert from odom frame to map frame
        pose_x = new_odom.pose.pose.position.x + initial_x
        pose_y = new_odom.pose.pose.position.y + initial_y
        pose_yaw = correct_angle(convert_quaternion_to_yaw(new_odom.pose.pose.orientation) + initial_yaw)
        # pose_covariance = new_odom.pose.covariance
        # twist_covariance = new_odom.twist.covariance
        # TODO see if we can split covariance values into their own items
        continuous_data = (pose_x, pose_y, pose_yaw)#, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]
        # rospy.logdebug(namespace + ': sensor_record received new continuous data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def discrete_odom_callback(new_odom):
    global discrete_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y, initial_yaw):
        # Measurement is already in map frame so no need to convert by adding initial position
        pose_x = new_odom.pose.pose.position.x
        pose_y = new_odom.pose.pose.position.y
        pose_yaw = correct_angle(convert_quaternion_to_yaw(new_odom.pose.pose.orientation))
        # pose_covariance = new_odom.pose.covariance
        # twist_covariance = new_odom.twist.covariance
        discrete_data = (pose_x, pose_y, pose_yaw)#, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]
        # rospy.logdebug(namespace + ': sensor_record received new discrete data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def gazebo_odom_callback(new_odom):
    global gazebo_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y, initial_yaw):
        # Must add in the initial pose values to convert from odom frame to map frame
        pose_x = new_odom.pose.pose.position.x + initial_x
        pose_y = new_odom.pose.pose.position.y + initial_y
        pose_yaw = correct_angle(convert_quaternion_to_yaw(new_odom.pose.pose.orientation) + initial_yaw)
        # pose_covariance = new_odom.pose.covariance
        # twist_covariance = new_odom.twist.covariance
        gazebo_data = (pose_x, pose_y, pose_yaw)#, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]
        # rospy.logdebug(namespace + ': sensor_record received new gazebo data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def external_pose_count_callback(count):
    global external_count
    global namespace

    rospy.logdebug(namespace + ': sensor_record received new external_count value of ' + str(count.data))
    external_count = [count.data]


def imu_callback(imu_msg):
    global imu_data

    yaw = convert_quaternion_to_yaw(imu_msg.orientation)
    velocity = (imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)
    acceleration = (imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z)

    imu_data = (yaw, velocity[0], velocity[1], velocity[2], acceleration[0], acceleration[1], acceleration[2])


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
        # rospy.logdebug(namespace + ': Writing headers to data files')
        filename = prefix + namespace + '_continuous_odometry_filtered.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position', 'yaw'])  # , 'pose_covariance', 'twist_covariance'])

        filename = prefix + namespace + '_discrete_odometry_filtered.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position', 'yaw'])  # , 'pose_covariance', 'twist_covariance'])

        filename = prefix + namespace + '_gazebo_odometry_filtered.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position', 'yaw'])  # , 'pose_covariance', 'twist_covariance'])

        filename = prefix + namespace + '_external_pose_count.csv'
        with open(filename, 'w+') as count_file:
            writer = csv.writer(count_file)
            writer.writerow(['count'])

        filename = prefix + namespace + '_raw_imu_data.csv'
        with open(filename, 'w+') as imu_file:
            writer = csv.writer(imu_file)
            writer.writerow(['yaw', 'x_vel', 'y_vel', 'z_vel', 'x_acc', 'y_acc', 'z_acc'])
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
    global imu_data
    global namespace
    global prefix

    if None not in (continuous_data, discrete_data, gazebo_data, external_count, imu_data, namespace, prefix):
        # rospy.logdebug(namespace + ': Writing to sensor data to files')

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

        filename = prefix + namespace + '_raw_imu_data.csv'
        with open(filename, 'a+') as imu_file:
            writer = csv.writer(imu_file)
            writer.writerow(imu_data)
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
    rospy.wait_for_service('/gazebo/set_physics_properties')

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
    imu_subscriber = rospy.Subscriber('imu_data_remapped', Imu, imu_callback)
    write_headers()

    # Must account for single robot simulation where there are no external poses
    global external_count
    if rospy.get_param('/number_of_robots') == 1:
        external_count = [0]

    rospy.sleep(rospy.Duration(2))  # Wait 2 seconds for filters to successfully localize before recording

    timer = rospy.Timer(rospy.Duration(.1), write_to_files)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
