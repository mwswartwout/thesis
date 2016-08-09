#!/usr/bin/env python

import rospy
import math
import helpers
import copy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from numpy.random import randn, normal
from helpers import convert_quaternion_to_yaw, convert_yaw_to_quaternion
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped

# get the current namespace
namespace = None
noisy = False
previous_pose = None
previous_odom = None
previous_time = None
initial_pose = None
imu_publisher = None
odom_publisher = None
noisy_odom_publisher = None

# def initial_position_callback(position):
#     # Record the initial position of the robot so that we can convert the odometry values from the robot's odom frame
#     # to the map frame
#     global initial_pose
#
#     initial_pose = Pose2D()
#     initial_pose.x = position.pose.pose.position.x
#     initial_pose.y = position.pose.pose.position.y
#     initial_pose.theta = convert_quaternion_to_yaw(position.pose.pose.orientation)


def imu_remap(imu_msg):
    global namespace
    # global noisy

    # remap imu message to base_link_self and publish the new message
    remapped_msg = imu_msg
    remapped_msg.header.frame_id = namespace + 'base_link_filter'

    # if noisy:
    #     # We will add noise to the IMU data
    #     noise = randn(7, 1)
    #
    #     orientation = remapped_msg.orientation
    #     noisy_yaw = convert_quaternion_to_yaw(orientation) + noise[0]
    #     orientation = convert_yaw_to_quaternion(noisy_yaw)
    #
    #     angular_velocity = remapped_msg.angular_velocity
    #     angular_velocity.x += noise[1]
    #     angular_velocity.y += noise[2]
    #     angular_velocity.z += noise[3]
    #
    #     linear_acceleration = remapped_msg.linear_acceleration
    #     linear_acceleration.x += noise[4]
    #     linear_acceleration.y += noise[5]
    #     linear_acceleration.z += noise[6]

    try:
        imu_publisher.publish(remapped_msg)
    except rospy.ROSException as e:
        rospy.logwarn(e.message)


def odom_remap(odom_msg):
    global namespace

    # remap odom message to base_footprint_self and publish the new message
    remapped_msg = odom_msg
    remapped_msg.child_frame_id = namespace + 'base_footprint_filter'

    if not rospy.is_shutdown():
        try:
            odom_publisher.publish(remapped_msg)
        except rospy.ROSException as e:
            rospy.logwarn(e.message)


def noisy_odom_remap(odom_msg):
    global namespace
    global previous_pose
    global previous_odom
    global previous_time

    # Odometry model taken from Probabilistic Robotics by Thrun et al.
    # Algorithm used is sample_motion_model_odometry from Table 5.6

    # Robot specific noise parameters
    # Default value = 0.02
    alpha_1 = 0.02
    alpha_2 = 0.02
    alpha_3 = 0.02
    alpha_4 = 0.02

    # Make our received odom reading more palatable
    current_odom = Pose2D()
    current_odom.x = odom_msg.pose.pose.position.x
    current_odom.y = odom_msg.pose.pose.position.y
    current_odom.theta = convert_quaternion_to_yaw(odom_msg.pose.pose.orientation)

    current_time = odom_msg.header.stamp

    if previous_time is None:
        previous_time = copy.deepcopy(current_time)

    # If we have no previous_pose, set to (0, 0, 0)
    if previous_pose is None:
        previous_pose = Pose2D()
        previous_pose.x = 0
        previous_pose.y = 0
        previous_pose.theta = 0

    # Create the previous_odom Pose2D if it doesn't exist, also (0, 0, 0)
    if previous_odom is None:
        previous_odom = Pose2D()
        previous_odom.x = 0
        previous_odom.y = 0
        previous_odom.theta = 0

    # Split movement into three distinct actions, rotate -> translate -> rotate
    delta_rotation_1 = helpers.correct_angle(math.atan2(current_odom.y - previous_odom.y, current_odom.x - previous_odom.x) - previous_odom.theta)
    delta_translation = math.sqrt((previous_odom.x - current_odom.x) ** 2 + (previous_odom.y - current_odom.y) ** 2)
    delta_rotation_2 = helpers.correct_angle(current_odom.theta - previous_odom.theta - delta_rotation_1)

    # Create random pose from given movements by adding noise
    std_dev_1 = alpha_1 * abs(delta_rotation_1) + alpha_2 * delta_translation
    delta_rotation_1_hat = delta_rotation_1 - normal(scale=std_dev_1)

    std_dev_2 = alpha_3 * delta_translation + alpha_4 * (abs(delta_rotation_1) + abs(delta_rotation_2))
    delta_translation_hat = delta_translation - normal(scale=std_dev_2)

    std_dev_3 = alpha_1 * abs(delta_rotation_2) + alpha_2 * delta_translation
    delta_rotation_2_hat = delta_rotation_2 - normal(scale=std_dev_3)

    noisy_odom = Odometry()
    noisy_odom.header = odom_msg.header
    noisy_odom.child_frame_id = namespace + 'base_footprint_filter'

    noisy_odom.pose.pose.position.x = previous_pose.x + delta_translation_hat * math.cos(previous_pose.theta + delta_rotation_1_hat)
    noisy_odom.pose.pose.position.y = previous_pose.y + delta_translation_hat * math.sin(previous_pose.theta + delta_rotation_1_hat)
    noisy_odom_yaw = previous_pose.theta + delta_rotation_1_hat + delta_rotation_2_hat
    noisy_odom.pose.pose.orientation = convert_yaw_to_quaternion(noisy_odom_yaw)

    delta_time = current_time - previous_time
    delta_time_seconds = delta_time.to_sec()
    assert delta_time_seconds >= 0

    # We will assume that our robot moves perfectly with only linear x velocity and angular z velocity
    # This is an okay assumption because we ignore other data anyways because all readings are noise and not valid
    noisy_odom.twist.twist.linear.y = 0
    noisy_odom.twist.twist.linear.z = 0
    noisy_odom.twist.twist.angular.x = 0
    noisy_odom.twist.twist.angular.y = 0

    if delta_time_seconds > 0:
        noisy_odom.twist.twist.linear.x = delta_translation_hat / delta_time_seconds
        noisy_odom.twist.twist.angular.z = (noisy_odom_yaw - previous_pose.theta) / delta_time_seconds
    else:
        noisy_odom.twist.twist.linear.x = 0
        noisy_odom.twist.twist.angular.z = 0

    # From documentation on nav_msgs/Odometry:
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    noisy_odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,  # x variance taken from Gazebo odometry
                                  0, 0.1, 0, 0, 0, 0,  # y variance taken from Gazebo odometry
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0.05]  # yaw variance taken from Gazebo odometry
    # Twist covariance copied from position covariance TODO this definitely wrong
    noisy_odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,  # x_vel variance
                                   0, 0.1, 0, 0, 0, 0,  # y_vel variance
                                   0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0.05]  # yaw_vel variance

    # Publish noisy odom message
    try:
        rospy.logdebug('Publishing noisy odom message')
        noisy_odom_publisher.publish(noisy_odom)
    except rospy.ROSException as e:
        rospy.logwarn(e.message)

    rospy.logdebug('Calculated new noisy odom pose. Previous pose was ' + str(previous_pose) + ',\n previous odom was ' +
                  str(previous_odom) + ',\n current odom was ' + str(current_odom) + ',\n and new noisy pose is ' +
                  str(noisy_odom))
    rospy.logdebug('d_rot_1 noise was ' + str(delta_rotation_1_hat - delta_rotation_1) + '\n d_trans noise was ' +
                  str(delta_translation_hat - delta_translation) + '\n d_rot_2 noise was ' +
                  str(delta_rotation_2_hat - delta_rotation_2))

    # Increment previous pose for use with next message
    previous_pose.x = noisy_odom.pose.pose.position.x
    previous_pose.y = noisy_odom.pose.pose.position.y
    previous_pose.theta = convert_quaternion_to_yaw(noisy_odom.pose.pose.orientation)

    # Increment previous odom for use with next message
    previous_odom.x = current_odom.x
    previous_odom.y = current_odom.y
    previous_odom.theta = current_odom.theta

    # Increment time
    previous_time = current_time


def main():
    debug = rospy.get_param('/debug')
    if debug:
        rospy.init_node('sensor_remap', log_level=rospy.DEBUG)
    else:
        rospy.init_node('sensor_remap')

    helpers.wait_for_services()

    global namespace
    namespace = rospy.get_namespace()[1:]

    # Set up publishers
    global imu_publisher
    global odom_publisher
    global noisy_odom_publisher
    imu_publisher = rospy.Publisher('imu_data_remapped', Imu, queue_size=1)
    odom_publisher = rospy.Publisher('odom_remapped', Odometry, queue_size=1)
    noisy_odom_publisher = rospy.Publisher('noisy_odom_remapped', Odometry, queue_size=1)

    # subscribe to imu and odom from the robot
    imu_subscriber = rospy.Subscriber('mobile_base/sensors/imu_data', Imu, imu_remap)
    odom_subscriber = rospy.Subscriber('odom_throttle', Odometry, odom_remap)
    noisy_odom_subscriber = rospy.Subscriber('odom_throttle', Odometry, noisy_odom_remap)

    # Get the initial position of the robot
    # initial_position_subscriber = rospy.Subscriber('initial_position', PoseWithCovarianceStamped, initial_position_callback)

    # just run this to pub/sub to the robot
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
