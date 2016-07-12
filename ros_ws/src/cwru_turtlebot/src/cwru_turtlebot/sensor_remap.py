#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from numpy.random import randn, normal
from helpers import convert_quaternion_to_yaw, convert_yaw_to_quaternion

# publisher for imu, odom
imu_publisher = rospy.Publisher('imu_data_remapped', Imu, queue_size=1)
odom_publisher = rospy.Publisher('odom_remapped', Odometry, queue_size=1)

# get the current namespace
namespace = rospy.get_namespace()[1:]
noisy = False


def imu_remap(imu_msg):
    global namespace
    global noisy

    # remap imu message to base_link_self and publish the new message
    remapped_msg = imu_msg
    remapped_msg.header.frame_id = namespace + 'base_link_filter'

    if noisy:
        # We will add noise to the IMU data
        noise = randn(7, 1)

        orientation = remapped_msg.orientation
        noisy_yaw = convert_quaternion_to_yaw(orientation) + noise[0]
        orientation = convert_yaw_to_quaternion(noisy_yaw)

        angular_velocity = remapped_msg.angular_velocity
        angular_velocity.x += noise[1]
        angular_velocity.y += noise[2]
        angular_velocity.z += noise[3]

        linear_acceleration = remapped_msg.linear_acceleration
        linear_acceleration.x += noise[4]
        linear_acceleration.y += noise[5]
        linear_acceleration.z += noise[6]

    if not rospy.is_shutdown():
        try:
            imu_publisher.publish(remapped_msg)
        except rospy.ROSException as e:
            rospy.logwarn(e.message)


def odom_remap(odom_msg):
    global namespace
    global noisy

    # remap odom message to base_footprint_self and publish the new message
    remapped_msg = odom_msg
    remapped_msg.child_frame_id = namespace + 'base_footprint_filter'

    cumulative_x_err = 0
    cumulative_y_err = 0
    cumulative_z_err = 0
    cumulative_yaw_err = 0

    if noisy:
        noise = normal(0, .01, 4)  # Use very small variance because of how many odometry messages gazebo publishes
        cumulative_x_err += noise[0]
        cumulative_y_err += noise[1]
        cumulative_z_err += noise[2]
        cumulative_yaw_err += noise[3]

        position = remapped_msg.pose.pose.position
        orientation = remapped_msg.pose.pose.orientation

        position.x += cumulative_x_err
        position.y += cumulative_y_err
        position.z += cumulative_z_err

        noisy_yaw = convert_quaternion_to_yaw(orientation) + cumulative_yaw_err
        orientation = convert_yaw_to_quaternion(noisy_yaw)

    if not rospy.is_shutdown():
        try:
            odom_publisher.publish(remapped_msg)
        except rospy.ROSException as e:
            rospy.logwarn(e.message)


def main():
    rospy.wait_for_service('/gazebo/set_physics_properties')

    # initialize ros node for the imu remap process
    debug = rospy.get_param('/debug')
    if debug:
        rospy.init_node('sensor_remap', log_level=rospy.DEBUG)
    else:
        rospy.init_node('sensor_remap')

    global noisy
    noisy = rospy.get_param('/noisy')

    # subscribe to imu and odom from the robot
    imu_subscriber = rospy.Subscriber('mobile_base/sensors/imu_data', Imu, imu_remap)
    odom_subscriber = rospy.Subscriber('odom', Odometry, odom_remap)

    # just run this to pub/sub to the robot
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
