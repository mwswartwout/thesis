#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Odometry


def continuous_odom_callback(new_odom):
    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_covariance = new_odom.pose.covariance
    twist_covariance = new_odom.twist.covariance
    data = [pose_x, pose_y, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]

    with open('continuous_odometry_filtered.csv', 'a') as pose_file:
        writer = csv.writer(pose_file)
        writer.writerow(data)

    # twist_covariance = new_odom.twist.covariance
    # with open('odometry_filtered_twist_modified.csv', 'a') as twist_file:
    #    writer = csv.writer(twist_file)
    #    writer.writerow(twist_covariance)


def discrete_odom_callback(new_odom):
    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_covariance = new_odom.pose.covariance
    twist_covariance = new_odom.twist.covariance
    data = [pose_x, pose_y, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]

    with open('discrete_odometry_filtered.csv', 'a') as pose_file:
        writer = csv.writer(pose_file)
        writer.writerow(data)

        # twist_covariance = new_odom.twist.covariance
        # with open('odometry_filtered_twist_modified.csv', 'a') as twist_file:
        #    writer = csv.writer(twist_file)
        #    writer.writerow(twist_covariance)


def gazebo_odom_callback(new_odom):
    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_covariance = new_odom.pose.covariance
    twist_covariance = new_odom.twist.covariance
    data = [pose_x, pose_y, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]

    with open('discrete_odometry_filtered.csv', 'a') as pose_file:
        writer = csv.writer(pose_file)
        writer.writerow(data)

 def main():
     rospy.init_node('sensor_record')
     continuous_odom_subscriber = rospy.Subscriber('odometry/filtered_continuous', Odometry, continuous_odom_callback)
     discrete_odom_subscriber = rospy.Subscriber('odometry/filtered_discrete', Odometry, discrete_odom_callback)
     gazebo_odom_subscriber = rospy.Subscriber('odom', Odometry, gazebo_odom_callback)
     while not rospy.is_shutdown():
         rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
