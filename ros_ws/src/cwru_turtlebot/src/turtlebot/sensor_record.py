#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Odometry

def odom_callback(new_odom):
    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_covariance = new_odom.pose.covariance
    twist_covariance = new_odom.twist.covariance
    data = [pose_x, pose_y, (''.join(str(pose_covariance))).split(","), (''.join(str(twist_covariance))).split(",")]

    with open('odometry_filtered.csv', 'a') as pose_file:
        writer = csv.writer(pose_file)
        writer.writerow(data)

    # twist_covariance = new_odom.twist.covariance
    # with open('odometry_filtered_twist_modified.csv', 'a') as twist_file:
    #    writer = csv.writer(twist_file)
    #    writer.writerow(twist_covariance)

if __name__ == '__main__':
    try:
        rospy.init_node('sensor_record')
        odom_subscriber = rospy.Subscriber('odometry/filtered', Odometry, odom_callback)

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass