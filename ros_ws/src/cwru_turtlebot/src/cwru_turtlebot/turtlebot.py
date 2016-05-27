#!/usr/bin/env python

import math
import numpy
import rospy

# Action server imports
import actionlib
from cwru_turtlebot.msg import ExternalPoseAction, ExternalPoseResult

#Publish/Subscribe type imports
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from cwru_turtlebot.msg import ScanWithVariance, ScanWithVarianceStamped


# Base class for all TurtleBots
class TurtleBot:

    def __init__(self, rate=10):
        rospy.init_node('robot')

        # Create initial pose object from parameter server
        self.initial_pose = Pose2D()
        self.initial_pose.x = rospy.get_param('x_pos')
        self.initial_pose.y = rospy.get_param('y_pos')
        self.initial_pose.theta = rospy.get_param('yaw')

        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_action_servers()

        self.position_publisher.publish(self.initial_pose)

        self.scan_received = False  # We haven't received a valid LaserScan yet
        self.most_recent_scan = None

        self.rate = rospy.Rate(rate)

        self.namespace = rospy.get_namespace()[1:]  # Get rid of the leading /

    def initialize_subscribers(self):
        self.lidar_subscriber = rospy.Subscriber('scan',
                                                 LaserScan,
                                                 self.scan_callback)


    def initialize_publishers(self):
        self.processed_scan_publisher = rospy.Publisher('processed_scan',
                                                        ScanWithVarianceStamped,
                                                        queue_size=1)

        self.position_publisher = rospy.Publisher('position',
                                                  Pose2D,
                                                  queue_size=1,
                                                  latch=True)

        self.external_pose_publisher = rospy.Publisher('external_poses',
                                                       PoseWithCovarianceStamped,
                                                       queue_size=10)

    def initialize_action_servers(self):
        self.external_pose_as = actionlib.SimpleActionServer('external_pose_action',
                                                             ExternalPoseAction,
                                                             execute_cb=self.external_pose_cb,
                                                             auto_start=False)
        self.external_pose_as.start()

    def initialize_scanner(self, scan_msg):
        # Scan properties
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        self.time_increment = scan_msg.time_increment
        self.scan_time = scan_msg.scan_time
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max

        # Don't reinitialize scanner multiple times
        self.scan_received = True

    def scan_callback(self, scan_msg):
        # initialize scanner properties for this robot
        if not self.scan_received:
            self.initialize_scanner(scan_msg)

        # First throw out invalid particles
        valid_particles = []
        # ranges = numpy.array(scan_msg.ranges)
        for particle in scan_msg.ranges:
            if self.range_min <= particle <= self.range_max:
                valid_particles.append(particle)

        scan = ScanWithVariance()

        # Find the min, max, mean, median, variance, std dev, and std error if there are particles
        if len(valid_particles) is not 0:
            scan.min = numpy.min(valid_particles)
            scan.max = numpy.max(valid_particles)
            scan.mean = numpy.mean(valid_particles)
            scan.median = numpy.median(valid_particles)
            scan.variance = numpy.var(valid_particles)
            scan.std_dev = math.sqrt(scan.variance) # standard deviation is square root of variance
            scan.std_error = scan.std_dev / math.sqrt(len(valid_particles))
        else:
            # otherwise if there are no valid particles, set all values to 0
            scan.min = 0
            scan.max = 0
            scan.mean = 0
            scan.variance = 0
            scan.std_dev = 0
            scan.median = 0
            scan.std_error = 0

        processed_scan = self.stamp_scan_w_variance(scan)
        self.processed_scan_publisher.publish(processed_scan)

        self.most_recent_scan = processed_scan

    def external_pose_cb(self, goal):
        self.external_pose_publisher.publish(goal)

        result = ExternalPoseResult()
        result.success = True
        self.external_pose_as.set_succeeded(result)

    @staticmethod
    def stamp_scan_w_variance(scan_w_variance):
        # Create time-stamped scan message including the scan and variance of points
        stamped_scan_w_variance = ScanWithVarianceStamped()
        stamped_scan_w_variance.scan = scan_w_variance
        stamped_scan_w_variance.header.stamp = rospy.get_rostime()

        return stamped_scan_w_variance
