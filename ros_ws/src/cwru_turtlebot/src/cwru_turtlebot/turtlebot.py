#!/usr/bin/env python

import math
import numpy
import rospy
import copy
# Action server imports
# TODO need to switch from SimpleActionServer to ActionServer so that new goals don't preempt old ones

import actionlib
import tf
from cwru_turtlebot.msg import ExternalPoseAction, ExternalPoseGoal, ExternalPoseResult

# Publish/Subscribe type imports
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from cwru_turtlebot.msg import ScanWithVariance, ScanWithVarianceStamped
from std_msgs.msg import UInt64

# Service type imports
from robot_localization.srv import SetPose, SetPoseRequest


# Base class for all TurtleBots
class TurtleBot:

    def __init__(self, rate=10):
        debug = rospy.get_param('/debug')
        if debug:
            rospy.init_node('robot', log_level=rospy.DEBUG)
        else:
            rospy.init_node('robot')

        # Create initial pose object from parameter server
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.pose.pose.position.x = rospy.get_param('x_pos')
        self.initial_pose.pose.pose.position.y = rospy.get_param('y_pos')
        self.initial_pose.pose.pose.orientation = self.convert_yaw_to_quaternion(rospy.get_param('yaw'))
        self.initial_pose.header.frame_id = 'map'

        self.current_continuous_pose = copy.deepcopy(self.initial_pose)

        self.initialize_subscribers()
        self.initialize_publishers()

        self.external_pose_publisher.publish()
        rospy.set_param('server_started', False)

        self.initialize_action_servers()

        self.position_publisher.publish(self.initial_pose)

        self.scan_received = False  # We haven't received a valid LaserScan yet
        self.most_recent_scan = None
        self.lidar_alarm = False

        self.rate_frequency = rate
        self.rate = rospy.Rate(rate)

        self.namespace = rospy.get_namespace()[1:]  # Get rid of the leading /

        self.client_list = []  # List to hold all of our action clients
        self.existing_clients = []  # List to hold names of all action clients in the client_list
        self.external_pose_count = 0  # Number of external poses received

        self.initialize_action_clients()

    def initialize_subscribers(self):
        self.lidar_subscriber = rospy.Subscriber('scan',
                                                 LaserScan,
                                                 self.scan_callback)

        self.odom_subscriber = rospy.Subscriber('odometry/filtered_continuous',
                                                Odometry,
                                                self.continuous_odom_callback)

    def initialize_publishers(self):
        # TODO do we really need this anymore?
        self.processed_scan_publisher = rospy.Publisher('processed_scan',
                                                        ScanWithVarianceStamped,
                                                        queue_size=1)

        # TODO do we really need this anymore?
        self.position_publisher = rospy.Publisher('position',
                                                  PoseWithCovarianceStamped,
                                                  queue_size=1,
                                                  latch=True)

        # Publisher for poses received from other robots, used as input to UKF
        self.external_pose_publisher = rospy.Publisher('external_poses',
                                                       PoseWithCovarianceStamped,
                                                       queue_size=10)

        self.external_pose_count_publisher = rospy.Publisher('external_poses_count',
                                                             UInt64,
                                                             queue_size=1)

    def initialize_action_servers(self):
        self.external_pose_as = actionlib.SimpleActionServer('external_pose_action',
                                                             ExternalPoseAction,
                                                             execute_cb=self.external_pose_cb,
                                                             auto_start=False)
        self.external_pose_as.start()
        rospy.set_param('server_started', True)

    def initialize_action_clients(self):
        self.update_client_list()

    def update_client_list(self):
        param_list = rospy.get_param_names()  # Get list of strings with all parameter names on the parameter server
        for param in param_list:
            if (param.endswith("server_started") and
                    not param.startswith(rospy.get_namespace()) and
                    rospy.get_param(param) is True):
                # Parameter indicates action server exists, is a different robot, and has been started
                server_name = param.replace("server_started", "external_pose_action")
                if server_name not in self.existing_clients:
                    # This server has not previously been added so let's make a new client for it
                    new_client = actionlib.SimpleActionClient(server_name, ExternalPoseAction)
                    self.client_list.append(new_client)
                    self.existing_clients.append(server_name)

        rospy.logdebug("Updated client list, it now contains %d robots", len(self.existing_clients))

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
            scan.std_dev = math.sqrt(scan.variance)  # standard deviation is square root of variance
            scan.std_error = scan.std_dev / math.sqrt(len(valid_particles))
            scan.valid = True
        else:
            # otherwise if there are no valid particles, set all values to 0
            scan.min = 0
            scan.max = 0
            scan.mean = 0
            scan.variance = 0
            scan.std_dev = 0
            scan.median = 0
            scan.std_error = 0
            scan.valid = False

        # Check whether we need to activate lidar alarm
        if 0 < scan.median < 0.5:
            self.lidar_alarm = True
        else:
            self.lidar_alarm = False

        processed_scan = self.stamp_scan_w_variance(scan)

        if not rospy.is_shutdown():
            try:
                self.processed_scan_publisher.publish(processed_scan)
            except rospy.ROSException as e:
                rospy.logwarn('Unable to publish most recent processed scan')
                rospy.logwarn(e.message)

        self.most_recent_scan = processed_scan
        self.send_scan_to_clients(processed_scan)

    def continuous_odom_callback(self, odom):
        # TODO convert this to using a map to generate a map -> odom transform rather than just consulting initial pose
        self.current_continuous_pose.pose.pose.position.x = odom.pose.pose.position.x + self.initial_pose.pose.pose.position.x
        self.current_continuous_pose.pose.pose.position.y = odom.pose.pose.position.y + self.initial_pose.pose.pose.position.y

        odom_yaw = self.convert_quaternion_to_yaw(odom.pose.pose.orientation)
        initial_yaw = self.convert_quaternion_to_yaw(self.initial_pose.pose.pose.orientation)
        current_yaw = odom_yaw + initial_yaw
        self.current_continuous_pose.pose.pose.orientation = self.convert_yaw_to_quaternion(current_yaw)

    def send_scan_to_clients(self, scan):
        pose = self.convert_scan_to_pose(scan)
        self.update_client_list()

        while not rospy.is_shutdown():
            for client in self.client_list:
                client.wait_for_server()
                goal = ExternalPoseGoal(pose)
                client.send_goal(goal)
                client.wait_for_result()
                success = client.get_result()
                if not success:
                    rospy.logwarn("A client returned unsucessfully when sent a pose measurement")

    def convert_scan_to_pose(self, scan):
        pose = None
        if scan.scan.valid is True:
            # Scan is valid so we can create a pose from it
            pose = PoseWithCovarianceStamped()
            pose.pose.pose.position.x = self.current_continuous_pose.pose.pose.position.x + scan.scan.median * math.cos(self.convert_quaternion_to_yaw(self.current_continuous_pose.pose.pose.orientation))
            pose.pose.pose.position.y = self.current_continuous_pose.pose.pose.position.y + scan.scan.median * math.sin(self.convert_quaternion_to_yaw(self.current_continuous_pose.pose.pose.orientation))

            # Right now use default quaternion since we can't figure out orientation from our scans
            pose.pose.pose.orientation.x = 0
            pose.pose.pose.orientation.y = 0
            pose.pose.pose.orientation.z = 0
            pose.pose.pose.orientation.w = 1
            pose.header.frame_id = "map"
            pose.header.stamp = scan.header.stamp

            # TODO figure out covariances for these poses
            pose.pose.covariance[0] = scan.scan.variance
            pose.pose.covariance[7] = scan.scan.variance
        return pose

    def external_pose_cb(self, goal):
        if not rospy.is_shutdown():
            try:
                self.external_pose_publisher.publish(goal.pose)
                self.external_pose_count += 1
                self.external_pose_count_publisher.publish(UInt64(data=self.external_pose_count))
            except rospy.ROSException as e:
                rospy.logwarn('Unable to publish most recent external pose')
                rospy.logwarn(e.message)

        result = ExternalPoseResult()
        result.success = True
        self.external_pose_as.set_succeeded(result)

    def wait_for_clients(self):
        num_clients = rospy.get_param('/number_of_robots')
        complete = False
        while not rospy.is_shutdown() and not complete:
            if num_clients - 1 != len(self.existing_clients):
                rospy.logdebug('Waiting for other robots to come online...')  # + str(num_clients) + ' - 1 != ' + str(len(self.existing_clients)))
                self.rate.sleep()
            else:
                complete = True

    @staticmethod
    def stamp_scan_w_variance(scan_w_variance):
        # Create time-stamped scan message including the scan and variance of points
        stamped_scan_w_variance = ScanWithVarianceStamped()
        stamped_scan_w_variance.scan = scan_w_variance
        stamped_scan_w_variance.header.stamp = rospy.get_rostime()

        return stamped_scan_w_variance

    @staticmethod
    def convert_quaternion_to_yaw(quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        yaw = math.atan2(2*(x*y + z*w), w**2 - z**2 - y**2 + x**2)
        return yaw

    @staticmethod
    def convert_yaw_to_quaternion(yaw):
        # Assumes roll/pitch = 0 always
        quaternion = Quaternion()
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = math.sin(yaw / 2)
        quaternion.w = math.cos(yaw / 2)

        return quaternion

    @staticmethod
    def reset_filters():
        rospy.logdebug('Resetting filters...')
        error = False
        # First reset the continuous filter
        rospy.wait_for_service('set_pose_continuous')
        try:
            continuous_service = rospy.ServiceProxy('set_pose_continuous', SetPose)
            request = SetPoseRequest()
            continuous_service(request)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s", e)
            error = True

        # Next reset the discrete filter
        rospy.wait_for_service('set_pose_discrete')
        try:
            continuous_service = rospy.ServiceProxy('set_pose_discrete', SetPose)
            request = SetPoseRequest()
            continuous_service(request)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s", e)
            error = True

        if not error:
            rospy.logdebug('Filter reset complete')
        else:
            rospy.logwarn('Filter reset encountered errors')