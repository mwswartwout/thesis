import rospy
import math
from geometry_msgs.msg import Quaternion


def get_curr_time():
    return rospy.get_time()


def scan_has_none_check(scan):
    # checks if the scan as a none value in its contents, returns True if so
    scan = [scan.min, scan.max, scan.mean, scan.variance, scan.std_dev, scan.median, scan.std_error]
    for item in scan:
        if item is None:
            return True
    return False


def convert_quaternion_to_yaw(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    yaw = math.atan2(2*(x*y + z*w), w**2 - z**2 - y**2 + x**2)
    return yaw


def convert_yaw_to_quaternion(yaw):
    # Assumes roll/pitch = 0 always since we're operating in 2D
    quaternion = Quaternion()
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = math.sin(yaw / 2)
    quaternion.w = math.cos(yaw / 2)

    return quaternion


def correct_angle(yaw):
    # Correct yaws to smallest possible value, accounting for periodicity
    while yaw > 2 * math.pi and not rospy.is_shutdown():
        yaw -= 2 * math.pi

    while yaw < -2 * math.pi and not rospy.is_shutdown():
        yaw += 2 * math.pi

    # Now let's always return a positive yaw just for continuity's sake
    # if yaw < 0:
    #   yaw += 2 * math.pi

    assert -1 * math.pi < yaw < math.pi

    # TODO add in calculation to rotate in shortest direction
    # Make sure we're using the shortest rotation
    #if yaw > math.pi:
    #    yaw = -1 * (2*math.pi - yaw)

    return yaw


def wait_for_services():
    # TODO add logic here to timeout and raise an error while waiting
    rospy.loginfo('Waiting for services for TurtleBot initialization...')
    # Wait for gazebo and filters to be fully initialized before starting our robot
    rospy.wait_for_service('/gazebo/set_physics_properties')
    rospy.wait_for_service('set_pose_continuous')
    rospy.wait_for_service('set_pose_discrete')
    rospy.loginfo('All required services are active')