import rospy
import math


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
