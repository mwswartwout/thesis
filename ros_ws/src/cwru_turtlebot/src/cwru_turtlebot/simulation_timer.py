#!/usr/bin/env python
import rospy


# Use this node (with required attribute set to true) to control the simulation runtime
def main():
    rospy.init_node('simulation_timer')
    rospy.sleep(rospy.get_param('sim_time'))

if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        pass
