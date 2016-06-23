#!/usr/bin/env python
import rospy


# Use this node (with required attribute set to true) to control the simulation runtime
def main():
    rospy.init_node('simulation_timer')
    sim_time = rospy.get_param('sim_time')
    interval = float(sim_time) / 10
    timer = rospy.Duration(interval)
    for i in range(0, 10):
        count = 10 * i
        rospy.loginfo("Simulation is " + str(count) + "% complete...")
        rospy.sleep(timer)


if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        pass
