#!/usr/bin/env python

import rospy

from thruster_control.thruster_manager import ThrusterManager
from thruster_control.thruster_interfaces import SimulatedThrusterInterface



if __name__ == "__main__":
    rospy.init_node('sim_thruster_controller')

    thrusters = [SimulatedThrusterInterface(i) for i in range(0, 6)]

    tm = ThrusterManager(thrusters)
    while not rospy.is_shutdown():
        rospy.sleep(1)
