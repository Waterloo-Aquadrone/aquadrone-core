#!/usr/bin/env python

import rospy

from thruster_control.thruster_manager import ThrusterManager
from thruster_control.thruster_interfaces import SimulatedThrusterInterface



if __name__ == "__main__":
    tm = ThrusterManager(SimulatedThrusterInterface)
    rospy.init_node('sim_thruster_controller')
    while not rospy.is_shutdown():
        rospy.sleep(1)
