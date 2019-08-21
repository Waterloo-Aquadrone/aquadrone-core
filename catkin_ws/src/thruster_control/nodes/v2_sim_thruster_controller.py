#!/usr/bin/env python

import rospy

from thruster_control.thruster_collection_manager import ThrusterCollectionManager
from thruster_control.thruster_interfaces import SimulatedThrusterInterface



if __name__ == "__main__":
    rospy.init_node('sim_thruster_controller')

    thrusters = [SimulatedThrusterInterface('aquadrone_v2', i) for i in range(0, 6)]

    tm = ThrusterCollectionManager(thrusters)
    while not rospy.is_shutdown():
        rospy.sleep(1)
