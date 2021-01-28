#!/usr/bin/env python

import rospy
import traceback

from thruster_control.thrust_computer.thruster_configurations import ThrusterConfiguration
from thruster_control.thrust_distributor.faulty_thrust_distributor import FaultyThrustDistributor




"""
This class is responsible for listening to the motor_command topic where the thrusts for all the thrusters are published
and distributing the individual thrusts to all the individual thrusters' topics. The topic names are used by both Gazebo
and the real thrusters. This node is necessary, because sometimes the ThrustComputer node may be bypassed
and controls may be directly sent here.
"""


if __name__ == "__main__":
    rospy.init_node('faulty_thrust_distributor_node')

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    config = ThrusterConfiguration(model)
    config.initialize()
    num = config.get_num_thrusters()

    distributor = FaultyThrustDistributor(num, namespace="aquadrone")
    distributor.run()
