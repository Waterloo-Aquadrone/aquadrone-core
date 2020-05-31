#!/usr/bin/env python

import rospy

from thruster_control.thrust_computer.thruster_configurations import get_configuration
from thruster_control.thrust_computer.thrust_computer import ThrustComputer

"""
This node is responsible for listening to all the relevant sources for thrust commands (each of which is a Wrench)
using a MovementCommandCollector. The aggregated Wrench will then be converted to a list of of thrusts for the 
individual motors (a MotorControls message) and send to the motor_command topic.
"""


if __name__ == "__main__":
    rospy.init_node('thrust_computer')

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    config = get_configuration(model)
    config.initialize()

    thrust_computer = ThrustComputer(config)
    thrust_computer.run()
