#!/usr/bin/env python

import rospy

from thrust_computer.thruster_configurations import get_configuration
from thrust_computer.thrust_computer import ThrustComputer


if __name__ == "__main__":
    rospy.init_node('thrust_computer')

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    config = get_configuration(model)
    config.initialize()

    thrust_computer = ThrustComputer(config)
    thrust_computer.run()
