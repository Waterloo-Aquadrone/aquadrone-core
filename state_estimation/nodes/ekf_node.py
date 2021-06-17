#!/usr/bin/env python3.8

import rospy

from thruster_control.thrust_computer.thruster_configurations import ThrusterConfiguration
from state_estimation.ekf import EKF


if __name__ == "__main__":
    rospy.init_node("ekf_state_estimation")

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    config = ThrusterConfiguration(model)
    config.initialize()

    ekf = EKF(config)
    ekf.run()
