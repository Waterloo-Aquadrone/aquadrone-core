#!/usr/bin/env python3

import rospy
import math

from state_estimation.ekf import EKF
from thruster_control.thrust_computer.thruster_configurations import V1Configuration, V2Configuration, V28Configuration


if __name__ == "__main__":
    rospy.init_node("ekf_state_estimation")


    # Assume V28 by default
    model = rospy.get_param("model", "v28")

    if model == "v1":
        config = V1Configuration()
        num = 6
    elif model == "v2":
        config = V1Configuration()
        num = 6
    elif model == "v28":
        config = V28Configuration()
        num = 8
    else:
        print("Error: unknown model for controls: %s" % str(model))
        exit()

    config.initialize()
    ekf = EKF(config)
    ekf.run()