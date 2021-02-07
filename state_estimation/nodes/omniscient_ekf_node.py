#!/usr/bin/env python3

import rospy
from state_estimation.omniscient_ekf import OmniscientEKF


if __name__ == "__main__":
    rospy.init_node("omniscient_ekf_state_estimation")

    ekf = OmniscientEKF()
    ekf.run()
