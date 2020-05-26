#!/usr/bin/env python

import rospy
from worldP_ekf import EKF


if __name__ == "__main__":
    rospy.init_node("world_state_estimation")

    ekf = EKF()
    ekf.run()
