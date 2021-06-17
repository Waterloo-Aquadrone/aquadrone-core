#!/usr/bin/env python3.8

import rospy
import rostest
import unittest
from mock import MagicMock

from state_estimation.ekf import EKF
from thruster_control.thrust_computer.thruster_configurations import ThrusterConfiguration


class TestEKF(unittest.TestCase):

    def test_contruction(self):
        config = ThrusterConfiguration('v28')
        config.initialize()
        ekf = EKF(config)
        ekf.get_state_msg()

if __name__ == '__main__':
    rospy.init_node('test_depth_control')
    rostest.rosrun('state_estimation', 'test_ekf', TestEKF)
