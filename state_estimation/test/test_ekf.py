#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

from state_estimation.ekf import EKF
from thruster_control.thrust_computer.thruster_configurations import V28Configuration


class TestEKF(unittest.TestCase):

    def test_contruction(self):
        ekf = EKF(V28Configuration())


if __name__ == '__main__':
    rospy.init_node('test_depth_control')
    rostest.rosrun('state_estimation', 'test_ekf', TestEKF)
