#!/usr/bin/env python3.8

import rospy
import rostest
import unittest
import numpy as np
from mock import MagicMock

from thruster_control.thrust_computer.thrust_computer import ThrustComputer


class TestThrustComputer(unittest.TestCase):
    def test_construction(self):
        c = ThrustComputer(None, None)

    def test_thrusts_optimization(self):
        thrust_one = np.array([15, 15, 15, 15, 15, 15, 15, 15])
        thrust_two = np.array([20, 1, 1, 1, 1, 1, 1, 1])
        thrust_three = np.array([1.9, 1, 1, 1, 1, 1, 1, 1])

        thrusts = [thrust_one, thrust_two, thrust_three]
        final_thrusts = ThrustComputer.optimize_thrusts(thrusts)
        self.assertTrue(np.all(np.isclose(final_thrusts,
                                          [23.13074398, 16.31153721, 16.31153721, 16.31153721,
                                           16.31153721, 16.31153721, 16.31153721, 16.31153721])))


if __name__ == '__main__':
    rospy.init_node('test_thrust_computer')
    rostest.rosrun('thruster_control', 'test_thrust_computer', TestThrustComputer)
