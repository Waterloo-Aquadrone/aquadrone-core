#!/usr/bin/env python3

import rospy
import rostest
import unittest
import numpy as np
from mock import MagicMock

from thruster_control.thrust_computer.thrust_computer import ThrustComputer
from thruster_control.thrust_computer.thruster_configurations import ThrusterConfiguration

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
    def test_thursts_optimization_two(self):
        W_1 = [87,43,15,57,47,58]
        W_2 = [87,43,15,57,47,58]
        W_3 = [87,43,15,57,47,58]

        config = ThrusterConfiguration()
        config.initialize()
        obj = ThrustComputer(config)

        answer = obj.optimize_thrusts_two(W_1, W_2, W_3)
        print(answer)

        

if __name__ == '__main__':
    rospy.init_node('test_thrust_computer')
    rostest.rosrun('thruster_control', 'test_thrust_computer', TestThrustComputer)
