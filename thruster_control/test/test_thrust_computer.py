#!/usr/bin/env python3.8

import rospy
import rostest
import unittest
import numpy as np
from mock import MagicMock
from time import time

from thruster_control.thrust_computer.thrust_computer import ThrustComputer
from thruster_control.thrust_computer.thruster_configurations import ThrusterConfiguration


class TestThrustComputer(unittest.TestCase):
    def test_construction(self):
        c = ThrustComputer(None, None)

    def test_thrusts_optimization(self):
        config = ThrusterConfiguration()
        config.initialize()
        computer = ThrustComputer(config)

        wrench1 = np.array([15, 15, 15, 15, 15, 15])
        wrench2 = np.array([20, 1, 1, 1, 1, 1])
        wrench3 = np.array([1.9, 1, 1, 1, 1, 1])

        start = time()
        final_thrusts = computer.optimize_thrusts([wrench1, wrench2, wrench3])
        self.assertLess(time() - start, 0.1, msg='Thrust optimization took longer than 0.1 seconds!')

    def test_get_efficiency_error(self):
        config = ThrusterConfiguration()
        config.initialize()
        computer = ThrustComputer(config)
        computer.k = 0

        W_1 = np.array([1, 0, 0, 0, 0, 0])
        W_2 = np.array([0, 1, 0, 0, 0, 0])
        W_3 = np.array([0, 0, 1, 0, 0, 0])
        ideal_thrusts = np.dot(config.wrench_to_thrusts_matrix, W_1 + W_2 + W_3)
        error = computer.get_efficiency_error(ideal_thrusts, [W_1, W_2, W_3])
        self.assertTrue(np.isclose(error, -111))

        computer.k = 5
        error = computer.get_efficiency_error(ideal_thrusts, [W_1, W_2, W_3])
        self.assertGreater(error, -111)

        a, b, c = 0.1, 0.3, 0.5
        test_thrusts = np.dot(config.wrench_to_thrusts_matrix, a * W_1 + b * W_2 + c * W_3)
        computer.k = 0
        error = computer.get_efficiency_error(test_thrusts, [W_1, W_2, W_3])
        self.assertTrue(np.isclose(error, -100 * a - 10 * b - 1 * c))

        computer.k = 5
        error = computer.get_efficiency_error(test_thrusts, [W_1, W_2, W_3])
        self.assertGreater(error, -100 * a - 10 * b - 1 * c)

    def test_thrusts_optimization_two(self):
        # x = 50
        # W_1 = np.array([x, 0, 0, 0, 0, 0])
        # W_2 = np.array([0, x, 0, 0, 0, 0])
        # W_3 = np.array([0, 0, x, 0, 0, 0])
        W_1 = 10 * np.random.random(6)
        W_2 = 10 * np.random.random(6)
        W_3 = 10 * np.random.random(6)
        print('W1', W_1)
        print('W2', W_2)
        print('W3', W_3)
        print('Total', W_1 + W_2 + W_3)

        config = ThrusterConfiguration()
        config.initialize()
        computer = ThrustComputer(config)
        computer.k = 0

        start = time()
        thrusts = computer.optimize_thrusts_2([W_1, W_2, W_3])
        # self.assertLess(time() - start, 1, msg='Thrust optimization took longer than 1 seconds!')
        # self.assertTrue(np.all(np.isclose(np.dot(computer.config.thrust_to_wrench_matrix, thrusts), W_1 + W_2 + W_3,
        #                                   atol=1)))
        print('Thrusts:', np.round(thrusts, decimals=4))
        print('Wrench:', np.round(np.dot(computer.config.thrust_to_wrench_matrix, thrusts), decimals=4))

    def test_compare_optimizers(self):
        config = ThrusterConfiguration()
        config.initialize()
        computer = ThrustComputer(config)
        computer.k = 0.001

        W_1 = 10 * np.random.random(6)
        W_2 = 10 * np.random.random(6)
        W_3 = 10 * np.random.random(6)
        print('W1', W_1)
        print('W2', W_2)
        print('W3', W_3)
        print('Total', W_1 + W_2 + W_3)

        start = time()
        thrusts1 = computer.optimize_thrusts([W_1, W_2, W_3])
        method_1_time = time() - start
        start = time()
        thrusts2 = computer.optimize_thrusts_2([W_1, W_2, W_3])
        method_2_time = time() - start
        print('Method 1 thrusts:', np.round(thrusts1, decimals=4))
        print('Method 2 thrusts:', np.round(thrusts2, decimals=4))
        print('Method 1 wrench:', np.round(np.dot(config.thrust_to_wrench_matrix, thrusts1), decimals=4))
        print('Method 2 wrench:', np.round(np.dot(config.thrust_to_wrench_matrix, thrusts2), decimals=4))

        print('Method 1 error:', computer.get_efficiency_error(thrusts1, [W_1, W_2, W_3]))
        print('Method 2 error:', computer.get_efficiency_error(thrusts2, [W_1, W_2, W_3]))

        print('Method 1 time:', method_1_time)
        print('Method 2 time:', method_2_time)


if __name__ == '__main__':
    rospy.init_node('test_thrust_computer')
    rostest.rosrun('thruster_control', 'test_thrust_computer', TestThrustComputer)
