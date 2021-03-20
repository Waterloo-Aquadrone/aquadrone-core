#!/usr/bin/env python3

import rospy
import rostest
import unittest
from mock import MagicMock
import numpy as np
from aquadrone_math_utils.quaternion import Quaternion


class TestQuaternion(unittest.TestCase):
    def check_quat_to_euler(self, quat):
        expected_euler_angles = quat.as_euler()
        actual_euler_angles = np.array(Quaternion.as_euler_sympy(quat), dtype=float)
        self.assertTrue(np.all(np.isclose(actual_euler_angles, expected_euler_angles)))

    def test_quat_to_euler(self):
        self.check_quat_to_euler(Quaternion.from_array([0.1, 0.2, 0.3, 0.4]).normalize())
        self.check_quat_to_euler(Quaternion.from_array([0.1, -0.2, 0.15, 0.1]).normalize())
        self.check_quat_to_euler(Quaternion.from_array([0.5, 0.5, 0.5, -0.5]).normalize())
        # sinp = -1 case
        self.check_quat_to_euler(Quaternion.from_array([-0.1, 0.5 ** 0.5, 0, 0.7]).normalize())
        # sinp = 1 case
        self.check_quat_to_euler(Quaternion.from_array([0, 0.5 / 0.7, 0, 0.7]).normalize())

        # random tests
        for _ in range(10):
            self.check_quat_to_euler(Quaternion.from_array(np.random.random(4) * 2 - 1).normalize())


if __name__ == '__main__':
    rospy.init_node('test_quaternion')
    rostest.rosrun('aquadrone_math_utils', 'test_quaternion', TestQuaternion)
