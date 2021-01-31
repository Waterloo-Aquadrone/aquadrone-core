#!/usr/bin/env python3

import rospy
import rostest
import unittest
from mock import MagicMock

import aquadrone_math_utils.orientation_math as OMath


class TestOMath(unittest.TestCase):
    def smoke_test(self):
        OMath.RPY_Matrix(0, 0, 0)


if __name__ == '__main__':
    rospy.init_node('test_o_math')
    rostest.rosrun('aquadrone_math_utils', 'test_orientation_math', TestOMath)
