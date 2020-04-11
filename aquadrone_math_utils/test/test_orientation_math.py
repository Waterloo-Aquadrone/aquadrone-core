#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

import aquadrone_math_utils.orientation_math as OMath

class TestOMath(unittest.TestCase):

    def smoke_test(self):
        OMath.Roll(0)
        OMath.Pitch(0)
        OMath.Yaw(0)
        


if __name__ == '__main__':
    rospy.init_node('test_o_math')
    rostest.rosrun('aquadrone_math_utils', 'test_orientation_math', TestOMath)
