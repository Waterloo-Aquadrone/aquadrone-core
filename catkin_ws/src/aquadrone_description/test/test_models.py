#!/usr/bin/env python

import unittest
import rospy
import rostest

class TestV2Loaded(unittest.TestCase):

    def test_param_loaded(self):
        val = rospy.get_param("/v2_description", None)
        self.assertIsNotNone(val)

if __name__ == '__main__':
    rostest.rosrun('aquadrone_description', 'test_v2', TestV2Loaded)