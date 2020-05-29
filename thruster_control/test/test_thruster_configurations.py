#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.thruster_configurations import V1Configuration, V2Configuration, V28Configuration


class TestV1Configuration(unittest.TestCase):
    def test_contruction(self):
        c = V1Configuration()

    def smoke_test(self):
        c = V1Configuration()
        c.initialize()
        c.get_num_thrusters()
        c.get_thrusts_to_wrench_matrix()
        # Not testing wrench-to-thrusts


class TestV2Configuration(unittest.TestCase):
    def test_contruction(self):
        c = V2Configuration()

    def smoke_test(self):
        c = V2Configuration()
        c.initialize()
        c.get_num_thrusters()
        c.get_thrusts_to_wrench_matrix()
        # Not testing wrench-to-thrusts


class TestV28Configuration(unittest.TestCase):
    def test_contruction(self):
        c = V28Configuration()

    def smoke_test(self):
        c = V28Configuration()
        c.initialize()
        c.get_num_thrusters()
        c.get_thrusts_to_wrench_matrix()
        # Not testing wrench-to-thrusts


if __name__ == '__main__':
    rospy.init_node('test_thruster_configurations')
    rostest.rosrun('thruster_control', 'test_thruster_configurations_v1', TestV1Configuration)
    rostest.rosrun('thruster_control', 'test_thruster_configurations_v2', TestV2Configuration)
    rostest.rosrun('thruster_control', 'test_thruster_configurations_v28', TestV28Configuration)
