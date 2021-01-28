#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.thrust_computer.thruster_configurations import ThrusterConfiguration


class TestV1Configuration(unittest.TestCase):
    def test_contruction(self):
        c = ThrusterConfiguration('v1')

    def smoke_test(self):
        c = ThrusterConfiguration('v1')
        c.initialize()
        c.get_num_thrusters()
        c.get_thrusts_to_wrench_matrix()
        # Not testing wrench-to-thrusts


class TestV2Configuration(unittest.TestCase):
    def test_contruction(self):
        c = ThrusterConfiguration('v2')

    def smoke_test(self):
        c = ThrusterConfiguration('v2')
        c.initialize()
        c.get_num_thrusters()
        c.get_thrusts_to_wrench_matrix()
        # Not testing wrench-to-thrusts


class TestV28Configuration(unittest.TestCase):
    def test_contruction(self):
        c = ThrusterConfiguration('v28')

    def smoke_test(self):
        c = ThrusterConfiguration('v28')
        c.initialize()
        c.get_num_thrusters()
        c.get_thrusts_to_wrench_matrix()
        # Not testing wrench-to-thrusts


if __name__ == '__main__':
    rospy.init_node('test_thruster_configurations')
    rostest.rosrun('thruster_control', 'test_thruster_configurations_v1', TestV1Configuration)
    rostest.rosrun('thruster_control', 'test_thruster_configurations_v2', TestV2Configuration)
    rostest.rosrun('thruster_control', 'test_thruster_configurations_v28', TestV28Configuration)
