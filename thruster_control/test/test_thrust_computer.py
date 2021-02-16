#!/usr/bin/env python3.8

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.thrust_computer.thrust_computer import ThrustComputer


class TestThrustComputer(unittest.TestCase):
    def test_contruction(self):
        c = ThrustComputer(None, None)


if __name__ == '__main__':
    rospy.init_node('test_thrust_computer')
    rostest.rosrun('thruster_control', 'test_thrust_computer', TestThrustComputer)

