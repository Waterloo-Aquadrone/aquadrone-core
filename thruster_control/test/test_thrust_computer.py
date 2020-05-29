#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.thruster_control_manager import ThrusterController


class TestThrustComputer(unittest.TestCase):
    def test_contruction(self):
        c = ThrusterController(None, None)


if __name__ == '__main__':
    rospy.init_node('test_thrust_computer')
    rostest.rosrun('thruster_control', 'test_thrust_computer', TestThrustComputer)

