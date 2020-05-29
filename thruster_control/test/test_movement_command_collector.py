#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.movement_command_collector import MovementCommandCollector, CommandSubscriber


class TestCommandSubscriber(unittest.TestCase):
    def test_contruction(self):
        cs = CommandSubscriber('test_topic')


class TestMovementCommandCollector(unittest.TestCase):
    def test_contruction(self):
        mcc = MovementCommandCollector()


if __name__ == '__main__':
    rospy.init_node('test_movement_command_collector')
    rostest.rosrun('thruster_control', 'test_command_subscriber', TestCommandSubscriber)
    rostest.rosrun('thruster_control', 'test_movement_command_collector', TestMovementCommandCollector)
