#!/usr/bin/env python3.8

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.thrust_computer.movement_command_collector import MovementCommandCollector, CommandSubscriber


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
