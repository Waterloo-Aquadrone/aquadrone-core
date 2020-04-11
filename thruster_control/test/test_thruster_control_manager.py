#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.thruster_control_manager import SimThrusterController, RealThrusterController



class TestSimThrusterControlManager(unittest.TestCase):

    def test_contruction(self):
        c = SimThrusterController(None, None, None, None)


class TestRealThrusterControlManager(unittest.TestCase):

    def test_contruction(self):
        c = RealThrusterController(None, None, None, None)



if __name__ == '__main__':
    rospy.init_node('test_thruster_control_manager')
    rostest.rosrun('thruster_control', 'test_thruster_control_manager', TestRealThrusterControlManager)

