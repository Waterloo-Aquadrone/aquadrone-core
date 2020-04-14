#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

from thruster_control.thruster_types import BlueRoboticsT100, UUVSimThruster



class TestT100Thruster(unittest.TestCase):

    def test_contruction(self):
        th = BlueRoboticsT100(60)


class TestUUVThruster(unittest.TestCase):

    def test_contruction(self):
        th = UUVSimThruster()



if __name__ == '__main__':
    rospy.init_node('test_thruster_control_manager')
    rostest.rosrun('thruster_control', 'test_T100_thruster', TestT100Thruster)
    rostest.rosrun('thruster_control', 'test_UUV_thruster', TestUUVThruster)

