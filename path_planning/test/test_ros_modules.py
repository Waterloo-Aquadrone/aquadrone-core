#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

import path_planning.ros_modules as RM

class TestROSConrols(unittest.TestCase):
    def test_contruction(self):
        rcm = RM.ROSControlsModule()



class TestROSSensors(unittest.TestCase):
    def test_contruction(self):
        rcm = RM.ROSSensorDataModule()



class TestROSStateEst(unittest.TestCase):
    def test_contruction(self):
        rcm = RM.ROSStateEstimationModule()



if __name__ == '__main__':
    rospy.init_node('test_ros_modules')
    rostest.rosrun('path_planning', 'test_ros_controls', TestROSConrols)
    rostest.rosrun('path_planning', 'test_ros_sensors', TestROSSensors)
    rostest.rosrun('path_planning', 'test_ros_state_est', TestROSStateEst)
