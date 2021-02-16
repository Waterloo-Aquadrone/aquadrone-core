#!/usr/bin/env python3.8

import rospy
import rostest
import unittest
from mock import MagicMock
from std_srvs.srv import Trigger

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


def initialize_state(msg):
    pass


if __name__ == '__main__':
    rospy.init_node('test_ros_modules')
    mock_reset_service = rospy.Service('reset_sub_state_estimation', Trigger, initialize_state)

    rostest.rosrun('path_planning', 'test_ros_controls', TestROSConrols)
    rostest.rosrun('path_planning', 'test_ros_sensors', TestROSSensors)
    rostest.rosrun('path_planning', 'test_ros_state_est', TestROSStateEst)
