#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock
from std_srvs.srv import Trigger

from aquadrone_msgs.msg import SubState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.base_state import t
from path_planning.ros_modules import ROSControlsModule, ROSStateEstimationModule, ROSSensorDataModule


class TestGoToDepth(unittest.TestCase):
    def test_construction(self):
        s = GoToDepthState(0)
    
    def test_wait_for_stabilized_at_depth(self):
        s = GoToDepthState(5)
        controls = ROSControlsModule()
        sub_state = ROSStateEstimationModule()
        sensors = ROSSensorDataModule()

        msg = SubState()
        sub_state.sub_state_callback(msg)
        s.initialize(t(), controls, sub_state, None, sensors)
        s.process(t(), controls, sub_state, None, sensors)
        assert(not s.has_completed())

        msg.position.z = -5
        msg.velocity.z = -10
        sub_state.sub_state_callback(msg)
        s.process(t(), controls, sub_state, None, sensors)
        assert(not s.has_completed())

        msg.velocity.z = 0
        sub_state.sub_state_callback(msg)
        s.process(t(), controls, sub_state, None, sensors)
        assert(s.has_completed())


def initialize_state(msg):
    pass


if __name__ == '__main__':
    rospy.init_node('test_pole_finder_states')
    mock_reset_service = rospy.Service('reset_sub_state_estimation', Trigger, initialize_state)
    rostest.rosrun('path_planning', 'test_goto_depth', TestGoToDepth)
