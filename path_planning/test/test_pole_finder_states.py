#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

import path_planning.pole_finder_demo_states as States

class TestGoToDepth(unittest.TestCase):
    def test_contruction(self):
        s = States.GoToDepthState(0)
    
    def test_wait_for_time(self):
        s = States.GoToDepthState(0)

        s.time_at_depth = 0
        assert(not s.depth_is_reached())

        s.time_at_depth = 10
        assert(s.depth_is_reached())



if __name__ == '__main__':
    rospy.init_node('test_pole_finder_states')
    rostest.rosrun('path_planning', 'test_goto_dpeth', TestGoToDepth)
