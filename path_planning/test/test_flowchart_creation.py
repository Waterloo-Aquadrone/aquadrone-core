#!/usr/bin/env python3

import rospy
import rostest
import unittest

from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.states.data_logger import DataLogger
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine

from path_planning.state_tree import Tree


class TestFlowchartCreation(unittest.TestCase):
    """
    This is an example of automatically generating a flowchart of a node.
    The example given is of the dive_test node.
    """
    
    def test_dive_test_flowchart(self):
        # create the states/state machines
        target_depth = -6  # m

        dive_machine = SequentialStateMachine('dive', [WaitingState(20), StabilizeState(),
                                                       GoToDepthState(target_depth, tolerance=0.05,
                                                                      velocity_tolerance=0.01),
                                                       WaitingState(10), GoToDepthState(0), WaitingState(10), ExitCodeState(0)])
        data_logger = DataLogger('dive-test')

        dive_logging_machine = ParallelStateMachine('dive_logger', states=[dive_machine],
                                                    daemon_states=[data_logger])

        # now, call the following functions on the top level state/state machine
        Tree.create_flowchart(dive_logging_machine, 'test-flowchart')



if __name__ == '__main__':
    rospy.init_node('test_flowchart_creation')
    rostest.rosrun('path_planning', 'test_flowchart_creation', TestFlowchartCreation)

