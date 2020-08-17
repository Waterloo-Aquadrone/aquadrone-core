#!/usr/bin/env python

import rospy

from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.states.data_logger import DataLogger
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.leader_follower_state_machine import LeaderFollowerStateMachine
from path_planning.state_executor import StateExecutor


if __name__ == "__main__":
    rospy.init_node("thruster_test")

    target_depth = 0.5  # m

    stabilise_at_depth_machine = ParallelStateMachine('stabilise_at_depth', [StabilizeState(),
                                                                             GoToDepthState(target_depth)])
    dive_machine = SequentialStateMachine('dive', [WaitingState(20), stabilise_at_depth_machine,
                                                   WaitingState(10), ExitCodeState(0)])
    dive_logging_machine = LeaderFollowerStateMachine('dive_logger', dive_machine, DataLogger())

    executor = StateExecutor(dive_logging_machine, rate=rospy.Rate(5))
    executor.run()
