#!/usr/bin/env python

import rospy

from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.barrel_roll import BarrelRoll
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine
from path_planning.state_machines.timed_state_machine import TimedStateMachine
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.branching_state_machine import BranchingStateMachine
from path_planning.states.base_state import run_state


if __name__ == "__main__":
    rospy.init_node("barrel_roll_test")

    timeout = 120  # seconds

    stabilise_at_depth_machine = ParallelStateMachine('stabilise_at_depth', [StabilizeState(), GoToDepthState(3)])
    timed_barrel_roll_state = TimedStateMachine(BarrelRoll(), timeout, timeout_exit_code=1)
    dive_and_roll_machine = SequentialStateMachine('dive_and_roll', [WaitingState(20),
                                                                     stabilise_at_depth_machine,
                                                                     timed_barrel_roll_state])
    success_surface_machine = SequentialStateMachine('surface', [GoToDepthState(0), WaitingState(10), ExitCodeState(0)])
    failure_surface_machine = SequentialStateMachine('stabilize_and_surface', [StabilizeState(), GoToDepthState(0),
                                                                               WaitingState(10), ExitCodeState(1)])

    machine = BranchingStateMachine('barrel_roll_test', dive_and_roll_machine, success_surface_machine,
                                    failure_surface_machine)
    success = run_state(machine, rospy.Rate(5))

    if success == 0:
        print('Successfully completed barrel roll!')
    else:
        print('Aborted barrel roll attempt!')
