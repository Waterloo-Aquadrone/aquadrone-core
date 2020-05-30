#!/usr/bin/env python

import rospy

from path_planning.states.waitingstate import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.barrel_roll import BarellRoll
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.state_machines.timed_state_machine import TimedStateMachine
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.markov_chain_state_machine import MarkovChainStateMachine
from path_planning.states.base_state import run_state


if __name__ == "__main__":
    rospy.init_node("barrel_roll_test")

    timeout = 60  # seconds

    starting_states = [WaitingState(10), StabilizeState(), GoToDepthState(6)]
    timed_barrel_roll_state = TimedStateMachine(BarellRoll(), timeout, timeout_exit_code=1)
    success_states = [GoToDepthState(0), WaitingState(10), ExitCodeState(0)]
    failure_states = [StabilizeState(), GoToDepthState(0), WaitingState(10), ExitCodeState(1)]

    states, dictionaries = zip((SequentialStateMachine(starting_states), {0: 1}),        # 0
                               (timed_barrel_roll_state,                 {0: 2, 1: 3}),  # 1
                               (SequentialStateMachine(success_states),  {0: -1}),       # 2
                               (SequentialStateMachine(failure_states),  {0: -1}))       # 3
    machine = MarkovChainStateMachine('barrel_roll_test', states, dictionaries)
    success = run_state(machine)

    if success == 0:
        print('Successfully completed barrel roll!')
    else:
        print('Aborted barrel roll attempt!')
