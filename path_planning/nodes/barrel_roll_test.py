#!/usr/bin/env python

import rospy

from path_planning.states.base_state import run_state
from path_planning.states.linear_state_machine import LinearStateMachine
from path_planning.states.waitingstate import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.barrel_roll import BarellRoll


if __name__ == "__main__":
    rospy.init_node("barrel_roll_test")
    states = [WaitingState(10), StabilizeState(), GoToDepthState(6), BarellRoll(), GoToDepthState(0), WaitingState(10)]
    run_state(LinearStateMachine(states))
