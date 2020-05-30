#!/usr/bin/env python

import rospy

from path_planning.states.go_to_depth import GoToDepthState
from path_planning.pole_finder_demo_states import ColoredPoleFinderState, ColoredPoleApproacherState
from path_planning.state_machines.markov_chain_state_machine import MarkovChainStateMachine
from path_planning.states.base_state import run_state


if __name__ == "__main__":
    rospy.init_node("pole_finder_demo")

    goal_depth = 3

    # BGR
    red_low = (0, 0, 40)
    red_high = (80, 80, 255)
    green_low = (0, 60, 0)
    green_high = (80, 255, 80)
    blue_low = (60, 0, 0)
    blue_high = (255, 10, 10)

    states, mappings = zip((GoToDepthState(goal_depth),                        {0: 1}),
                           (ColoredPoleFinderState(red_low, red_high),         {0: 2}),
                           (ColoredPoleApproacherState(red_low, red_high),     {0: 3, 1: 1}),
                           (ColoredPoleFinderState(green_low, green_high),     {0: 4}),
                           (ColoredPoleApproacherState(green_low, green_high), {0: 5, 1: 3}),
                           (ColoredPoleFinderState(blue_low, blue_high),       {0: 6}),
                           (ColoredPoleApproacherState(blue_low, blue_high),   {0: 2, 1: 5}))

    machine = MarkovChainStateMachine('pole_finder_demo', states, mappings)

    run_state(machine, rate=rospy.Rate(5))
