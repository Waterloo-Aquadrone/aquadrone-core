#!/usr/bin/env python3

import rospy

from path_planning.states.go_to_depth import GoToDepthState
from path_planning.pole_finder_demo_states import ColoredPoleFinderState, ColoredPoleApproacherState
from path_planning.state_machines.markov_chain_state_machine import MarkovChainStateMachine
from path_planning.state_machines.timed_state_machine import TimedStateMachine
from path_planning.state_executor import StateExecutor


if __name__ == "__main__":
    rospy.init_node("pole_finder_demo")

    depth_target = -3

    # BGR
    red_low = (0, 0, 40)
    red_high = (80, 80, 255)
    green_low = (0, 60, 0)
    green_high = (80, 255, 80)
    blue_low = (60, 0, 0)
    blue_high = (255, 10, 10)

    states, mappings = zip((GoToDepthState(depth_target),                      {0: 1}),        # 0
                           (ColoredPoleFinderState(red_low, red_high),         {0: 2}),        # 1
                           (ColoredPoleApproacherState(red_low, red_high),     {0: 3, 1: 1}),  # 2
                           (ColoredPoleFinderState(green_low, green_high),     {0: 4}),        # 3
                           (ColoredPoleApproacherState(green_low, green_high), {0: 5, 1: 3}),  # 4
                           (ColoredPoleFinderState(blue_low, blue_high),       {0: 6}),        # 5
                           (ColoredPoleApproacherState(blue_low, blue_high),   {0: 2, 1: 5}))  # 6

    machine = MarkovChainStateMachine('pole_finder_demo', states, mappings)
    timed_machine = TimedStateMachine(machine, timeout=5 * 60, timeout_exit_code=0)

    # TODO: add flowchart generation once MarkovChainStateMachine is supported
    executor = StateExecutor(timed_machine, rate=rospy.Rate(5))
    executor.run()

