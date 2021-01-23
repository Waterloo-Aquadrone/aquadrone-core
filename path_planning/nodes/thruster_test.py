#!/usr/bin/env python

import rospy

from path_planning.states.waiting_state import WaitingState
from path_planning.states.thruster_test_state import ThrusterTestState
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_executor import StateExecutor


if __name__ == "__main__":
    rospy.init_node("thruster_test")

    machine = SequentialStateMachine('thruster_test', [WaitingState(10), ThrusterTestState(thruster_count=8, thrust_amplitude=3, thrust_period=5), WaitingState(10)])

    executor = StateExecutor(machine, rate=rospy.Rate(5))
    executor.run()
