#!/usr/bin/env python3.8

import rospy
import rospkg
from time import time
import matplotlib.pyplot as plt

from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.barrel_roll import BarrelRoll
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine
from path_planning.state_machines.timed_state_machine import TimedStateMachine
from path_planning.states.data_logger import DataLogger
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.branching_state_machine import BranchingStateMachine
from path_planning.state_executor import StateExecutor


def plot_orientation_data(data):
    directory = rospkg.RosPack().get_path('path_planning')
    for angle in ['roll', 'pitch', 'yaw']:
        plt.plot(data['t'], data[angle], label=angle)
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.legend()
    plt.title('Orientation Versus Time During Barrel Roll')
    plt.savefig(directory + '/barrel-roll-' + str(time()) + '.png')


if __name__ == "__main__":
    rospy.init_node("barrel_roll_test")

    timeout = 120  # seconds

    stabilise_at_depth_machine = ParallelStateMachine('stabilise_at_depth', [StabilizeState(), GoToDepthState(-3)])
    timed_barrel_roll_state = TimedStateMachine(BarrelRoll(), timeout, timeout_exit_code=1)
    dive_and_roll_machine = SequentialStateMachine('dive_and_roll', [WaitingState(20),
                                                                     stabilise_at_depth_machine,
                                                                     timed_barrel_roll_state])
    success_surface_machine = SequentialStateMachine('surface', [GoToDepthState(0), WaitingState(10), ExitCodeState(0)])
    failure_surface_machine = SequentialStateMachine('stabilize_and_surface', [StabilizeState(), GoToDepthState(0),
                                                                               WaitingState(10), ExitCodeState(1)])
    machine = BranchingStateMachine('barrel_roll_test', dive_and_roll_machine, success_surface_machine,
                                    failure_surface_machine)

    data_logger = DataLogger()
    data_logger.add_data_post_processing_func(plot_orientation_data)
    logging_machine = ParallelStateMachine('logging_state_machine', [machine], daemon_states=[data_logger])

    # TODO: add flowchart generation once BranchingStateMachine is supported
    executor = StateExecutor(logging_machine, rospy.Rate(5))
    executor.run()

    if executor.exit_code() == 0:
        print('Successfully completed barrel roll!')
    else:
        print('Aborted barrel roll attempt!')
