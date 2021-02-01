#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
from matplotlib import pyplot as plt
from time import time

from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.states.data_logger import DataLogger
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine
from path_planning.state_executor import StateExecutor


def plot_yaw_data(data):
    directory = rospkg.RosPack().get_path('path_planning')
    plt.plot(data['t'], np.degrees(data['yaw']))
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (deg)')
    plt.title('Yaw Versus Time')
    plt.savefig(directory + '/yaw-control-' + str(time()) + '.png')


if __name__ == "__main__":
    rospy.init_node("yaw_test")

    target_depth = 3  # m
    target_yaw = 90  #

    yaw_machine = SequentialStateMachine('yaw', [WaitingState(20), StabilizeState(),
                                                 GoToDepthState(target_depth, tolerance=0.05,
                                                                velocity_tolerance=0.01),
                                                 WaitingState(10),
                                                 StabilizeState(yaw=np.radians(target_yaw), tolerance_degrees=1,
                                                                velocity_tolerance_degrees=0.1),
                                                 WaitingState(5),
                                                 StabilizeState(yaw=0, tolerance_degrees=1,
                                                                velocity_tolerance_degrees=0.1),
                                                 WaitingState(5),
                                                 GoToDepthState(0), WaitingState(10),
                                                 ExitCodeState(0)])
    data_logger = DataLogger('yaw-test')
    data_logger.add_data_post_processing_func(plot_yaw_data)

    yaw_logging_machine = ParallelStateMachine('yaw_logger', states=[yaw_machine],
                                               daemon_states=[data_logger])

    executor = StateExecutor(yaw_logging_machine, rate=rospy.Rate(5))
    executor.run()
