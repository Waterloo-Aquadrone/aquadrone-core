#!/usr/bin/env python3.8

import rospy
import rospkg
from time import time
from matplotlib import pyplot as plt

from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.travel_state import TravelState
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.states.data_logger import DataLogger
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine
from path_planning.state_executor import StateExecutor


def plot_travel_data(data):
    directory = rospkg.RosPack().get_path('path_planning')
    plt.plot(data['t'], data['x'], label='x')
    plt.plot(data['t'], data['y'], label='y')
    plt.plot(data['t'], data['yaw'], label='yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m), Angle (radians)')
    plt.title('Travelling in a Square')
    plt.legend()
    plt.savefig(directory + '/square-travel-' + str(time()) + '.png')


if __name__ == "__main__":
    rospy.init_node("square_test")

    target_depth = -4  # m
    target_radius = 3  # m
    laps = 4

    dive_machine = SequentialStateMachine('dive', [WaitingState(20), StabilizeState(), GoToDepthState(target_depth),
                                                   WaitingState(10)])
    square_machine = SequentialStateMachine('square', [TravelState(target_radius, 0, target_depth, 90),
                                                       TravelState(target_radius, target_radius, target_depth, 180),
                                                       TravelState(0, target_radius, target_depth, 270),
                                                       TravelState(0, 0, target_depth, 0)])
    surface_machine = SequentialStateMachine('surface', [GoToDepthState(0), WaitingState(10), ExitCodeState(0)])
    mission_machine = SequentialStateMachine('main', [dive_machine] + (laps * [square_machine]) + [surface_machine])

    data_logger = DataLogger('square-travel')
    data_logger.add_data_post_processing_func(plot_travel_data)

    dive_logging_machine = ParallelStateMachine('dive_logger', states=[mission_machine], daemon_states=[data_logger])

    executor = StateExecutor(dive_logging_machine, rate=rospy.Rate(5))
    executor.run()
