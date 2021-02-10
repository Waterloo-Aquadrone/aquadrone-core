#!/usr/bin/env python3
import rospy
import rospkg

from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.relative_travel_state import RelativeTravelState
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.states.data_logger import DataLogger
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine
from path_planning.state_executor import StateExecutor
from matplotlib import pyplot as plt
from time import time


def plot_circling_data(data):
    directory = rospkg.RosPack().get_path('path_planning')
    plt.plot(data['t'], data['x'], label='x_pos')
    plt.plot(data['t'], data['y'], label='y_pos')
    plt.xlabel('Time(s)')
    plt.ylabel('x and y positions(m)')
    plt.title('Position vs Time')
    plt.legend()
    plt.savefig(f'{directory}/pole circling plot{time()}.png')


if __name__ == "__main__":
    rospy.init_node("pole_circling_test")

    target_depth = 3
    target_radius = 3
    relative_yaw = 0
    laps = 1

    dive_machine = SequentialStateMachine('dive', [WaitingState(5), StabilizeState(), GoToDepthState(target_depth, 0.5, 0.5),
                                                   WaitingState(1)])
    circling_machine = SequentialStateMachine('circle', [RelativeTravelState('red_pole',0,target_radius,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('green_pole', target_radius, 0,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('green_pole', 0, target_radius,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('green_pole', -target_radius, 0,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('red_pole', 0, target_radius,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('red_pole', target_radius, 0,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('red_pole', 0, -target_radius,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('red_pole', -target_radius, 0,
                                                                             target_depth, relative_yaw),
                                                         RelativeTravelState('red_pole', 0, target_radius,
                                                                             target_depth, relative_yaw)
                                                         ])

    surface_machine = SequentialStateMachine('surface', [GoToDepthState(0), WaitingState(10), ExitCodeState(0)])
    mission_machine = SequentialStateMachine('main', [dive_machine] + (laps * [circling_machine]) + [surface_machine])

    data_logger = DataLogger('pole_circling')
    data_logger.add_data_post_processing_func(plot_circling_data)

    pole_logging_machine = ParallelStateMachine('pole_logger', states=[mission_machine], daemon_states=[data_logger])

    executor = StateExecutor(pole_logging_machine, rate=rospy.Rate(5))
    executor.run()

