#!/usr/bin/env python3

import rospy
import rospkg
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
from path_planning.states.searching_state import SearchingState
from path_planning.state_machines.timed_state_machine import TimedStateMachine
from path_planning.state_tree import Tree
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
    plt.savefig(f'{directory}/pole searching plot{time()}.png')


if __name__ == "__main__":
    rospy.init_node("pole_search_test")

    target_depth = 3  # m

    dive_machine = SequentialStateMachine('dive', [WaitingState(20), StabilizeState(),
                                                   GoToDepthState(target_depth, tolerance=0.5,
                                                                  velocity_tolerance=0.5),
                                                   WaitingState(10), TimedStateMachine(SearchingState(target='blue_pole'), 60), WaitingState(10), ExitCodeState(0)])
    data_logger = DataLogger('pole_search_test')

    data_logger.add_data_post_processing_func(plot_circling_data)

    dive_logging_machine = ParallelStateMachine('dive_logger', states=[dive_machine],
                                                daemon_states=[data_logger])

    Tree.create_flowchart(dive_logging_machine, 'pole_search_travel_test')

    executor = StateExecutor(dive_logging_machine, rate=rospy.Rate(5))
    executor.run()
