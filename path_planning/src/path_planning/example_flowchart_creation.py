"""
This is an example of automatically generating a flowchart of a node.
The example given is of the dive_test node.
"""
from path_planning.states.waiting_state import WaitingState
from path_planning.states.stabilize_state import StabilizeState
from path_planning.states.go_to_depth import GoToDepthState
from path_planning.states.exit_code_state import ExitCodeState
from path_planning.states.data_logger import DataLogger
from path_planning.state_machines.sequential_state_machine import SequentialStateMachine
from path_planning.state_machines.parallel_state_machine import ParallelStateMachine

# create the states/state machines
target_depth = -6  # m

dive_machine = SequentialStateMachine('dive', [WaitingState(20), StabilizeState(),
                                               GoToDepthState(target_depth, tolerance=0.05,
                                                              velocity_tolerance=0.01),
                                               WaitingState(10), GoToDepthState(0), WaitingState(10), ExitCodeState(0)])
data_logger = DataLogger('dive-test')
data_logger.add_data_post_processing_func(plot_depth_data)

dive_logging_machine = ParallelStateMachine('dive_logger', states=[dive_machine],
                                            daemon_states=[data_logger])

# now, call the following functions on the top level state/state machine
dive_logging_machine.get_tree().render_graph('node_flowcharts/dive-test')  # path to destination of flowchart (PDF)
