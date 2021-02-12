from numpy import abs
from path_planning.states.base_state import BaseState


class GoToDepthState(BaseState):
    """
    This state does not interact with the orientation stability system. It sends a single command to the depth control
    system during initialization (which will remain active after finalization as well).
    """

    def __init__(self, depth, tolerance=0.25, velocity_tolerance=0.1, verbose=False):
        self.depth_goal = depth
        self.completed = False
        self.tolerance = tolerance
        self.velocity_tolerance = velocity_tolerance
        self.verbose = verbose

    def state_name(self):
        return "go_to_depth_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.completed = False
        controls.set_depth_goal(self.depth_goal)
        print(self.state_name(), 'starting to go to depth', self.depth_goal, 'm')

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        current_depth = sub_state.get_submarine_state().position.z
        depth_err = abs(current_depth - self.depth_goal)
        if self.verbose:
            print("Depth err (abs): %f" % depth_err)

        v_z = sub_state.get_submarine_state().velocity.z
        if depth_err < self.tolerance and abs(v_z) < self.velocity_tolerance:
            self.completed = True
            print(self.state_name(), 'completed!')

    def has_completed(self):
        return self.completed
