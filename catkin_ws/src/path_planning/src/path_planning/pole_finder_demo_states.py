class BaseState(object):
    def __init__(self):
        pass

    def state_name(self):
        return "base_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        # Set up anything that needs initializing
        # Run EACH time the state is chosen as the next state
        # process(...) will be called with the next available data
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        # Clean up anything necessary
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        # Regular tick at some rate
        pass

    # Expose functions to identify when a state should exit
    # Ex: has_timed_out, has_lost_track_of_[some object]



class GoToDepthState(BaseState):
    def __init__(self, d):
        self.depth_goal = d
        self.current_depth = 0

    def state_name(self):
        return "Go To Depth"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        controls.set_depth_goal(self.depth_goal)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        self.current_depth = -sub_state.get_submarinne_state().position.z

    def depth_is_reached(self):
        return abs(self.current_depth - self.depth_goal) < 0.25
