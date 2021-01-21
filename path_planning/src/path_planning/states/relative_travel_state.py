from path_planning.states.base_state import BaseState
from path_planning.states.travel_state import TravelState


class RelativeTravelState(BaseState):
    """
    Moves the submarine to the target offset from an object in the world.
    """

    def __init__(self, target='pole', x_offset=None, y_offset=None, z_offset=None, relative_yaw=None):
        self.target = target
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.relative_yaw = relative_yaw

    def state_name(self):
        return "relative_travel"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        target_pos = world_state.get_world_state()[self.target].position

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        pass

    def exit_code(self):
        return 0
