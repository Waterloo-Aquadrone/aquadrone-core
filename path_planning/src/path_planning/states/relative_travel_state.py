from path_planning.states.base_state import BaseState
from path_planning.states.travel_state import TravelState
import math


class RelativeTravelState(BaseState):
    """
    Moves the submarine to the target offset from an object in the world.
    """

    def __init__(self, target='pole', x_offset=None, y_offset=None, z_offset=None, relative_yaw=None):
        """
        :param x_offset: This parameter is the offset in the x-direction from the target to the submarine
        :param y_offset: This parameter is the offset in the y-direction from the target to the submarine
        :param z_offset: This parameter is the offset in the z-direction from the target to the submarine
        :param relative_yaw: This parameter is the yaw that makes the submarine face the target plus the added relative yaw
        """
        self.target = target
        self.travel_state_helper = TravelState()
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.relative_yaw = relative_yaw
        self.absolute_target_yaw = math.atan2(-y_offset, -x_offset) + relative_yaw
        
    def update_target(self, target):
        self.target = target
        
    def update_target_offset(self, x_offset=0, y_offset=0, z_offset=0, relative_yaw=0):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.relative_yaw = relative_yaw
        self.absolute_target_yaw = math.atan2(-y_offset, -x_offset) + relative_yaw

    def update_travel_state_helper(self, world_state):
        targetPos = world_state.get_world_state()[self.target].position
        new_target_x = targetPos.x + self.x_offset
        new_target_y = targetPos.y + self.y_offset
        new_target_z = targetPos.z + self.z_offset

        self.travel_state_helper.update_target(new_target_x, new_target_y, new_target_z, self.absolute_target_yaw)

    def state_name(self):
        return "relative_travel"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.update_travel_state_helper(world_state)
        self.travel_state_helper.initialize(t, controls, sub_state, world_state, sensors)

    def process(self, t, controls, sub_state, world_state, sensors):

        self.update_travel_state_helper(world_state)
        self.travel_state_helper.process(t, controls, sub_state, world_state, sensors)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.travel_state_helper.finalize(t, controls, sub_state, world_state, sensors)

    def has_completed(self):
        return self.travel_state_helper.has_completed()

    def exit_code(self):
        return 0
