from path_planning.states.base_state import BaseState


class TravelState(BaseState):
    """
    Moves the submarine to the target position, assuming there are no obstacles in the way.
    """

    def __init__(self, target_x=None, target_y=None, target_z=None, target_yaw=None):
        """
        This class assumes that the target roll and pitch are 0.
        """
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = target_yaw

    def update_target(self, target_x=None, target_y=None, target_z=None, target_yaw=None):
        """
        Updates the target to the new location.
        """
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = target_yaw

    def state_name(self):
        return "travel"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        pass

    def exit_code(self):
        return 0
