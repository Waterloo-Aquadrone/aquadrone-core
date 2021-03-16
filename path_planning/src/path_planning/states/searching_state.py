from path_planning.states.base_state import BaseState
import math

class SearchingState(BaseState):
    """
      Moves the submarine to the target position, assuming there are no obstacles in the way.
     """

    def __init__(self, origin_x=None, origin_y=None, origin_z=None, target_yaw=None, target =""):
        """
        This class assumes that the target roll and pitch are 0.
        """
        self.target = target
        self.completed = False
        self.checkpoints = None
        self.target_x = None
        self.target_y = None
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_z = origin_z
        self.target_yaw = target_yaw

    @staticmethod
    def spiral_calc(origin_x, origin_y,):
        """
        Calculates the x and y coordinates in a spiral
        plot and updates an origin x and y coordinate by
        accepting the target_x and target_y in its arguments
        """

        i = 0
        while True:
            space_btw_lines = 5
            velo_time_rate = 20
            t = i / velo_time_rate * math.pi
            x = (1 + space_btw_lines * t) * math.cos(t)
            y = (1 + space_btw_lines * t) * math.sin(t)
            new_target_x = origin_x + x
            new_target_y = origin_y + y
            yield new_target_x, new_target_y
            i += 1

    def state_name(self):
        return "searching_state"
    def initialize(self, t, controls, sub_state, world_state, sensors):
        if self.origin_x is None:
            position = sub_state.get_submarine_state().position
            self.origin_x = position.x
        if self.origin_y is None:
            position = sub_state.get_submarine_state().position
            self.origin_y = position.y
        if self.origin_z is None:
            position = sub_state.get_submarine_state().position
            self.origin_z = position.z
        self.checkpoints = self.spiral_calc(self.origin_x, self.origin_y)

        self .target_x, self.target_y = next(self.checkpoints)
        controls.set_movement_target(self.target_x, self.target_y)

        """
        target_x and target_y are points on the spiral
        from the list of x and y checkpoints
        """

    def process(self, t, controls, sub_state, world_state, sensors):
        position = sub_state.get_submarine_state().position
        subpos_x = position.x
        subpos_y = position.y
        tolerance = 1
        if abs(self.target_x - subpos_x) < tolerance and abs(self.target_y - subpos_y) < tolerance:
            self.target_x, self.target_y = next(self.checkpoints)
            controls.set_movement_target(self.target_x, self.target_y)

        if self.target in world_state.get_world_state().keys():
            self.completed = True

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        return self.completed

    def exit_code(self):
        return 0

