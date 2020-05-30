import numpy as np
from path_planning.states.base_state import BaseState


class StabilizeState(BaseState):
    """
    This state does not interact with the depth control system. It sends a single command to the orientation stability
    system during initialization (which will remain active after finalization as well).
    """

    def __init__(self, r=0, p=0, y=0, tolerance_degrees=5):
        self.r = r
        self.p = p
        self.y = y
        self.tolerance = np.radians(tolerance_degrees)
        self.velocity_tolerance = np.radians(2)  # rad/s
        self.completed = False

    @staticmethod
    def state_name():
        return "stabilize_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        controls.set_orientation_goal(r=self.r, p=self.p, y=self.y)

    def process(self, t, controls, sub_state, world_state, sensors):
        if np.abs(self.r - sub_state.orientation_rpy.x) < self.tolerance and \
                np.abs(self.p - sub_state.orientation_rpy.y) < self.tolerance and \
                np.abs(self.y - sub_state.orientation_rpy.z) < self.tolerance and \
                sub_state.angular_velocity.magnitude() < self.velocity_tolerance:
            self.completed = True

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        return self.completed
