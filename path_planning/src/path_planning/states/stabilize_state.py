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
        self.velocity_tolerance = np.radians(2)  # 2 deg/s
        self.completed = False

    @staticmethod
    def state_name():
        return "stabilize_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.completed = False
        controls.set_orientation_goal(r=self.r, p=self.p, y=self.y)
        print(self.state_name(), 'starting to stabilize to (r, p, y)=(', self.r, ',', self.p, ',', self.y, ')')

    def process(self, t, controls, sub_state, world_state, sensors):
        state = sub_state.get_submarine_state()
        if np.abs(self.r - state.orientation_rpy.x) < self.tolerance and \
                np.abs(self.p - state.orientation_rpy.y) < self.tolerance and \
                np.abs(self.y - state.orientation_rpy.z) < self.tolerance and \
                state.angular_velocity.magnitude() < self.velocity_tolerance:
            self.completed = True
            print(self.state_name(), 'stabilized!')

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        return self.completed
