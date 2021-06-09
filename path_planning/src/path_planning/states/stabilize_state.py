import numpy as np
from path_planning.states.base_state import BaseState
from aquadrone_math_utils.angle_math import abs_angle_difference


class StabilizeState(BaseState):
    """
    This state does not interact with the depth control system. It sends a single command to the orientation stability
    system during initialization (which will remain active after finalization as well).
    """

    def __init__(self, roll=0, pitch=0, yaw=0, tolerance_degrees=5, velocity_tolerance_degrees=2):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.tolerance = np.radians(tolerance_degrees)
        self.velocity_tolerance = np.radians(velocity_tolerance_degrees)
        self.completed = False

    def __repr__(self):
        return f'StabilizeState(roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}, ' \
               f'tolerance_degrees={np.degrees(self.tolerance)}, velocity_tolerance_degrees={self.velocity_tolerance})'

    def __str__(self):
        return f'StabilizeState(roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.completed = False
        controls.set_orientation_goal(roll=self.roll, pitch=self.pitch, yaw=self.yaw)
        print(self, 'starting to stabilize to (r, p, y)=(', self.roll, ',', self.pitch, ',', self.yaw, ')')

    def process(self, t, controls, sub_state, world_state, sensors):
        state = sub_state.get_submarine_state()
        if abs_angle_difference(self.roll, state.orientation_rpy.x) < self.tolerance and \
                abs_angle_difference(self.pitch, state.orientation_rpy.y) < self.tolerance and \
                abs_angle_difference(self.yaw, state.orientation_rpy.z) < self.tolerance and \
                state.angular_velocity.magnitude() < self.velocity_tolerance:
            self.completed = True
            print(self, 'stabilized!')

    def has_completed(self):
        return self.completed
