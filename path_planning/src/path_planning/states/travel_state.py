import numpy as np
from path_planning.states.base_state import BaseState
from aquadrone_math_utils.angle_math import abs_angle_difference


class TravelState(BaseState):
    """
    Moves the submarine to the target position, assuming there are no obstacles in the way.
    """

    def __init__(self, target_x=0, target_y=0, target_z=0, target_yaw=0, verbose=True):
        """
        This class assumes that the target roll and pitch are 0.
        Units are meters and degrees.
        """
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = np.radians(target_yaw)
        self.verbose = verbose
        self.invalidated = True
        self.completed = False

    def __repr__(self):
        return f'TravelState({self.target_x}, {self.target_y}, {self.target_z}, target_yaw={self.target_yaw}, ' \
               f'verbose={self.verbose})'

    def __str__(self):
        return f'TravelState({self.target_x}, {self.target_y}, {self.target_z}, target_yaw={self.target_yaw})'

    def update_target(self, target_x=None, target_y=None, target_z=None, target_yaw=None):
        """
        Updates the target to the new location. Any parameters left as None will be ignored.
        Units are meters and degrees.
        """
        if target_x is not None:
            self.target_x = target_x
            self.invalidated = True
        if target_y is not None:
            self.target_y = target_y
            self.invalidated = True
        if target_z is not None:
            self.target_z = target_z
            self.invalidated = True
        if target_yaw is not None:
            self.target_yaw = np.radians(target_yaw)
            self.invalidated = True

    def update_targets(self, controls):
        if self.invalidated:
            controls.set_movement_target(self.target_x, self.target_y)
            controls.set_depth_goal(self.target_z)
            controls.set_yaw_goal(self.target_yaw)
            self.invalidated = False

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.invalidated = True
        self.completed = False
        if self.verbose:
            print(f'Starting to travel to: (x={self.target_x}, y={self.target_y}, '
                  f'z={self.target_z}, yaw={self.target_yaw})')
        self.update_targets(controls)

    def process(self, t, controls, sub_state, world_state, sensors):
        self.update_targets(controls)

        sub = sub_state.get_submarine_state()
        displacement = np.array([self.target_x - sub.position.x,
                                 self.target_y - sub.position.y])
        dist = np.linalg.norm(displacement)
        v = np.array([sub.velocity.x, sub.velocity.y])

        # check to see if we are completed
        if dist < 0.1 and np.linalg.norm(v) < 0.1 and \
                abs_angle_difference(sub.orientation_rpy.z, self.target_yaw) < 10 and \
                np.degrees(np.abs(sub.angular_velocity.z)) < 5 and \
                abs(sub.position.z - self.target_z) < 0.1 and np.abs(sub.velocity.z) < 0.1:
            self.completed = True
            return

    def has_completed(self):
        return self.completed
