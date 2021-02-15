import numpy as np
from path_planning.states.base_state import BaseState


class BarrelRoll(BaseState):
    """
    This state does not send any commands to the depth control system. It expects the depth control to be active at a
    depth that is sufficiently deep to avoid any part of the sub surfacing during the barrel roll.
    """

    def __init__(self, positive_dir=True):
        """
        :param positive_dir: If True, then the sub will roll in the direction corresponding to positive x.
        """
        self.positive_dir = positive_dir
        self.crossed_90 = False  # True once the sub has rotated 90 deg
        self.crossed_270 = False  # True once the sub has rotated 270 deg
        self.completed = False  # True once the sub has stabilized and completed the barrel roll

    @staticmethod
    def state_name():
        return "barrel_roll"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.crossed_90 = False
        self.crossed_270 = False
        self.completed = False

        angle = sub_state.get_submarine_state().orientation_rpy.x
        if np.pi / 4 < angle < 3 * np.pi / 4:
            print('Sub not in valid position to start barrel roll!')
            controls.halt_and_catch_fire()
            return
        print(self.state_name(), 'starting to barrel roll in',
              'positive' if self.positive_dir else 'negative', 'x direction')

    def process(self, t, controls, sub_state, world_state, sensors):
        if self.completed:
            return

        roll = sub_state.get_submarine_state().orientation_rpy.x
        roll_rate = sub_state.get_submarine_state().angular_velocity.x
        if self.positive_dir:
            if not self.crossed_90:
                if np.pi / 2 < roll < np.pi:
                    self.crossed_90 = True
                    print(self.state_name(), 'completed 90 degrees!')
                target_angle = roll + np.pi / 2
            elif not self.crossed_270:
                if 3 * np.pi / 2 < roll < 2 * np.pi:
                    self.crossed_270 = True
                    print(self.state_name(), 'completed 270 degrees!')
                    target_angle = 0
                else:
                    target_angle = roll + np.pi / 2
            else:
                # If within 5 deg of level and |roll_rate| < 2 deg/s
                if self.is_stable(roll, roll_rate):
                    self.completed = True
                    print(self.state_name(), 'completed barrel roll!')
                # target angle was already set to 0, so we don't need to send it again
                return
        else:
            if not self.crossed_90:
                if np.pi < roll < 3 * np.pi / 2:
                    self.crossed_90 = True
                    print(self.state_name(), 'completed 90 degrees!')
                target_angle = roll - np.pi / 2
            elif not self.crossed_270:
                if 0 < roll < np.pi / 2:
                    self.crossed_270 = True
                    print(self.state_name(), 'completed 270 degrees!')
                    target_angle = 0
                else:
                    target_angle = roll - np.pi / 4
            else:
                if self.is_stable(roll, roll_rate):
                    self.completed = True
                    print(self.state_name(), 'completed barrel roll!')
                # target angle was already set to 0, so we don't need to send it again
                return
        controls.set_roll_goal(target_angle)

    @staticmethod
    def is_stable(roll, roll_rate):
        # If within 5 deg of level and |roll_rate| < 2 deg/s
        return (roll < np.radians(5) or roll > np.radians(355)) and np.abs(roll_rate) < np.radians(2)

    def has_completed(self):
        return self.completed
