import math
import cv2
import numpy as np
import scipy.misc
from simple_pid import PID

from base_state import BaseState



class WaitForSubmergenceState(BaseState):
    def __init__(self):
        self.current_depth = 0

        self.time_required = 3

        self.time_submerged = 0
        self.last_t = 0

    def state_name(self):
        return "wait_for_submergence_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        # TODO: disable motor controls while out of water
        self.last_t = t

    def finalize(self, t, controls, sub_state, world_state, sensors):
        # TODO: re-enable motor controls
        # TODO: reset state estimation
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        self.current_depth = -sub_state.get_submarine_state().position.z
        dt = t - self.last_t
        self.last_t = t

        if self.current_depth  > 0.25:
            self.time_submerged = self.time_submerged + dt
        else:
            self.time_submerged = 0


    def has_submerged(self):
        return self.time_submerged > self.time_required


class TurnOffForRetrievalState(BaseState):
    def __init__(self):
        self.turned_off = False

    def state_name(self):
        return "turn_off_for_retrieval_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        # TODO: disable motor controls while out of water
        self.turned_off = True

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_turned_off(self):
        return self.turned_off


class GoToDepthState(BaseState):
    def __init__(self, d_goal, t_required=5):
        self.depth_goal = d_goal
        self.current_depth = 0

        self.time_required = t_required

        self.time_at_depth = 0
        self.last_t = 0

    def state_name(self):
        return "go_to_depth_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        controls.set_depth_goal(self.depth_goal)
        self.last_t = t

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        self.current_depth = -sub_state.get_submarine_state().position.z
        dt = t - self.last_t
        self.last_t = t

        depth_err = abs(self.current_depth - self.depth_goal)

        if depth_err < 0.25:
            self.time_at_depth = self.time_at_depth + dt
        else:
            self.time_at_depth = 0

        print("Depth err (abs): %f" % depth_err)

    def depth_is_reached(self):
        return self.time_at_depth > self.time_required


class GoToSurfaceState(BaseState):
    def __init__(self):
        self.depth_goal = 0
        self.current_depth = 0

        self.time_required = 2

        self.time_at_surface = 0
        self.last_t = 0

    def state_name(self):
        return "go_to_surface"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        controls.set_depth_goal(self.depth_goal)
        self.last_t = t

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        self.current_depth = -sub_state.get_submarine_state().position.z
        dt = t - self.last_t
        self.last_t = t

        if self.current_depth < 0.5:
            self.time_at_surface = self.time_at_surface + dt
        else:
            self.time_at_surface = 0

    def has_surfaced(self):
        return self.time_at_surface > self.time_required
