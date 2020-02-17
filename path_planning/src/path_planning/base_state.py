import math
import cv2
import numpy as np
import scipy.misc
from simple_pid import PID

class BaseState(object):
    def __init__(self):
        pass

    def state_name(self):
        return "base_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        # Set up anything that needs initializing
        # Run EACH time the state is chosen as the next state
        # process(...) will be called with the next available data
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        # Clean up anything necessary
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        # Regular tick at some rate
        pass

    # Expose functions to identify when a state should exit
    # Should not require any arguments to call
    # Ex: has_timed_out, has_lost_track_of_[some object]


class InitialState(BaseState):
    def __init__(self):
        pass

    def state_name(self):
        return "init_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        pass