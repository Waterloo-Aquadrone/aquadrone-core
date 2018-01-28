import math
import cv2
import numpy as np
import scipy.misc
from simple_pid import PID

from base_state import BaseState
from aquadrone_msgs.msg import MotorControls


class CheckThrustersState(BaseState):
    def __init__(self, time_each=1):
        self.current_thruster = 0
        self.thruster_mode = -1

        self.time_each = time_each
        self.time_mode_started = 0

    def state_name(self):
        return "check_thrusters_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        # TODO: disable automatic control system
        self.time_mode_started = t
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        # TODO: re-enable automatic control system
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        # Go through all 8 thrusters once
        if self.current_thruster > 7:
            return

        # Alternate between -1, 0, 1 for 1 second each
        dt = t - self.time_mode_started
        if dt > self.time_each:
            self.time_mode_started = t

            if self.thruster_mode == 1:
                self.thruster_mode = -1
                self.current_thruster = self.current_thruster + 1
                return
            else:
                self.thruster_mode = self.thruster_mode + 1
        
        thrusts = [0 for i in range(0, 8)]
        thrusts[self.current_thruster] = self.thruster_mode

        controls.raw_thrust_command(thrusts)      

    

    def all_thrusters_tested(self):
        return self.current_thruster > 7
