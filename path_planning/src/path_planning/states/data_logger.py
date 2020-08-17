#!/usr/bin/env python

from time import time
import numpy as np
from path_planning.states.base_state import BaseState


class DataLogger(BaseState):
    """
    This state listens to positional information about the submarine and saves it.
    Once it is finalized, it will save the data to a csv file.
    This state does not send any commands to any of the systems.
    """

    def __init__(self):
        self.data = []
        self.completed = False

    @staticmethod
    def state_name():
        return "data_logger"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.completed = False
        self.data = []
        print(self.state_name(), 'starting to log data')

    def process(self, t, controls, sub_state, world_state, sensors):
        self.data.append([t, sub_state.get_submarine_state().z])

    def finalize(self, t, controls, sub_state, world_state, sensors):
        np.savetxt('log-' + str(time()) + '.csv', self.data, delimiter=',')
        self.completed = True

    def has_completed(self):
        return self.completed
