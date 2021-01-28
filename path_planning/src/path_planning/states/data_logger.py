#!/usr/bin/env python

from time import time
import rospy
import rospkg
import numpy as np
import pandas as pd
from path_planning.states.base_state import BaseState


class DataLogger(BaseState):
    """
    This state listens to positional information about the submarine and saves it.
    Once it is finalized, it will save the data to a csv file.
    This state does not send any commands to any of the systems.
    """

    def __init__(self, file_name='submarine-log'):
        self.file_name = file_name
        self.data = []
        self.completed = False
        rospy.on_shutdown(self.shutdown_hook)
        self.output_dir = rospkg.RosPack().get_path('path_planning')
        self.data_post_processing_funcs = []

    def add_data_post_processing_func(self, data_post_processing_func):
        """
        :param data_post_processing_func: A function that receives the logged data and processes it as needed.
        """
        self.data_post_processing_funcs.append(data_post_processing_func)

    def shutdown_hook(self):
        """
        This is special to this specific state. If ros is externally shut down,
        save the data before exiting in this shutdown hook.
        """
        if not self.completed and len(self.data) > 0:
            self.save_data()

    def state_name(self):
        return "data_logger"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.completed = False
        self.data = []
        print(self.state_name(), 'starting to log data')

    def process(self, t, controls, sub_state, world_state, sensors):
        # for now just log the depth
        sub = sub_state.get_submarine_state()
        self.data.append([t, sub.position.x, sub.position.y, sub.position.z,
                          sub.orientation_rpy.x, sub.orientation_rpy.y, sub.orientation_rpy.z])

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.save_data()
        self.completed = True

    def save_data(self):
        np.savetxt(self.output_dir + '/' + self.file_name + '-' + str(time()) + '.csv', self.data, delimiter=',',
                   header='t, x, y, z, r, p, y\n')

        if len(self.data_post_processing_funcs) > 0:
            # convert to data frame before passing to post_processing_funcs
            self.data = pd.DataFrame(self.data, columns=['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
            for data_post_processing_func in self.data_post_processing_funcs:
                data_post_processing_func(self.data)

    def has_completed(self):
        return self.completed
