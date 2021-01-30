#!/usr/bin/env python

import rospkg
import json
import numpy as np

from thruster_control.thrust_computer import geometry_helper as gh


class ThrusterConfiguration:
    """
    This class handles converting between Wrenches and motor thrusts for the various submarine configurations.

    All calculations throughout the thruster_control class are done in the frame of reference of the submarine.
    As such, all commands sent to the thrusters must be provided in the submarine's reference frame.
    For example, a positive x force will push the submarine forward, regardless of the direction that is facing.
    """
    def __init__(self, model='v28'):
        self.model = model
        self.thrust_to_wrench_matrix = None
        self.wrench_to_thrusts_matrix = None

    def initialize(self):
        rospack = rospkg.RosPack()
        with open(rospack.get_path('thruster_control') + "/config/thrusters_" + self.model + ".json") as thruster_json:
            thruster_config = json.load(thruster_json)

        A_cols = [gh.get_thruster_wrench_vector(x=thruster_data['x'], y=thruster_data['y'], z=thruster_data['z'],
                                                roll=np.radians(thruster_data['roll']),
                                                pitch=np.radians(thruster_data['pitch']),
                                                yaw=np.radians(thruster_data['yaw']))
                  for thruster_data in thruster_config['thruster_geometry']]
        A = np.column_stack(A_cols)
        self.thrust_to_wrench_matrix = A

        if A.shape[1] == 6:
            # perfectly actuated
            self.wrench_to_thrusts_matrix = np.linalg.pinv(A)
        elif A.shape[1] > 6:
            # over-actuated
            """
            Since this case is over-actuated, we want to minimize the required power output as long as it gives the 
            required wrench. 
            
            terminology:
             - k: number of thrusters (k > 6 so it is over-actuated)
             - T: vector of thrusts (want to find this), dim kx1
             - W: vector of wrench forces (given), dim 6x1
             - A: matrix mapping thrusts to wrench (known based on geometry), dim 6xk

            Optimization Problem:
              min 0.5*T'*T
              subject to: A*T = W  (eq. 1)

            Has lagrangian:
              0.5*T'*T + v'*(A*T-W)
              v is lagrangian variables, dim 6x1

            Set derivative of lagrangian w.r.t. T to 0:
              T + A'*v = 0 (eq. 2)

            Solve 1 and 2 simultaneously:

            | A   0 | * |T|  =  |W|
            | I   A'|   |v|     |0|
            
            |T|  = inv(| A  0 |) * |W|  === | B C |  * |W| = | B | * W
            |v|       (| I  A'|)   |0|      | D E |    |0|   | D |
            Thus, T = B * W, v = D * W

            Process:
             Initializing:
             - Assemble large matrix on initialization, invert
             - Remove portions of the matrix that would be multiplied by 0 (C and E)
             - Extract portions of the matrix that result in the thrusts (B), since we don't care about v

             Calculating thrusts
             - T = B * W
            """
            thruster_count = A.shape[1]
            block = np.block([[A,                      np.zeros((6, 6))],
                              [np.eye(thruster_count), np.transpose(A)]])
            self.wrench_to_thrusts_matrix = np.linalg.inv(block)[:thruster_count, :6]
        else:
            # under-actuated
            raise NotImplementedError()

    def get_thrusts_to_wrench_matrix(self):
        return self.thrust_to_wrench_matrix

    def wrench_to_thrusts(self, wrench_vec):
        """
        :param wrench_vec: The desired Wrench.
        :return: A vector of size get_num_thrusters() with the individual thrusts needed to achieve the desired Wrench.
        """
        return np.dot(self.wrench_to_thrusts_matrix, wrench_vec)

    def get_num_thrusters(self):
        """
        :return: The number of thrusters in this thruster configuration.
        """
        return self.wrench_to_thrusts_matrix.shape[0]

    @staticmethod
    def get_thruster_class(thruster_index):
        """
        :param thruster_index: The index of the thruster of interest.
                               Must be the case that 0 <= thruster_index < get_num_thrusters())
        :return: The class that should be used for handling the given thruster index.
        """
        from thruster_control.real_thrusters.thruster_types import T100Thruster
        # Currently, only the T100Thruster is implemented
        return T100Thruster
