#!/usr/bin/env python

import numpy as np

import geometry_helper as gh
from thruster_control.T100Thruster import T100Thruster


a2 = np.pi / 2.0
a4 = np.pi / 4.0
in2m = 2.54 * 0.01


def get_configuration(model):
    """
    Note that the resulting configuration is not initialized. You must call the initialize function!
    """
    if model == "v1":
        return V1Configuration()
    elif model == "v2":
        return V2Configuration()
    elif model == "v28":
        return V28Configuration()
    else:
        raise Exception("Error: unknown model for controls: %s" % str(model))


def get_thruster_count(model):
    if model == "v1":
        return V1Configuration.get_num_thrusters()
    elif model == "v2":
        return V2Configuration.get_num_thrusters()
    elif model == "v28":
        return V28Configuration.get_num_thrusters()
    else:
        raise Exception("Error: unknown model for controls: %s" % str(model))


class ThrusterConfiguration:
    """
    Subclasses of this class define all the relevant data for dealing with different thruster configurations.
    """

    def initialize(self):
        raise NotImplementedError

    def get_thrusts_to_wrench_matrix(self):
        raise NotImplementedError

    def wrench_to_thrusts(self, wrench_vec):
        """
        :param wrench_vec: The desired Wrench.
        :return: A vector of size get_num_thrusters() with the individual thrusts needed to achieve the desired Wrench.
        """
        raise NotImplementedError

    @staticmethod
    def get_num_thrusters():
        """
        :return: The number of thrusters in this thruster configuration.
        """
        raise NotImplementedError('Cannot get thruster count for abstract class!')

    @staticmethod
    def get_thruster_class(thruster_index):
        """
        :param thruster_index: The index of the thruster of interest.
                               Must be the case that 0 <= thruster_index < get_num_thrusters())
        :return: The class that should be used for handling the given thruster index.
        """
        # Currently, only the T100Thruster is implemented
        return T100Thruster


class V1Configuration(ThrusterConfiguration):
    def __init__(self):
        self.dX = 36.0 * in2m * 0.5
        self.dY = 12.0 * in2m * 0.5

        self.A_inv = None

    def initialize(self):
        self.A_inv = np.linalg.pinv(self.get_thrusts_to_wrench_matrix())

    def get_thrusts_to_wrench_matrix(self):
        # TODO: read from csv
        th_0 = gh.get_thruster_wrench_vector(x=self.dX,        y=self.dY,   z=0,  roll=0,  pitch=-a2,  yaw=0)
        th_1 = gh.get_thruster_wrench_vector(x=self.dX,   y=-self.dY,  z=0,  roll=0,  pitch=-a2,  yaw=0)
        th_2 = gh.get_thruster_wrench_vector(x=0,              y=self.dY,   z=0,  roll=0,  pitch=0,         yaw=0)
        th_3 = gh.get_thruster_wrench_vector(x=0,              y=-self.dY,  z=0,  roll=0,  pitch=0,         yaw=0)
        th_4 = gh.get_thruster_wrench_vector(x=-self.dX,       y=self.dY,   z=0,  roll=0,  pitch=-a2,  yaw=0)
        th_5 = gh.get_thruster_wrench_vector(x=-self.dX,       y=-self.dY,  z=0,  roll=0,  pitch=-a2,  yaw=0)
        return np.column_stack((th_0, th_1, th_2, th_3, th_4, th_5))

    def wrench_to_thrusts(self, wrench_vec):
        th = np.dot(self.A_inv, wrench_vec)
        return th

    @staticmethod
    def get_num_thrusters():
        return 6


class V2Configuration(ThrusterConfiguration):
    def __init__(self):
        self.dX = 36.0 * in2m * 0.5
        self.dY = 12.0 * in2m * 0.5

        self.A_inv = None

    def initialize(self):
        self.A_inv = np.linalg.pinv(self.get_thrusts_to_wrench_matrix())

    def get_thrusts_to_wrench_matrix(self):
        # TODO: read from csv
        th_0 = gh.get_thruster_wrench_vector(x=self.dX,   y=self.dY,   z=0,  roll=0,  pitch=a4,  yaw=a2)
        th_1 = gh.get_thruster_wrench_vector(x=self.dX,   y=-self.dY,  z=0,  roll=0,  pitch=a4,  yaw=-a2)
        th_2 = gh.get_thruster_wrench_vector(x=0,         y=self.dY,   z=0,  roll=0,  pitch=0,        yaw=0)
        th_3 = gh.get_thruster_wrench_vector(x=0,         y=-self.dY,  z=0,  roll=0,  pitch=0,        yaw=0)
        th_4 = gh.get_thruster_wrench_vector(x=-self.dX,  y=self.dY,   z=0,  roll=0,  pitch=a4,  yaw=a2)
        th_5 = gh.get_thruster_wrench_vector(x=-self.dX,  y=-self.dY,  z=0,  roll=0,  pitch=a4,  yaw=-a2)
        return np.column_stack((th_0, th_1, th_2, th_3, th_4, th_5))

    def wrench_to_thrusts(self, wrench_vec):
        th = np.dot(self.A_inv, wrench_vec)
        return th

    @staticmethod
    def get_num_thrusters():
        return 6


class V28Configuration(ThrusterConfiguration):
    def __init__(self):
        self.dX = 36.0 * in2m * 0.5
        self.dY = 12.0 * in2m * 0.5
        self.dZ = 10.0 * 0.5  # Random Value

        self.bigA_inv = None
        self.full_w_vec = None

    def initialize(self):
        A = self.get_thrusts_to_wrench_matrix()
        O = np.zeros((6, 6))
        I = np.eye(8)
        print(A)

        bigA = np.block([[A, O],
                         [I, np.transpose(A)]])
        np.set_printoptions(precision=1)
        print(bigA)
        self.bigA_inv = np.linalg.inv(bigA)
        self.full_w_vec = np.zeros((8 + 6, 1))

    def get_thrusts_to_wrench_matrix(self):
        # Up-Down Thrusters
        th_0 = gh.get_thruster_wrench_vector(x=0.2,    y=0.285,    z=0.1,  roll=0,  pitch=a2,  yaw=0)
        th_1 = gh.get_thruster_wrench_vector(x=0.2,    y=-0.285,   z=0.1,  roll=0,  pitch=a2,  yaw=0)
        th_2 = gh.get_thruster_wrench_vector(x=-0.2,   y=0.285,    z=0.1,  roll=0,  pitch=a2,  yaw=0)
        th_3 = gh.get_thruster_wrench_vector(x=-0.2,   y=-0.285,   z=0.1,  roll=0,  pitch=a2,  yaw=0)

        # Forwards-Backwards Thrsuters
        th_4 = gh.get_thruster_wrench_vector(x=0,         y=0.285,   z=0.1,  roll=0,  pitch=0,        yaw=0)
        th_5 = gh.get_thruster_wrench_vector(x=0,         y=-0.285,  z=0.1,  roll=0,  pitch=0,        yaw=0)

        # Lef-Right Thrusters
        th_6 = gh.get_thruster_wrench_vector(x=0,    y=0.17,    z=0,  roll=0,  pitch=0,  yaw=-a2)
        th_7 = gh.get_thruster_wrench_vector(x=0,   y=-0.17,    z=0,  roll=0,  pitch=0,  yaw=-a2)

        return np.column_stack((th_0, th_1, th_2, th_3, th_4, th_5, th_6, th_7))

    def wrench_to_thrusts(self, wrench_vec):
        """
        terminology:
         - T: vector of thrusts, dim 8x1
         - W: vector of wrench forces, dim 6x1
         - A: self.A after intialization, see eq 1. dim 6x8

        Optimization Problem:
          min 0.5*T'*T
          subject to: A*T = W  (1)

        Has lagrangian:
          0.5*T'*T + v'*(A*T-W)
          v is lagrangian variables, dim 6x1

        Set derivative of lagrangian w.r.t. T to 0:
          T + A'*v = 0 (2)

        Solve 1 and 2 simultaneously:

        | A   0 | * |T|  =  |W|
        | I   A'|   |v|     |0|

        Process:
         Initializing:
         - Assemble large matrix on initialization, invert
         - Pre-assemble the W and 0 vector

         Calculating thrusts
         - Fill in wrench values
         - Matrix multiplication
         - Extract thrusts
        """
        for i in range(0, 6):
            self.full_w_vec[i] = wrench_vec[i]

        full_thrust_vec = np.dot(self.bigA_inv, self.full_w_vec)

        thrusts = full_thrust_vec[0:8]
        thrusts.shape = (1, 8)
        return thrusts

    @staticmethod
    def get_num_thrusters():
        return 8
