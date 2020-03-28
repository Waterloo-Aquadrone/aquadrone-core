import numpy as np
import math

#import geometry_helper as gh

from geometry_msgs.msg import Wrench

import aquadrone_math_utils.orientation_math as OH

import thruster_control.geometry_helper as gh

a2 = math.pi / 2.0
a4 = math.pi / 4.0
in2m = 2.54 * 0.01
'''
class GeometryHelper:


    def vector_to_wrench(v):
        w = Wrench()
        w.force.x = v[0]
        w.force.y = v[1]
        w.force.z = v[2]
        w.torque.x = v[0]
        w.torque.y = v[1]
        w.torque.z = v[2]
        return w

    def get_thruster_wrench_vector(x, y, z, roll, pitch, yaw):
        # 6d vector, with force on top, moment on bottom
        force = get_thruster_force(roll, pitch, yaw)

       # Moment is r x f
       offset = np.array([x, y, z])
       offset.shape = (3,1)
       moment = np.cross(offset, force, axis=0)

       return np.vstack((force, moment))

    def get_thruster_force(r, p, y):
       return np.dot(OH.RPY_Matrix(r,p,y), thruster_force_dir_vec())

    def thruster_force_dir_vec():
        # Constant direction that a thruster applies force in
        # Local to the thruster's own body
        v = np.array([1, 0, 0])
        v.shape = (3,1)
        return v
'''

#gh = GeometryHelper()


class ThrusterConfiguration:
    def __init__(self):
        raise NotImplementedError
    def initialize(self):
        raise NotImplementedError
    def get_thrusts_to_wrench_matrix(self):
        raise NotImplementedError
    def wrench_to_thrusts(self):
        raise NotImplementedError


class V1Configuration(ThrusterConfiguration):
    def __init__(self):
        self.dX = 36.0 * in2m * 0.5
        self.dY = 12.0 * in2m * 0.5

        self.A_inv = None

    def initialize(self):
        self.A_inv = np.linalg.pinv(self.get_thrusts_to_wrench_matrix())


    def get_thrusts_to_wrench_matrix(self):
        th_0 = gh.get_thruster_wrench_vector(x=self.dX,        y=self.dY,   z=0,  roll=0,  pitch=-a2,  yaw=0)
        th_1 = gh.get_thruster_wrench_vector(x=self.dX,   y=-self.dY,  z=0,  roll=0,  pitch=-a2,  yaw=0)
        th_2 = gh.get_thruster_wrench_vector(x=0,              y=self.dY,   z=0,  roll=0,  pitch=0,         yaw=0)
        th_3 = gh.get_thruster_wrench_vector(x=0,              y=-self.dY,  z=0,  roll=0,  pitch=0,         yaw=0)
        th_4 = gh.get_thruster_wrench_vector(x=-self.dX,       y=self.dY,   z=0,  roll=0,  pitch=-a2,  yaw=0)
        th_5 = gh.get_thruster_wrench_vector(x=-self.dX,       y=-self.dY,  z=0,  roll=0,  pitch=-a2,  yaw=0)
        return np.column_stack((th_0, th_1, th_2, th_3, th_4, th_5))

    def wrench_to_thrusts(self, wrench_vec):
        th =  np.dot(self.A_inv, wrench_vec)
        return th

    def get_num_thrusters(self):
        return 6


class V2Configuration(ThrusterConfiguration):
    def __init__(self):
        self.dX = 36.0 * in2m * 0.5
        self.dY = 12.0 * in2m * 0.5

        self.A_inv = None

    def initialize(self):
        self.A_inv = np.linalg.pinv(self.get_thrusts_to_wrench_matrix())

    def get_thrusts_to_wrench_matrix(self):
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

    def get_num_thrusters(self):
        return 6


class V28Configuration(ThrusterConfiguration):
    def __init__(self):
        self.dX = 36.0 * in2m * 0.5
        self.dY = 12.0 * in2m * 0.5
        self.dZ = 10.0 * 0.5 # Random Value

        self.bigA_inv = None
        self.full_w_vec = None

    def initialize(self):
        A = self.get_thrusts_to_wrench_matrix()
        O = np.zeros((6,6))
        I = np.eye(8)
        print(A)

        bigA = np.block([[A, O], [I, np.transpose(A)]])
        np.set_printoptions(precision=1)
        print(bigA)
        self.bigA_inv = np.linalg.inv(bigA)
        self.full_w_vec = np.zeros((8+6,1))

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
        '''
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
        '''
        for i in range(0, 6):
            self.full_w_vec[i] = wrench_vec[i]

        full_thrust_vec  = np.dot(self.bigA_inv, self.full_w_vec)

        thrusts = full_thrust_vec[0:8]
        thrusts.shape = (1,8)
        return thrusts

    def get_num_thrusters(self):
        return 8
