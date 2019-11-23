import numpy as np
import math

import geometry_helper as gh

class V28Configuration:
    def __init__(self):
        self.in2m = 2.54 * 0.01

        self.dX = 36.0 * self.in2m * 0.5
        self.dY = 12.0 * self.in2m * 0.5
        self.dZ = 10.0 * 0.5 # Random Value

        self.a2 = math.pi / 2.0
        self.a4 = math.pi / 4.0

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
        th_0 = gh.get_thruster_wrench_vector(x=self.dX,    y=self.dY,    z=-self.dZ,  roll=0,  pitch=self.a2,  yaw=0)
        th_1 = gh.get_thruster_wrench_vector(x=self.dX,    y=-self.dY,   z=-self.dZ,  roll=0,  pitch=self.a2,  yaw=0)
        th_2 = gh.get_thruster_wrench_vector(x=-self.dX,   y=self.dY,    z=-self.dZ,  roll=0,  pitch=self.a2,  yaw=0)
        th_3 = gh.get_thruster_wrench_vector(x=-self.dX,   y=-self.dY,   z=-self.dZ,  roll=0,  pitch=self.a2,  yaw=0)

        # Forwards-Backwards Thrsuters
        th_4 = gh.get_thruster_wrench_vector(x=0,         y=self.dY,   z=0,  roll=0,  pitch=0,        yaw=0)
        th_5 = gh.get_thruster_wrench_vector(x=0,         y=-self.dY,  z=0,  roll=0,  pitch=0,        yaw=0)

        # Lef-Right Thrusters
        th_6 = gh.get_thruster_wrench_vector(x=self.dX,    y=0,    z=self.dZ,  roll=0,  pitch=0,  yaw=-self.a2)
        th_7 = gh.get_thruster_wrench_vector(x=-self.dX,   y=0,    z=self.dZ,  roll=0,  pitch=0,  yaw=-self.a2)


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
        return thrusts

if __name__ == "__main__":
    v = V28Configuration()
    v.initialize()