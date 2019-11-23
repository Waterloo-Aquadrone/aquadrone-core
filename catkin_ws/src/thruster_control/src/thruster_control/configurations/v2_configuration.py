import numpy as np
import math

import geometry_helper as gh

class V2Configuration:
    def __init__(self):
        self.in2m = 2.54 * 0.01

        self.dX = 36.0 * self.in2m * 0.5
        self.dY = 12.0 * self.in2m * 0.5

        self.a2 = math.pi / 2.0
        self.a4 = math.pi / 4.0

    def initialize(self):
        pass

    def get_thrusts_to_wrench_matrix(self):
        th_0 = gh.get_thruster_wrench_vector(x=self.dX,   y=self.dY,   z=0,  roll=0,  pitch=self.a4,  yaw=self.a2)
        th_1 = gh.get_thruster_wrench_vector(x=self.dX,   y=-self.dY,  z=0,  roll=0,  pitch=self.a4,  yaw=-self.a2)
        th_2 = gh.get_thruster_wrench_vector(x=0,         y=self.dY,   z=0,  roll=0,  pitch=0,        yaw=0)
        th_3 = gh.get_thruster_wrench_vector(x=0,         y=-self.dY,  z=0,  roll=0,  pitch=0,        yaw=0)
        th_4 = gh.get_thruster_wrench_vector(x=-self.dX,  y=self.dY,   z=0,  roll=0,  pitch=self.a4,  yaw=self.a2)
        th_5 = gh.get_thruster_wrench_vector(x=-self.dX,  y=-self.dY,  z=0,  roll=0,  pitch=self.a4,  yaw=-self.a2)
        return np.column_stack((th_0, th_1, th_2, th_3, th_4, th_5))

    def get_wrench_to_thrusts_lb_in(self):
        return np.linalg.pinv(self.get_thrusts_to_wrench())
