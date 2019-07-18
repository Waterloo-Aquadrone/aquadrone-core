import numpy as np
import math

import geometry_helper as gh

def get_thrusts_to_wrench():

    in2m = 2.54 * 0.01

    dX = 36.0 * in2m * 0.5
    dY = 12.0 * in2m * 0.5

    a2 = math.pi / 2.0
    a4 = math.pi / 4.0

    th_0 = gh.get_thruster_wrench_vector(x=dX,   y=dY,   z=0,  roll=0,  pitch=-a2,  yaw=0)
    th_1 = gh.get_thruster_wrench_vector(x=dX,   y=-dY,  z=0,  roll=0,  pitch=-a2,  yaw=0)
    th_2 = gh.get_thruster_wrench_vector(x=0,    y=dY,   z=0,  roll=0,  pitch=0,    yaw=0)
    th_3 = gh.get_thruster_wrench_vector(x=0,    y=-dY,  z=0,  roll=0,  pitch=0,    yaw=0)
    th_4 = gh.get_thruster_wrench_vector(x=-dX,  y=dY,   z=0,  roll=0,  pitch=-a2,  yaw=0)
    th_5 = gh.get_thruster_wrench_vector(x=-dX,  y=-dY,  z=0,  roll=0,  pitch=-a2,  yaw=0)
    return np.column_stack((th_0, th_1, th_2, th_3, th_4, th_5))

def get_wrench_to_thrusts_lb_in():

    return np.linalg.pinv(get_thrusts_to_wrench())