import autograd.numpy as np  # Thinly-wrapped numpy
import math
from aquadrone_math_utils.ros_utils import make_vector, quaternion_to_np
import sympy as sp
# TODO: replace with scipy Rotations
"""
Throughout the Aquadrone code base, orientations are specified in one of 2 formats:
1. Intrinsic ZYX Euler angles (yaw, pitch, and roll respectively). 
This corresponds to the human intuition of yaw, then pitch about new y axis, then roll about new x axis.
2. Unit quaternion

Angular velocities are specified in one of 2 formats:
1. roll, pitch and yaw rates. These are in rad/s along the submarine's relative ZYX axes.
2. A vector [Wx, Wy, Wz] which points in the direction of the rotation axis in the static
   coordinate system and has a magnitude equal to the rotation rate in rad/s (following the right hand rule).
"""


def msg_quaternion_to_euler(quat):
    q_vec = quaternion_to_np(quat)
    rpy_vec = quaternion_to_euler(q_vec)
    return make_vector(rpy_vec)


def quaternion_to_euler(quat_vec):
    # doi:10.1.1.468.5407 p16
    w, x, y, z = quat_vec / np.sum(quat_vec*quat_vec)**0.5

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1 - 2.0 * (x * x + y * y)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * ( w * z + x * y )
    cosy_cosp = 1 - 2.0 * ( y * y + z * z )

    # pitch (y-axis rotation)
    sinp = 2.0 * (-x * z + w * y)

    if np.abs(sinp) == 1:
        return [-2 * sp.sign(sinp) * sp.atan2(x, w).evalf(),
                 sp.sign(sinp) * math.pi / 2,
                 0
                 ]
    else:
        return [sp.atan2(siny_cosp, cosy_cosp).evalf(),
                 sp.asin(sinp).evalf(),
                 sp.atan2(sinr_cosp, cosr_cosp).evalf()
                 ]


def Yaw(y):
    return np.array([
        [np.cos(y), -np.sin(y), 0.0],
        [np.sin(y), np.cos(y), 0.0],
        [0.0, 0.0, 1.0]
    ])


def Pitch(p):
    return np.array([
        [np.cos(p), 0.0, np.sin(p)],
        [0.0, 1.0, 0.0],
        [-np.sin(p), 0.0, np.cos(p)]
    ])


def Roll(r):
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(r), -np.sin(r)],
        [0.0, np.sin(r), np.cos(r)]
    ])


def RPY_Matrix(r, p, y):
    r = Roll(r)
    p = Pitch(p)
    y = Yaw(y)
    rpy = np.linalg.multi_dot([y, p, r])
    return rpy
