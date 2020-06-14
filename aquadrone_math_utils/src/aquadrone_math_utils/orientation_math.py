import autograd.numpy as np  # Thinly-wrapped numpy
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Vector3


"""
Throughout the Aquadrone code base, orientations are specified in one of 2 formats:
1. Extrinsic ZYX Euler angles (yaw, pitch, and roll respectively). 
This corresponds to the human intuition of yaw, then pitch about new y axis, then roll about new x axis.
2. Unit quaternion

Angular velocities are specified in one of 2 formats:
1. roll, pitch and yaw rates. These are in rad/s along the submarine's relative ZYX axes.
2. A vector [Wx, Wy, Wz] which points in the direction of the rotation axis in the static
   coordinate system and has a magnitude equal to the rotation rate in rad/s (following the right hand rule).
"""


def quat_msg_to_vec(q):
    return np.array([q.w, q.x, q.y, q.z])


def rpy_vec_to_msg(vec):
    angles = Vector3()
    angles.x, angles.y, angles.z = vec
    return angles


def msg_quaternion_to_euler(quat):
    q_vec = quat_msg_to_vec(quat)
    rpy_vec = quaternion_to_euler(q_vec)
    return rpy_vec_to_msg(rpy_vec)


def quaternion_to_euler(quat_vec):
    return Rotation.from_quat(quat_vec).as_euler('ZYX')[::-1]


def euler_to_quaternion(euler_vec):
    return Rotation.from_euler('ZYX', np.asarray(euler_vec)[::-1]).as_quat()
