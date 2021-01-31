import numpy as np

from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Vector3


def quat_msg_to_vec(q):
    return np.array([q.w, q.x, q.y, q.z])


def rpy_vec_to_msg(vec):
    angles = Vector3()
    angles.x = vec[0]
    angles.y = vec[1]
    angles.z = vec[2]
    return angles


def msg_quaternion_to_euler(quat):
    q_vec = quat_msg_to_vec(quat)
    rpy_vec = quaternion_to_euler(q_vec)
    return rpy_vec_to_msg(rpy_vec)


def quaternion_to_euler(quat_vec):
    euler = Rotation.from_quat(quat_vec).as_euler('ZYX')[::-1]
    euler[1] *= -1
    return euler


def euler_to_quat(euler):
    euler[1] *= -1
    return Rotation.from_euler('ZYX', euler[::-1]).as_quat()


def RPY_Matrix(r, p, y, degrees=False):
    """
    Roll, pitch, and yaw angles are defined as the intrinsic Euler angles.
    The separate rotations are applied in the order: yaw, pitch (about new y axis), roll (about new x axis).
    The pitch angle is negated to align with typical representations for submarines.

    Note that the inverse of a rotation matrix is the same as its transpose,
    although the transpose is much easier to compute.
    """
    return Rotation.from_euler('ZYX', np.array([y, -p, r]), degrees=degrees).as_matrix()
