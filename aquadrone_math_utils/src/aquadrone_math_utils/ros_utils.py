import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion


def ros_time():
    return rospy.Time.now().to_sec()


def make_vector(arr):
    msg = Vector3()
    for var, value in zip('xyz', arr):
        setattr(msg, var, value)
    return msg


def vector_to_np(vector3):
    return np.array([vector3.x, vector3.y, vector3.z])


def make_quaternion(arr, real_first=True):
    msg = Quaternion()
    for var, value in zip('wxyz' if real_first else 'xyzw', arr):
        setattr(msg, var, value)
    return msg


def quaternion_to_np(quaternion, real_first=True):
    return np.array([quaternion.w, quaternion.x, quaternion. y, quaternion.z]) if real_first \
        else np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
