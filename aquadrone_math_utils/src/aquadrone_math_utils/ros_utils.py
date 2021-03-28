import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion, Wrench


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


def make_wrench(arr):
    msg = Wrench()
    msg.force = make_vector(arr[:3])
    msg.torque = make_vector(arr[3:])
    return msg


def wrench_to_np(wrench):
    return np.array([wrench.force.x, wrench.force.y, wrench.force.z,
                     wrench.torque.x, wrench.torque.y, wrench.torque.z])
def msg_quaternion_to_euler(quat):
    q_vec = quaternion_to_np(quat)
    rpy_vec = Quaternion.from_array(q_vec).as_euler()
    return make_vector(rpy_vec)
