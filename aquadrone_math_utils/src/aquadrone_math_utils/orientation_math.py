import autograd.numpy as np  # Thinly-wrapped numpy
import math

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
        # From wikipedia (https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

        w = quat_vec[0]
        x = quat_vec[1]
        y = quat_vec[2]
        z = quat_vec[3]

        angles = np.array([0, 0 ,0])

        # roll (x-axis rotation)
        sinr_cosp = +2.0 * (w * x + y * z)
        cosr_cosp = +1.0 - 2.0 * (x * x + y * y)
        #angles[0] = np.arctan2(sinr_cosp, cosr_cosp)
        

        # yaw (z-axis rotation)
        siny_cosp = +2.0 * (w * z + x * y)
        cosy_cosp = +1.0 - 2.0 * (y * y + z * z)
        #angles[2] = math.atan2(siny_cosp, cosy_cosp)

        # pitch (y-axis rotation)
        sinp = +2.0 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            #angles[1] = np.sign(sinp) * math.pi # use 90 degrees if out of range
            return np.array([ np.arctan2(sinr_cosp, cosr_cosp),
                              np.sign(sinp) * math.pi,
                              np.arctan2(siny_cosp, cosy_cosp)
                              ])
        else:
            #angles[1] = math.asin(sinp)
            return np.array([ np.arctan2(sinr_cosp, cosr_cosp),
                              np.arcsin(sinp),
                              np.arctan2(siny_cosp, cosy_cosp)
                              ])

        return angles

def Yaw(y):
    return np.array([
            [np.cos(y), -np.sin(y), 0.0],
            [np.sin(y),  np.cos(y), 0.0],
            [        0.0,          0.0, 1.0]
            ])

def Pitch(p):
        return np.array([
                [ np.cos(p), 0.0, np.sin(p)],
                [         0.0, 1.0,         0.0],
                [-np.sin(p), 0.0, np.cos(p)]
                ])

def Roll(r):
        return np.array([
                [1.0,         0.0,          0.0],
                [0.0, np.cos(r), -np.sin(r)],
                [0.0, np.sin(r),  np.cos(r)]
                ])

def RPY_Matrix(r, p, y):
    r = Roll(r)
    p = Pitch(p)
    y = Yaw(y)
    rpy = np.linalg.multi_dot([y, p, r])
    return rpy