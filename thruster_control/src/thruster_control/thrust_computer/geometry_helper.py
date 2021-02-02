import numpy as np

from geometry_msgs.msg import Wrench

import aquadrone_math_utils.orientation_math as OMath


def vector_to_wrench(v):
    w = Wrench()
    w.force.x = v[0]
    w.force.y = v[1]
    w.force.z = v[2]
    w.torque.x = v[3]
    w.torque.y = v[4]
    w.torque.z = v[5]
    return w


def get_thruster_wrench_vector(x, y, z, roll, pitch, yaw):
    # 6d vector, with force on top, moment on bottom
    force = get_thruster_force(roll, pitch, yaw)
    
    # Moment is r x f
    offset = np.array([x, y, z])
    moment = np.cross(offset, force, axis=0)
    # TODO: add torque based on propeller handedness

    return np.concatenate((force, moment))


def get_thruster_force(r, p, y):
    return np.dot(OMath.RPY_Matrix(r, p, y), thruster_force_dir_vec())


def thruster_force_dir_vec():
    # Constant direction that a thruster applies force in
    # Local to the thruster's own body
    v = np.array([1, 0, 0])
    return v

