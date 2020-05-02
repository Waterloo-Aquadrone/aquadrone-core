import math
import numpy as np

from geometry_msgs.msg import Wrench

import aquadrone_math_utils.orientation_math as OH

def vector_to_wrench(v):
    w = Wrench()
    w.force.x = v[0]
    w.force.y = v[1]
    w.force.z = v[2]
    w.torque.x = v[0]
    w.torque.y = v[1]
    w.torque.z = v[2]
    return w

def get_thruster_wrench_vector(x, y, z, roll, pitch, yaw):
    # 6d vector, with force on top, moment on bottom
    force = get_thruster_force(roll, pitch, yaw)
    
    # Moment is r x f
    offset = np.array([x, y, z])
    offset.shape = (3,1)
    moment = np.cross(offset, force, axis=0)

    return np.vstack((force, moment))

def get_thruster_force(r, p, y):
    return np.dot(OH.RPY_Matrix(r,p,y), thruster_force_dir_vec())

def thruster_force_dir_vec():
    # Constant direction that a thruster applies force in
    # Local to the thruster's own body
    v = np.array([1, 0, 0])
    v.shape = (3,1)
    return v

