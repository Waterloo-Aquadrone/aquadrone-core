import numpy as np
import math

def Yaw(y):
    return np.matrix([
            [math.cos(y), -math.sin(y), 0],
            [math.sin(y),  math.cos(y), 0],
            [          0,            0, 1]
            ])

def Pitch(p):
        return np.matrix([
                [ math.cos(p), 0, math.sin(p)],
                [               0, 1,               0],
                [-math.sin(p), 0, math.cos(p)]
                ])

def Roll(r):
        return np.matrix([
                [1,              0,               0],
                [0, math.cos(r), -math.sin(r)],
                [0, math.sin(r),  math.cos(r)]
                ])

def RPY_Matrix(r, p, y):
    r = Roll(r)
    p = Pitch(p)
    y = Yaw(y)

    interm = np.dot(y,p)
    return np.dot(interm, r)