#!/usr/bin/env python
import numpy as np

from thruster_control.configurations.v1_configuration import V1Configuration
from thruster_control.configurations.v2_configuration import V2Configuration

v1 = V1Configuration()
v2 = V2Configuration()

np.set_printoptions(precision=2)


T2W = v2.get_thrusts_to_wrench()
print("T2W (new)")
print(T2W)
print("T2W rank")
print(np.linalg.matrix_rank(T2W))
print("T2W cond")
print(np.linalg.cond(T2W))

print("T2W eig")
w,v = np.linalg.eig(T2W)
print(w)

W2T = v2.get_wrench_to_thrusts_lb_in()
print(W2T)

print("W2T cond")
print(np.linalg.cond(W2T))

print("Forward")
T = np.dot(W2T, np.array([[1], [0], [0], [0], [0], [0]]))
print(np.linalg.norm(T, ord=1))

print("Side")
T = np.dot(W2T, np.array([[0], [1], [0], [0], [0], [0]]))
print(np.linalg.norm(T, ord=1))

print("Vertical")
T = np.dot(W2T, np.array([[0], [0], [1], [0], [0], [0]]))
print(np.linalg.norm(T, ord=1))



print("Roll")
T = np.dot(W2T, np.array([[0], [0], [0], [1], [0], [0]]))
print(np.linalg.norm(T, ord=1))

print("Pitch")
T = np.dot(W2T, np.array([[0], [0], [0], [0], [1], [0]]))
print(np.linalg.norm(T, ord=1))

print("Yaw")
T = np.dot(W2T, np.array([[0], [0], [0], [0], [0], [1]]))
print(np.linalg.norm(T, ord=1))