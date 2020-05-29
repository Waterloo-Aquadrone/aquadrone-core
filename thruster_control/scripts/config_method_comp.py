#!/usr/bin/env python
import numpy as np

from thrust_computer.thruster_configurations import V28Configuration

config = V28Configuration()

T2W = config.get_thrusts_to_wrench()
print("T2W (new)")
print(np.linalg.matrix_rank(T2W))
print(np.linalg.pinv(T2W))

W2T = np.linalg.inv(T2W)

np.set_printoptions(precision=2)

print("W2T (orig)")
print(W2T)