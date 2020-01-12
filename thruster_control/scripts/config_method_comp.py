#!/usr/bin/env python
import numpy as np

from thruster_control.configurations.v2_configuration import get_wrench_to_thrusts_lb_in
from thruster_control.configurations.v2_configuration import get_thrusts_to_wrench



W2T = get_wrench_to_thrusts_lb_in()

np.set_printoptions(precision=2)

print("W2T (orig)")
print(W2T)

T2W = get_thrusts_to_wrench()
print("T2W (new)")
print(np.linalg.matrix_rank(T2W))
print(np.linalg.pinv(T2W))