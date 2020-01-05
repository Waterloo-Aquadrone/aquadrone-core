#!/usr/bin/env python

import rospy
import math
import numpy as np
import scipy.linalg

import autograd.numpy as np  # Thinly-wrapped numpy
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, FluidPressure

from aquadrone_msgs.msg import SubState, MotorControls

from ekf_indices import IDx as IDx

class BaseSensorListener(object):
    def __init__(self):
        self.last_time = rospy.Time.now().to_sec()
        self.calc_H = jacobian(self.state_to_measurement)

    def is_valid(self):
        return rospy.Time.now().to_sec() - self.last_time < self.get_timeout_sec()

    def get_H(self, x, u):
        # Jacobian of measurement wrt state (as calculated by get_measurement)
        H = self.calc_H(x, u)
        H = np.reshape(H, (self.get_p(), x.shape[0]))
        return H

    def get_timeout_sec(self):
        raise NotImplementedError
    def get_p(self):
        raise NotImplementedError
    def get_measurement(self):
        raise NotImplementedError
    def get_R(self):
        raise NotImplementedError
    def state_to_measurement(self, x, u):
        raise NotImplementedError



class PressureSensorListener(BaseSensorListener):
    def __init__(self):
        super(PressureSensorListener, self).__init__()
        self.depth_sub = rospy.Subscriber("aquadrone/out/pressure", FluidPressure, self.depth_cb)

        self.z = 0
        self.var = 1

        self.pressure_offset = 100.0
        self.g = 9.8

    def get_timeout_sec(self):
        return 0.1

    def get_p(self):
        return 1

    def get_measurement(self):
        vec = np.zeros((1,1))
        vec[0] = self.z
        return vec

    def get_R(self):
        # Variance of measurements
        var = np.zeros((1,1))
        var[0,0] = self.var
        return var

    def state_to_measurement(self, x, u):
        return x[IDx.Pz]

    def depth_cb(self, msg):
        press = msg.fluid_pressure
        var = msg.variance

        self.z = -self.pressure_to_depth(press)
        self.var = var / self.g
        self.last_time = rospy.Time.now().to_sec()

    def pressure_to_depth(self, press):
        return (press - self.pressure_offset) / self.g


class IMUSensorListener(BaseSensorListener):
    def __init__(self):
        super(IMUSensorListener, self).__init__()
        self.imu_sub = rospy.Subscriber("aquadrone/out/imu", Imu, self.imu_cb)

        self.orientation = np.array([1, 0, 0, 0])
        self.orientation_var = np.array([1, 1, 1, 1])

        self.pressure_offset = 100.0
        self.g = 9.8

    def get_timeout_sec(self):
        return 0.1

    def get_p(self):
        return 4

    def get_measurement(self):
        vec = np.zeros((4,1))
        vec[0:4] = self.orientation
        return vec

    def get_R(self):
        # Variance of measurements
        var = np.zeros((4,4))
        for i in range(0, 4):
            var[i,i] = self.orientation_var[i]
        return var

    def state_to_measurement(self, x, u):
        z =  np.array( [ x[IDx.Ow],
                         x[IDx.Ox],
                         x[IDx.Oy],
                         x[IDx.Oz] ])
        #z.shape = (4,1)
        return z

    def imu_cb(self, msg):
        self.orientation = np.array([ msg.orientation.w,
                                      msg.orientation.x,
                                      msg.orientation.y,
                                      msg.orientation.z ])[np.newaxis]
        self.orientation.shape = (4, 1)
        self.orientation_var = np.array([ msg.orientation_covariance[0],
                                          msg.orientation_covariance[0],
                                          msg.orientation_covariance[0],
                                          msg.orientation_covariance[0] ])
        self.last_time = rospy.Time.now().to_sec()
