#!/usr/bin/env python

import rospy
import math
import numpy as np

import autograd.numpy as np  # Thinly-wrapped numpy
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, FluidPressure


class PressureSensorListener:
    def __init__(self):
        self.last_time = rospy.Time.now().to_sec()
        self.depth_sub = rospy.Subscriber("aquadrone/out/pressure", FluidPressure, self.depth_cb)

        self.p = 1

        self.z = 0
        self.var = 1

        self.pressure_offset = 100.0
        self.g = 9.8

        self.calc_H = jacobian(self.state_to_measurement)

    def is_valid(self):
        return rospy.Time.now().to_sec() - self.last_time < 0.5

    def get_measurement(self):
        vec = np.zeros((1,1))
        vec[0] = self.z
        return vec

    def get_R(self):
        # Variance of measurements
        var = np.zeros((1,1))
        var[0,0] = self.var
        return var

    def get_H(self, x, u):
        # Jacobian of measurement wrt state (as calculated by get_measurement)
        H = self.calc_H(x, u)
        H = np.reshape(H, (self.p, x.shape[0]))
        return H

    def state_to_measurement(self, x, u):
        return x[0]

    def depth_cb(self, msg):
        press = msg.fluid_pressure
        var = msg.variance

        self.z = -self.pressure_to_depth(press)
        self.var = var / self.g
        self.last_time = rospy.Time.now().to_sec()

    def pressure_to_depth(self, press):
        return (press - self.pressure_offset) / self.g


class EKF:
    def __init__(self):
        # https://en.wikipedia.org/wiki/Extended_Kalman_filter

        self.n = 2
        self.m = 2
        self.x = np.zeros((self.n, 1))
        self.u = np.zeros((self.m,1))

        self.P = np.eye(self.n)
        self.Q = np.eye(self.n) * 0.01

        self.depth_sub = PressureSensorListener()

        self.rate = 10
        self.rate_ctrl = rospy.Rate(self.rate)

        self.calc_F = jacobian(self.f)


    def prediction(self):
        xu = self.stack_xu(self.x, self.u)
        F = self.calc_F(xu)
        F = np.reshape(F, (self.n,self.n + self.m))
        Fx = F[0:self.n, 0:self.n]
        #print("Fx")
        #print(Fx)

        self.x = self.f(self.stack_xu(self.x, self.u))
        inter = np.dot(Fx, self.P)
        self.P = np.dot(  inter,  np.transpose(Fx)  ) + self.Q

        

    def update(self):
        z = np.zeros((0,0))
        h = np.zeros((0,0))

        H = np.zeros((0,0))
        R = np.zeros((0,0))

        def add_block(H, newH):
            if H.shape[0] == 0:
                return newH
            return np.block([H, newH])

        def read_listener(listener, z, h, H, R):
            if listener.is_valid():
                meas = listener.get_measurement()
                z = np.append(z, np.array([meas]))
                pred = listener.state_to_measurement(self.x, self.u)
                h = np.append(h, np.array([pred]))

                H = add_block(H, listener.get_H(self.x, self.u))
                R = add_block(R, listener.get_R())

                return z, h, H, R

        z, h, H, R = read_listener(self.depth_sub, z, h, H, R)

        if R.shape[0] == 0:
            return

        y = z - h
        y.shape = (y.shape[0], 1)

        Ht = np.transpose(H)
        S = np.dot(np.dot(H, self.P), Ht) + R
        K = np.dot(np.dot(self.P, Ht), np.linalg.inv(S))

        KH = np.dot(K, H)
        I = np.eye(KH.shape[0])

        diff = np.dot(K, y)

        self.x = self.x + diff
        self.P = np.dot(I - KH, self.P)

    def stack_xu(self, x, u):
        return np.concatenate((x, u), axis=0)

    def split_xu(self, xu):
        x = xu[0:self.n]
        u = xu[self.n:]
        return x,u

    def f(self, xu):
        x, u = self.split_xu(xu)
        n = x.shape[0]

        #return np.dot(1.0*np.eye(n), x) + np.dot(2.0*np.eye(m), u)
        return np.dot(np.array([[1, 1.0/self.rate], [0, 1]]), x) + 3.0*u

    def output(self):
        print("X")
        print(self.x)
        print("P")

        print(" ")
        print(self.P)

    def run(self):
        while not rospy.is_shutdown():
            self.rate_ctrl.sleep()
            self.output()
            self.prediction()

            self.update()
            
            

if __name__ == "__main__":
    rospy.init_node("simple_state_estimation")

    ekf = EKF()
    ekf.run()