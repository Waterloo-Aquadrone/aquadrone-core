#!/usr/bin/env python2.7

import rospy

import numpy as np  # only used for local testing, must use autograd wrapper to actually run this
# import autograd.numpy as np  # Thinly-wrapped numpy
from autograd import jacobian

from sensor_msgs.msg import Imu, FluidPressure

from ekf_indices import Idx


class BaseSensorListener:
    # https://en.wikipedia.org/wiki/Extended_Kalman_filter

    def __init__(self, parent_ekf):
        self.last_time = rospy.Time.now().to_sec()
        self.h_jacobian_func = jacobian(self.state_to_measurement_h)
        self.parent_ekf = parent_ekf

    @staticmethod
    def get_t():
        return rospy.Time.now().to_sec()

    def is_valid(self):
        return self.get_t() - self.last_time < self.get_timeout_sec()

    def get_h_jacobian(self, x, u):
        """
        Must return a matrix of shape (self.get_p(), n) where x is of shape (n, n)

        :param x: The current state.
        :param u: The current control inputs.
        :return: Jacobian of self.state_to_measurement_h(x, u)
        """
        # Jacobian of measurement with respect to state (as calculated by get_measurement_z)
        return self.h_jacobian_func(x, u)

    def get_timeout_sec(self):
        # Amount of time to use old measurements for
        raise NotImplementedError

    def get_p(self):
        # Number of elements in measurement vector
        raise NotImplementedError

    def get_measurement_z(self):
        """
        Must be a vector of length self.get_p()

        :return: The most recent reading of the actual measurement of the sensor.
        """
        raise NotImplementedError

    def get_R(self):
        """
        Must be a matrix with shape (self.get_p(), self.get_p())

        :return: The uncertainty matrix of measurements.
        """
        raise NotImplementedError

    def state_to_measurement_h(self, x, u):
        """
        Must return an array of length self.get_p()
        Must be autograd-able

        :param x: The current state.
        :param u: The current control inputs.
        :return: The expected sensor readings based on the given state and inputs.
        """
        raise NotImplementedError


class PressureSensorListener(BaseSensorListener):
    def __init__(self, parent_ekf):
        super().__init__(parent_ekf)
        rospy.Subscriber("aquadrone/out/pressure", FluidPressure, self.depth_cb)

        self.z = 0
        self.variance = 1

        self.pressure_offset = 100.0
        self.g = 9.8

    def get_timeout_sec(self):
        return 0.1

    def get_p(self):
        return 1

    def get_measurement_z(self):
        return np.array([self.z])

    def get_R(self):
        # Variance of measurements
        return np.array([self.variance])

    def state_to_measurement_h(self, x, u):
        return x[Idx.z]

    def depth_cb(self, msg):
        pressure = msg.fluid_pressure
        variance = msg.variance

        self.z = -self.pressure_to_depth(pressure)
        self.variance = variance / self.g
        self.last_time = self.get_t()

    def pressure_to_depth(self, pressure):
        return (pressure - self.pressure_offset) / self.g


class IMUSensorListener(BaseSensorListener):
    def __init__(self, parent_ekf):
        super().__init__(parent_ekf)
        rospy.Subscriber("aquadrone/out/imu", Imu, self.imu_cb)

        self.accel = np.array([0, 0, 0])
        self.accel_var = np.array([1, 1, 1])

        self.orientation = np.array([1, 0, 0, 0])
        self.orientation_var = np.array([1, 1, 1, 1])

        self.ang_vel = np.array([0, 0, 0])
        self.ang_vel_variance = np.array([1, 1, 1])

    def get_timeout_sec(self):
        return 0.1

    def get_p(self):
        # 3 linear accelerations
        # 4 orientation elements in quaternion form
        # 3 angular velocities
        return 10

    def get_measurement_z(self):
        return np.concatenate((self.accel, self.orientation, self.ang_vel))

    def get_R(self):
        # Variance of measurements
        return np.diag(np.concatenate((self.accel_var, self.orientation_var, self.ang_vel_variance)))

    def state_to_measurement_h(self, x, u):
        net_wrench = self.parent_ekf.get_net_wrench_func(x, u)
        acceleration = net_wrench[:3] / self.parent_ekf.mass
        return np.concatenate((acceleration, x[Idx.Ow:Idx.Oz + 1], x[Idx.Wx:Idx.Wz + 1]))

    def imu_cb(self, msg):
        self.accel = np.array([msg.linear_acceleration.x,
                               msg.linear_acceleration.y,
                               msg.linear_acceleration.z])

        self.accel_var = np.array([msg.linear_acceleration_covariance[0],
                                   msg.linear_acceleration_covariance[0],
                                   msg.linear_acceleration_covariance[0]])

        self.orientation = np.array([msg.orientation.w,
                                     msg.orientation.x,
                                     msg.orientation.y,
                                     msg.orientation.z])[np.newaxis]

        # Need to verify what our onboard sensor reports
        # Keep as this as an initial estimate
        self.orientation_var = np.array([msg.orientation_covariance[0],
                                         msg.orientation_covariance[0],
                                         msg.orientation_covariance[0],
                                         msg.orientation_covariance[0]])

        self.ang_vel = np.array([msg.angular_velocity.x,
                                 msg.angular_velocity.y,
                                 msg.angular_velocity.z])

        self.ang_vel_variance = np.array([msg.angular_velocity_covariance[0],
                                          msg.angular_velocity_covariance[4],
                                          msg.angular_velocity_covariance[8]])

        self.last_time = self.get_t()
