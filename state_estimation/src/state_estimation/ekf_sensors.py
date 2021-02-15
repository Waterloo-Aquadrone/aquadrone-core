from abc import ABC, abstractmethod
import rospy

import numpy as np
import sympy as sp

from sensor_msgs.msg import Imu, FluidPressure

from state_estimation.ekf_indices import Idx
from aquadrone_math_utils.ros_utils import ros_time, vector_to_np, quaternion_to_np
from scipy.linalg import block_diag


class BaseSensorListener(ABC):
    # https://en.wikipedia.org/wiki/Extended_Kalman_filter

    def __init__(self, parent_ekf):
        self.last_time = ros_time()
        self.parent_ekf = parent_ekf

        # this must be calculated after self.parent_ekf is assigned
        self.get_h_jacobian = self.get_h_jacobian_func()

    def is_valid(self):
        return ros_time() - self.last_time < self.get_timeout_sec()

    def get_h_jacobian(self, x, u):
        """
        Must return a matrix of shape (self.get_p(), n) where x is of shape (n,)

        :param x: The current state.
        :param u: The current control inputs.
        :return: Jacobian of self.state_to_measurement_h(x, u)
        """
        # Jacobian of measurement with respect to state (as calculated by get_measurement_z)
        pass

    def get_h_jacobian_func(self):
        x_vars = np.asarray(sp.symbols(f'x_:{self.parent_ekf.n}', real=True))
        u_vars = np.asarray(sp.symbols(f'u_:{self.parent_ekf.m}', real=True))

        h = self.state_to_measurement_h(x_vars, u_vars)
        jacobian_matrix = [[sp.lambdify([x_vars, u_vars], sp.diff(h_i, x_i)) for x_i in x_vars] for h_i in h]

        return lambda x, u: np.array([[func(x, u) for func in jacobian_row]
                                      for jacobian_row in jacobian_matrix])

    @abstractmethod
    def get_timeout_sec(self):
        # Amount of time to use old measurements for
        pass

    @abstractmethod
    def get_p(self):
        """
        :return: The number of elements in the measurement vector
                 (the size of the vector returned by self.get_measurement_z()).
        """
        pass

    @abstractmethod
    def get_measurement_z(self):
        """
        Must be a vector of length self.get_p()

        :return: The most recent reading of the actual measurement of the sensor.
        """
        pass

    @abstractmethod
    def get_R(self):
        """
        Must be a matrix with shape (self.get_p(), self.get_p())

        :return: The uncertainty matrix of measurements.
        """
        pass

    @abstractmethod
    def state_to_measurement_h(self, x, u):
        """
        Must return an array of length self.get_p()
        Must be sympy-able

        :param x: The current state.
        :param u: The current control inputs.
        :return: The expected sensor readings based on the given state and inputs.
        """
        pass


class PressureSensorListener(BaseSensorListener):
    def __init__(self, parent_ekf):
        super(PressureSensorListener, self).__init__(parent_ekf)
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
        return np.array([[self.variance]])

    def state_to_measurement_h(self, x, u):
        return np.array([x[Idx.z]])

    def depth_cb(self, msg):
        pressure = msg.fluid_pressure
        variance = msg.variance

        self.z = -self.pressure_to_depth(pressure)
        self.variance = variance / self.g
        self.last_time = ros_time()

    def pressure_to_depth(self, pressure):
        return (pressure - self.pressure_offset) / self.g


class IMUSensorListener(BaseSensorListener):
    def __init__(self, parent_ekf):
        super(IMUSensorListener, self).__init__(parent_ekf)
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
        return block_diag(self.accel_var, self.orientation_var, self.ang_vel_variance)

    def state_to_measurement_h(self, x, u):
        net_wrench = self.parent_ekf.get_net_wrench(x, u)
        acceleration = net_wrench[:3] / self.parent_ekf.mass
        return np.concatenate((acceleration, x[Idx.Ow:Idx.Oz + 1], x[Idx.Wx:Idx.Wz + 1]))

    def imu_cb(self, msg):
        self.accel = vector_to_np(msg.linear_acceleration)
        self.accel_var = np.array(msg.linear_acceleration_covariance).reshape((3, 3))

        self.orientation = quaternion_to_np(msg.orientation)

        # Need to verify what our onboard sensor reports
        # Keep as this as an initial estimate
        # TODO: convert RPY covariances (3x3) to quaternion covariances (4x4)
        #  page 69 of this paper: https://drive.google.com/file/d/1p1gx4UxiwRlnUXe5bEa_7E2FWrMfDk3y/view?usp=sharing
        self.orientation_var = np.diag([msg.orientation_covariance[0],
                                        msg.orientation_covariance[0],
                                        msg.orientation_covariance[0],
                                        msg.orientation_covariance[0]])

        self.ang_vel = vector_to_np(msg.angular_velocity)
        self.ang_vel_variance = np.array(msg.angular_velocity_covariance).reshape((3, 3))

        self.last_time = ros_time()
