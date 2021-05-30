from abc import ABC, abstractmethod
import rospy

import numpy as np
import sympy as sp

from sensor_msgs.msg import Imu, FluidPressure
from aquadrone_msgs.msg import WorldObjectState, WorldState

from state_estimation.ekf_indices import Idx
from aquadrone_math_utils.quaternion import Quaternion
from aquadrone_math_utils.ros_utils import ros_time, vector_to_np, quaternion_to_np, make_vector, make_quaternion
from scipy.linalg import block_diag


class BaseSensorListener(ABC):
    # https://en.wikipedia.org/wiki/Extended_Kalman_filter

    def __init__(self, parent_ekf):
        self.last_time = ros_time()
        self.parent_ekf = parent_ekf
        self.get_h_jacobian = None

    def initialize(self):
        # this must be calculated after self.parent_ekf is assigned
        self.get_h_jacobian = self.get_h_jacobian_func()

    def is_valid(self):
        return ros_time() - self.last_time < self.get_timeout_sec()

    def get_h_jacobian(self, x, u):
        """
        Calculates the jacobian of the measurement with respect to state (as calculated by get_measurement_z)
        Must return a matrix of shape (self.get_p(), n) where x is of shape (n,)
        This function will be defined automatically using self.get_h_jacobian_func during initialization.

        :param x: The current state.
        :param u: The current control inputs.
        :return: Jacobian of self.state_to_measurement_h(x, u)
        """
        pass

    def get_h_jacobian_func(self):
        x_vars = np.asarray(sp.symbols(f'x_:{self.parent_ekf.n + self.parent_ekf.p}', real=True))
        u_vars = np.asarray(sp.symbols(f'u_:{self.parent_ekf.m}', real=True))

        h = self.state_to_measurement_h(x_vars, u_vars)
        jacobian_matrix = [[sp.lambdify([x_vars, u_vars], sp.diff(h_i, x_i)) for x_i in x_vars] for h_i in h]

        return lambda x, u: np.array([[func(x, u) for func in jacobian_row]
                                      for jacobian_row in jacobian_matrix])

    @abstractmethod
    def get_timeout_sec(self):
        """
        :return: The amount of time (in seconds) to use old measurements for.
        """
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
        Must be sympy-able.

        :param x: The current state.
        :param u: The current control inputs.
        :return: The expected sensor readings based on the given state and inputs.
        """
        pass


class PressureSensorListener(BaseSensorListener):
    WATER_SPECIFIC_GRAVITY = 1000 * 9.81  # Pa/m
    ATMOSPHERIC_PRESSURE = 101325  # Pa

    def __init__(self, parent_ekf):
        super(PressureSensorListener, self).__init__(parent_ekf)
        rospy.Subscriber("aquadrone/sensors/pressure", FluidPressure, self.depth_cb)

        self.z = 0
        self.variance = 1
        self.g = 9.8
        pressure_offset_dict = rospy.get_param("/submarine/pressure_sensor_offset")
        self.pressure_offset = [pressure_offset_dict["x"], pressure_offset_dict["y"], pressure_offset_dict["z"]]

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
        quad_orientation = Quaternion.from_array(x[Idx.Ow:Idx.Oz + 1])
        rotated_offset = quad_orientation.rotate(self.pressure_offset)
        return np.array([x[Idx.z]+rotated_offset[2]])

    def depth_cb(self, msg):
        absolute_pressure = msg.fluid_pressure
        variance = msg.variance

        gauge_pressure = absolute_pressure - self.ATMOSPHERIC_PRESSURE
        depth = gauge_pressure / self.WATER_SPECIFIC_GRAVITY

        self.z = -depth
        self.variance = variance / self.WATER_SPECIFIC_GRAVITY
        self.last_time = ros_time()


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


class VisionSensorManager:
    # TODO: expand to support non-point objects
    WORLD_OBJECTS = ['red_pole', 'green_pole', 'blue_pole', 'white_pole']

    def __init__(self, parent_ekf, world_objects=None):
        if world_objects is None:
            world_objects = self.WORLD_OBJECTS
        self.listeners = {identifier: PointObjectListener(parent_ekf, identifier,
                                                          slice(Idx.NUM + 3 * index, Idx.NUM + 3 * (index + 1)))
                          for index, identifier in enumerate(world_objects)}
        rospy.Subscriber("aquadrone/vision_data", WorldState, self.vision_cb)

    def initialize(self):
        for listener in self.listeners.values():
            listener.initialize()

    def vision_cb(self, vision_data):
        for world_object_state in vision_data.data:
            if world_object_state.identifier in self.listeners.keys():
                self.listeners[world_object_state.identifier].vision_cb(world_object_state.pose_with_covariance)

    def create_msg(self, x, P):
        msg = WorldState()
        msg.data = list(filter(lambda m: m is not None,
                               [listener.create_msg(x, P) for listener in self.get_listeners()]))
        return msg

    def get_listeners(self):
        return list(self.listeners.values())

    def get_total_state_variables(self):
        return sum([listener.get_p() for listener in self.listeners.values()])


class PointObjectListener(BaseSensorListener):
    def __init__(self, parent_ekf, identifier, state_slice):
        """
        :param parent_ekf:
        :param identifier:
        :param state_slice: Slice object representing the indices in the state vector where the point object's
                            absolute position is located.
        """
        super(PointObjectListener, self).__init__(parent_ekf)
        self.identifier = identifier
        self.state_slice = state_slice
        self.pose = np.zeros(3)
        self.covariance = 0.1 * np.identity(3)

    def vision_cb(self, pose_with_covariance):
        self.pose = vector_to_np(pose_with_covariance.pose.position)
        self.covariance = np.array(pose_with_covariance.covariance).reshape((6, 6))[:3, :3]
        self.last_time = ros_time()

    def create_msg(self, x, P):
        pos_covariance = P[self.state_slice, self.state_slice]
        if np.any(pos_covariance >= self.parent_ekf.MAX_VARIANCE):
            return None

        msg = WorldObjectState()
        msg.identifier = self.identifier
        msg.pose_with_covariance.pose.position = make_vector(x[self.state_slice])
        msg.pose_with_covariance.pose.orientation = make_quaternion(Quaternion.identity().as_array())
        covariance = np.zeros((6, 6))
        covariance[:3, :3] = pos_covariance
        msg.pose_with_covariance.covariance = covariance.flatten()
        return msg

    def get_timeout_sec(self):
        return 0.1

    def get_p(self):
        return 3

    def get_measurement_z(self):
        return self.pose

    def get_R(self):
        return self.covariance

    def state_to_measurement_h(self, x, u):
        # combine submarine's state and absolute position of the target to compute the relative position of the target
        sub_pos = x[Idx.x:Idx.z + 1]
        sub_quat = Quaternion.from_array(x[Idx.Ow:Idx.Oz + 1])
        absolute_target_pos = x[self.state_slice]

        relative_target_pos = sub_quat.unrotate(absolute_target_pos - sub_pos)
        return relative_target_pos
