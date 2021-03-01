import rospy
import scipy.linalg

import numpy as np
import sympy as sp
from autograd import jacobian

from geometry_msgs.msg import Vector3, Quaternion, PoseWithCovariance

from std_srvs.srv import Trigger, TriggerResponse

from aquadrone_msgs.msg import SubState, WorldState, MotorControls

from state_estimation.ekf_indices import Idx
from state_estimation.ekf_sensors import IMUSensorListener, PressureSensorListener, VisionSensorManager
import aquadrone_math_utils.orientation_math as OMath
from aquadrone_math_utils.ros_utils import ros_time, make_vector, make_quaternion
from aquadrone_math_utils.quaternion import Quaternion

# TODO implement using Quaternion.py to remove dependency on autograd and to enable using scipy Rotations
quat_to_euler_jacobian = jacobian(OMath.quaternion_to_euler)


class EKF:
    MAX_VARIANCE = 10_000  # m^2

    def __init__(self, config, rate=None):
        # https://en.wikipedia.org/wiki/Kalman_filter
        # https://en.wikipedia.org/wiki/Extended_Kalman_filter

        """ Model Description
        x = state
        u = inputs (thruster forces, etc)
        z = outputs (measurements)
        P = uncertainty/variance matrix of state x

        Dynamics: x[k+1] = f(x[k], u[k])
        Outputs:    z[k] = h(x[k], u[k])

        Linear Form:
        x[k+1] = A*x[k] + B*u[k]
        z[k] = C*x[k] + D*u[k]
        """

        """ Process Description

        Each loop will do a prediction step based on the motor thrusts,
        gravity, buoyancy, drag, and other forces to update the expected
        state and its variance.

        Then a measurement step, where it uses input from the pressure sensor
        and gyro (and sensors added in the future) to refine its expected state
        and variance.

        Then there is a step where the new expected state/variance is converted
        to a SubState message which is then published.

        """
        self.rate = rate if rate is not None else rospy.Rate(20)  # Hz

        rospy.Subscriber("motor_command", MotorControls, self.motor_callback)
        self.sub_state_publisher = rospy.Publisher("/state_estimation", SubState, queue_size=1)
        self.world_state_publisher = rospy.Publisher("/world_state_estimation", WorldState, queue_size=1)
        rospy.Service('reset_sub_state_estimation', Trigger, self.initialize_state)
        self.last_t = ros_time()

        # constants related to ambient conditions and submarine properties
        self.g = 9.81  # m/s^2
        self.rho_water = 997  # kg/m^3
        self.mass = 10  # kg, TODO: add this to config
        self.volume = 10  # m^3, used for buoyancy calculations, TODO: add this to config
        self.buoyancy_offset = np.array(
            [0, 0, 0.5])  # location where buoyancy force is applied (center of buoyancy) TODO: add this to config
        inertia = np.array([[2850, 1, -25],
                            [1, 3800, 200],
                            [-25, 200, 2700]])  # kg * m^2
        self.inertia_inv = np.linalg.inv(inertia)  # inverse of moment of inertia matrix, TODO: add this to config

        self.n = Idx.NUM  # Number of state elements for the submarine
        self.x = None  # state (initialized at the end of this function)
        self.P = None  # uncertainty in state (initialized at the end of this function)

        self.m = config.get_num_thrusters()  # number of control inputs
        self.u = np.zeros(self.m)  # most recent motor thrust received
        self.B = config.get_thrusts_to_wrench_matrix()

        self.depth_sub = PressureSensorListener(self)
        self.imu_sub = IMUSensorListener(self)
        self.vision_sensor_manager = VisionSensorManager(self)
        self.p = self.vision_sensor_manager.get_total_state_variables()  # number of state elements for tracked objects
        # Potential Future Sensors:
        # - ZED mini localization
        # - ZED mini IMU

        self.f_jacobian = self.get_f_jacobian_func()
        self.Q = scipy.linalg.block_diag(np.eye(self.n), 0.01 * np.eye(self.p))  # uncertainty in dynamics model
        self.initialize_state()  # must initialize state after defining n and p

    def initialize_state(self, msg=None):
        self.x = np.zeros(self.n + self.p)
        # set quaternion such that sub is upright
        self.x[Idx.Ow:Idx.Oz + 1] = Quaternion.identity().as_array()

        # x and y variance are initialized to 0 since the sub is by definition starting at (0, 0)
        # all other submarine variances are initialized to 1
        # all tracked object variances are initialized to the MAX_VARIANCE
        self.P = np.diag([0, 0] + [1] * (self.n - 2) + [self.MAX_VARIANCE] * self.p)
        return TriggerResponse(success=True, message="reset")

    def motor_callback(self, msg):
        self.u = np.asarray(msg.motorThrusts)

    def prediction(self, dt):
        f_jacobian = self.f_jacobian(self.x, self.u, dt)

        # Update x and uncertainty P
        self.x = self.f(self.x, self.u, dt)
        self.P = np.linalg.multi_dot([f_jacobian, self.P, f_jacobian.T]) + self.Q

    def update(self):
        """
        Updates the state and uncertainty based on sensor measurements.
        """
        # extract data from sensors into lists
        listeners = [self.depth_sub, self.imu_sub] + self.vision_sensor_manager.get_listeners()
        z, h, h_jacobian, R = zip(*[(listener.get_measurement_z(),
                                     listener.state_to_measurement_h(self.x, self.u),
                                     listener.get_h_jacobian(self.x, self.u),
                                     listener.get_R())
                                    for listener in listeners if listener.is_valid()])
        # package lists into arrays/matrices as appropriate
        z = np.concatenate(z)  # measurements
        h = np.concatenate(h)  # predicted measurements
        h_jacobian = np.vstack(h_jacobian)  # Jacobian of function h
        R = scipy.linalg.block_diag(*R)  # Uncertainty matrix of measurement, note: this doesn't need to be sympy-able

        if len(z) == 0:
            # no valid measurements
            return

        # Error in measurements vs predicted
        y = z - h

        # Calculate Kalman gain
        # This section of the code is incredibly elegant and basically black magic. Given just the actual and expected
        # measurements, we can calculate the Kalman gain and make the appropriate updates. In essence, we provide the
        # forward model: given the state what are the measurements; and this is enough to determine the reverse
        # question: given the measurements what changes should be made to the state.
        # Conceptually, components of the state that have no measurement will have a Kalman gain of 0 (and thus variance
        # will increase by a fixed amount every time through the loop), and components of the state that have multiple
        # measurements will have everything taken into account based on their relative variances.
        S = np.linalg.multi_dot([h_jacobian, self.P, h_jacobian.T]) + R
        K = np.linalg.multi_dot([self.P, h_jacobian.T, np.linalg.inv(S)])

        # Update state x and uncertainty P
        self.x += np.dot(K, y)  # increase by error in measurement vs predicted, scaled by Kalman gain
        self.P -= np.linalg.multi_dot([K, h_jacobian, self.P])  # decrease, scaled by Kalman gain

    def f(self, x, u, dt):
        """
        Calculate next state from current state x and inputs u.
        Must be sympy-able.

        :param x: The current state.
        :param u: The currently applied motor thrusts.
        :param dt: The amount of elapsed time.
        :return: The resulting state.
        """
        new_x = np.copy(x)
        total_wrench = self.get_net_wrench(x, u)

        # update position
        new_x[Idx.x:Idx.z + 1] += x[Idx.Vx:Idx.Vz + 1] * dt

        # update orientation
        new_x[Idx.Ow:Idx.Oz + 1] += (dt / 2) * np.dot(Quaternion.from_array(x[Idx.Ow:Idx.Oz + 1]).E().transpose(),
                                                  x[Idx.Wx:Idx.Wz + 1])
        # rescale to unit quaternion
        new_x[Idx.Ow:Idx.Oz + 1] /= np.sum(x[Idx.Ow:Idx.Oz + 1] ** 2) ** 0.5  # can't use np.linalg.norm with sympy

        # update linear velocity
        new_x[Idx.Vx:Idx.Vz + 1] += total_wrench[:3] / self.mass * dt

        # update angular velocity
        new_x[Idx.Wx:Idx.Wz + 1] += np.dot(self.inertia_inv, total_wrench[3:]) * dt

        # since the world object data is in absolute coordinates, there is nothing to update for them
        return new_x

    def get_f_jacobian_func(self):
        x_vars = np.asarray(sp.symbols(f'x_:{self.n + self.p}', real=True))
        u_vars = np.asarray(sp.symbols(f'u_:{self.m}', real=True))
        dt_var = sp.symbols('dt', real=True)

        new_x = self.f(x_vars, u_vars, dt_var)
        jacobian_matrix = [[sp.lambdify([x_vars, u_vars, dt_var], sp.diff(new_x_i, x_i)) for x_i in x_vars]
                           for new_x_i in new_x]

        return lambda x, u, dt: np.array([[func(x, u, dt) for func in jacobian_row]
                                          for jacobian_row in jacobian_matrix])

    def get_net_wrench(self, x, u):
        """
        Calculates the net wrench (forces and torques) on the submarine.
        Must be sympy-able.

        :param x: The current state of the submarine.
        :param u: The motor thrusts.
        :return: The total wrench being applied to the submarine.
        """
        net_wrench = np.dot(self.B, u)

        # linear drag forces
        net_wrench[:3] += -0.01 * x[Idx.Vx:Idx.Vz + 1]

        # quadratic drag forces
        net_wrench[:3] += -0.01 * x[Idx.Vx:Idx.Vz + 1] * np.abs(x[Idx.Vx:Idx.Vz + 1])

        # TODO: add buoyancy force and torque

        return net_wrench

    def get_state_msg(self):
        sub_state_msg = SubState()

        # copy position
        sub_state_msg.position = make_vector(self.x[Idx.x:Idx.z + 1])
        sub_state_msg.pos_variance = make_vector(np.diag(self.P[Idx.x:Idx.z + 1, Idx.x:Idx.z + 1]))

        # copy velocity
        sub_state_msg.velocity = make_vector(self.x[Idx.Vx:Idx.Vz + 1])
        sub_state_msg.vel_variance = make_vector(np.diag(self.P[Idx.Vx:Idx.Vz + 1, Idx.Vx:Idx.Vz + 1]))

        # copy orientation
        quaternion = make_quaternion(self.x[Idx.Ow:Idx.Oz + 1])
        sub_state_msg.orientation_quat = quaternion
        quat_variance = make_quaternion(np.diag(self.P[Idx.Ow:Idx.Oz + 1, Idx.Ow:Idx.Oz + 1]))
        sub_state_msg.orientation_quat_variance = quat_variance

        # create roll, pitch, yaw copy of orientation
        sub_state_msg.orientation_RPY = OMath.msg_quaternion_to_euler(quaternion)

        # Get RPY Variance from quaternion variance
        # https://stats.stackexchange.com/questions/119780/what-does-the-covariance-of-a-quaternion-mean
        Cq = self.P[Idx.Ow:Idx.Oz + 1, Idx.Ow:Idx.Oz + 1]
        G = quat_to_euler_jacobian(self.x[Idx.Ow:Idx.Oz + 1]).reshape((3, 4))

        rpy_var_mat = np.linalg.multi_dot([G, Cq, G.T])
        sub_state_msg.orientation_RPY_variance = make_vector(np.diag(rpy_var_mat))

        # copy angular velocity
        sub_state_msg.ang_vel = make_vector(self.x[Idx.Wx:Idx.Wz + 1])
        sub_state_msg.ang_vel_variance = make_vector(np.diag(self.P[Idx.Wx:Idx.Wz + 1, Idx.Wx:Idx.Wz + 1]))

        return sub_state_msg

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

            current_t = ros_time()
            dt = current_t - self.last_t
            self.last_t = current_t

            self.prediction(dt)
            self.update()

            self.sub_state_publisher.publish(self.get_state_msg())
            self.world_state_publisher.publish(self.vision_sensor_manager.create_msg(self.x, self.P))
