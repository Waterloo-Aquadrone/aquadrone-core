import rospy
import scipy.linalg

import numpy as np
import sympy as sp
from autograd import jacobian

from geometry_msgs.msg import Vector3, Quaternion

from std_srvs.srv import Trigger, TriggerResponse

from aquadrone_msgs.msg import SubState, MotorControls

from state_estimation.ekf_indices import Idx
from state_estimation.ekf_sensors import IMUSensorListener, PressureSensorListener
import aquadrone_math_utils.orientation_math as OMath
from aquadrone_math_utils.ros_utils import ros_time
from aquadrone_math_utils.quaternion import Quaternion

quat_to_euler_jacobian = jacobian(OMath.quaternion_to_euler)


class EKF:
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
        self.state_publisher = rospy.Publisher("state_estimation", SubState, queue_size=1)
        rospy.Service('reset_sub_state_estimation', Trigger, self.initialize_state)
        self.last_t = ros_time()

        # constants related to ambient conditions and submarine properties
        self.g = 9.81  # m/s^2
        self.rho_water = 997  # kg/m^3
        self.mass = 10  # TODO: add this to config
        self.volume = 10  # used for buoyancy calculations, TODO: add this to config
        self.buoyancy_offset = np.array(
            [0, 0, 0.5])  # location where buoyancy force is applied (center of buoyancy) TODO: add this to config
        inertia = np.array([[2850, 1, -25],
                            [1, 3800, 200],
                            [-25, 200, 2700]])
        self.inertia_inv = np.linalg.inv(inertia)  # inverse of moment of inertia matrix, TODO: add this to config

        self.n = Idx.NUM  # Number of state elements
        self.x = None  # state
        self.P = None  # uncertainty in state
        self.initialize_state()

        self.m = config.get_num_thrusters()  # number of control inputs
        self.u = np.zeros(self.m)  # most recent motor thrust received
        self.B = config.get_thrusts_to_wrench_matrix()

        self.f_jacobian = self.get_f_jacobian_func()
        self.net_wrench_jacobian = self.get_net_wrench_jacobian_func()
        self.Q = np.eye(self.n)  # uncertainty in dynamics model

        self.depth_sub = PressureSensorListener(self)
        self.imu_sub = IMUSensorListener(self)
        # Potential Future Sensors:
        # - ZED mini localization
        # - ZED mini IMU
        # - Position info from detecting objects

    def initialize_state(self, msg=None):
        self.x = np.zeros(self.n)
        # set quaternion such that sub is upright
        self.x[Idx.Ow] = 1  # TODO: verify that this corresponds to being upright

        # x and y variance are initialized to 0 since the sub is by definition starting at (0, 0)
        self.P = np.diag([0, 0] + [1] * (self.n - 2))
        return TriggerResponse(success=True, message="reset")

    def motor_callback(self, msg):
        self.u = np.asarray(msg.motorThrusts)

    def prediction(self, dt):
        f_jacobian = self.f_jacobian(self.x, self.u, dt).reshape((self.n, self.n))

        # Update x and uncertainty P
        self.x = self.f(self.x, self.u, dt)
        self.P = np.linalg.multi_dot([f_jacobian, self.P, f_jacobian.T]) + self.Q

    def update(self):
        """
        Updates the state and uncertainty based on sensor measurements.
        """
        # extract data from sensors into lists
        listeners = [self.depth_sub, self.imu_sub]
        z, h, h_jacobian, R = zip(*[(listener.get_measurement_z(),
                                     listener.state_to_measurement_h(self.x, self.u),
                                     listener.get_h_jacobian(self.x, self.u),
                                     listener.get_R())
                                    for listener in listeners if listener.is_valid()])
        # package lists into arrays/matrices as appropriate
        z = np.concatenate(z)  # measurements
        h = np.concatenate(h)  # predicted measurements
        h_jacobian = np.vstack(h_jacobian)  # Jacobian of function h
        R = scipy.linalg.block_diag(
            *R)  # Uncertainty matrix of measurement, note: this doesn't need to be autograd-able

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
        total_wrench = self.get_net_wrench(x, u)

        # update position
        x[Idx.x:Idx.z + 1] += x[Idx.Vx:Idx.Vz + 1] * dt

        # update orientation
        x[Idx.Ow:Idx.Oz + 1] += (dt / 2) * np.dot(Quaternion.from_array(x[Idx.Ow:Idx.Oz + 1]).E().transpose(),
                                                  x[Idx.Wx:Idx.Wz + 1])
        # rescale to unit quaternion
        x[Idx.Ow:Idx.Oz + 1] /= np.sum(x[Idx.Ow:Idx.Oz + 1] ** 2) ** 0.5  # can't use np.linalg.norm with sympy

        # update linear velocity
        x[Idx.Vx:Idx.Vz + 1] += total_wrench[:3] / self.mass * dt

        # update angular velocity
        x[Idx.Wx:Idx.Wz + 1] += np.dot(self.inertia_inv, total_wrench[3:]) * dt

        return x

    def get_f_jacobian_func(self):
        x_vars = list(sp.symbols(', '.join([f'x_{i}' for i in range(self.n)])))
        u_vars = list(sp.symbols(', '.join([f'u_{i}' for i in range(self.m)])))
        dt_var = sp.symbols('dt')

        new_x = self.f(x_vars, u_vars, dt_var)
        jacobian_matrix = [[sp.lambdify([x_vars, u_vars, dt_var], sp.diff(new_x_i, x_i)) for x_i in x_vars]
                           for new_x_i in new_x]

        return lambda x, u, dt: np.array([[func(x, u, dt) for func in jacobian_row]
                                          for jacobian_row in jacobian_matrix])

    def get_net_wrench(self, x, u):
        """
        Calculates the net wrench (forces and torques) on the submarine.
        Must be sympy-able

        :param x: The current state of the submarine.
        :param u: The motor thrusts.
        :return: The total wrench being applied to the submarine.
        """
        net_wrench = np.dot(self.B, u)

        # linear drag forces
        net_wrench[:3] += -0.01 * x[Idx.Vx:Idx.Vz + 1]

        # quadratic drag forces
        net_wrench[:3] += -0.01 * x[Idx.Vx:Idx.Vz + 1] * np.abs(x[Idx.Vx:Idx.Vz + 1])

        return net_wrench

    def get_net_wrench_jacobian_func(self):
        x_vars = list(sp.symbols(', '.join([f'x_{i}' for i in range(self.n)])))
        u_vars = list(sp.symbols(', '.join([f'u_{i}' for i in range(self.m)])))

        net_wrench = self.get_net_wrench(x_vars, u_vars)
        jacobian_matrix = [[sp.lambdify([x_vars, u_vars], sp.diff(wrench_component, x_i)) for x_i in x_vars]
                           for wrench_component in net_wrench]

        return lambda x, u: np.array([[func(x, u) for func in jacobian_row] for jacobian_row in jacobian_matrix])

    def get_state_msg(self):
        sub_state_msg = SubState()

        # copy position
        position = Vector3()
        position.x = self.x[Idx.x]
        position.y = self.x[Idx.y]
        position.z = self.x[Idx.z]
        sub_state_msg.position = position
        pos_variance = Vector3()
        pos_variance.x = self.P[Idx.x, Idx.x]
        pos_variance.y = self.P[Idx.y, Idx.y]
        pos_variance.z = self.P[Idx.z, Idx.z]
        sub_state_msg.pos_variance = pos_variance

        # copy velocity
        velocity = Vector3()
        velocity.x = self.x[Idx.Vx]
        velocity.y = self.x[Idx.Vy]
        velocity.z = self.x[Idx.Vz]
        sub_state_msg.velocity = velocity
        vel_variance = Vector3()
        vel_variance.x = self.P[Idx.Vx, Idx.Vx]
        vel_variance.y = self.P[Idx.Vy, Idx.Vy]
        vel_variance.z = self.P[Idx.Vz, Idx.Vz]
        sub_state_msg.vel_variance = vel_variance

        # copy orientation
        quaternion = Quaternion()
        quaternion.x = self.x[Idx.Ox]
        quaternion.y = self.x[Idx.Oy]
        quaternion.z = self.x[Idx.Oz]
        quaternion.w = self.x[Idx.Ow]
        sub_state_msg.orientation_quat = quaternion
        quat_variance = Quaternion()
        quat_variance.w = self.P[Idx.Ow, Idx.Ow]
        quat_variance.x = self.P[Idx.Ox, Idx.Ox]
        quat_variance.y = self.P[Idx.Oy, Idx.Oy]
        quat_variance.z = self.P[Idx.Oz, Idx.Oz]
        sub_state_msg.orientation_quat_variance = quat_variance

        # create roll, pitch, yaw copy of orientation
        sub_state_msg.orientation_RPY = OMath.msg_quaternion_to_euler(quaternion)

        # Get RPY Variance from quaternion variance
        # https://stats.stackexchange.com/questions/119780/what-does-the-covariance-of-a-quaternion-mean
        Cq = np.diag([sub_state_msg.orientation_quat_variance.w,
                      sub_state_msg.orientation_quat_variance.x,
                      sub_state_msg.orientation_quat_variance.y,
                      sub_state_msg.orientation_quat_variance.z])

        quat_vec = OMath.quat_msg_to_vec(sub_state_msg.orientation_quat)
        G = quat_to_euler_jacobian(quat_vec)
        G.shape = (3, 4)

        rpy_var_mat = np.linalg.multi_dot([G, Cq, G.T])
        RPY_variance = Vector3()
        RPY_variance.x = rpy_var_mat[0, 0]
        RPY_variance.y = rpy_var_mat[1, 1]
        RPY_variance.z = rpy_var_mat[2, 2]
        sub_state_msg.orientation_RPY_variance = RPY_variance

        # copy angular velocity
        ang_vel = Vector3()
        ang_vel.x = self.x[Idx.Wx]
        ang_vel.y = self.x[Idx.Wy]
        ang_vel.z = self.x[Idx.Wz]
        sub_state_msg.ang_vel = ang_vel

        ang_vel_var = Vector3()
        ang_vel_var.x = self.P[Idx.Wx][Idx.Wx]
        ang_vel_var.y = self.P[Idx.Wy][Idx.Wy]
        ang_vel_var.z = self.P[Idx.Wz][Idx.Wz]
        sub_state_msg.ang_vel_variance = ang_vel_var

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

            self.state_publisher.publish(self.get_state_msg())
