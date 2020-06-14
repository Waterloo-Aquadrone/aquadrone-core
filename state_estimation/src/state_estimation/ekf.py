#!/usr/bin/env python

import rospy
# import numpy as np
import scipy.linalg
from scipy.spatial.transform import Rotation

import autograd.numpy as np  # Thinly-wrapped numpy
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion

from std_srvs.srv import Trigger, TriggerResponse

from aquadrone_msgs.msg import SubState, MotorControls

from ekf_indices import IDx as IDx
from ekf_sensors import IMUSensorListener, PressureSensorListener
import aquadrone_math_utils.orientation_math as OMath

quat_to_euler_jacobian = jacobian(OMath.quaternion_to_euler)


class EKF:
    def __init__(self, config):
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
            z[k] = C*x[k] + B*u[k]
        """

        ''' Process Description

        Each loop will do a prediction step based on the motor thrusts,
        gravity, buoyancy, drag, and other forces to update the expected
        state and its variance.
        
        Then a measurement step, where it uses input from the pressure sensor
        and gyro (and sensors added in hte future) to refine its expected state
        and variance.
        
        Then there is a step where the new expected state/variance is converted
        to a SubState message which is then published.

        '''
        # EKF coordinate system is at the center of mass, with axes oriented as normal for submarines
        # x is forwards, y is left, z is upwards (towards surface)
        self.mass = 10
        self.inertia_inv = np.zeros((3, 3))  # Inverse of moment of inertia matrix
        self.volume = 0  # for buoyancy
        self.cob_offset = np.array([0, 0, 0.1])  # center of buoyancy offset relative to center of mass

        self.n = IDx.NUM  # Number of state elements
        self.m = config.get_num_thrusters()  # Number of inputs

        self.x = None
        self.P = None
        self.initialize_state()

        self.u = np.zeros((self.m, 1))

        self.B = np.array(config.get_thrusts_to_wrench_matrix())

        self.Q = np.eye(self.n) * 0.01  # Uncertanty in dynamics model

        self.depth_sub = PressureSensorListener()
        self.imu_sub = IMUSensorListener()
        # Potential Future Sensors:
        # - ZED mini localization
        # - ZED mini IMU
        # - Position info from detecting objects

        self.rate = 20
        self.rate_ctrl = rospy.Rate(self.rate)

        self.calc_F = jacobian(self.f)

        self.sub_state_msg = SubState()
        self.state_pub = rospy.Publisher("state_estimation", SubState, queue_size=1)

        self.motor_sub = rospy.Subscriber("motor_command", MotorControls, self.motor_cb)

        self.reset_service = rospy.Service('reset_sub_state_estimation', Trigger, self.initialize_state)

        self.last_prediction_t = self.get_t()

    def initialize_state(self, msg=None):
        self.x = np.zeros((self.n, 1))
        self.x[IDx.Ow] = 1
        # Shouldn't x and y variance be initialized to 0, since the sub is by definition starting at (0, 0)
        self.P = np.eye(self.n)
        return TriggerResponse(success=True, message="reset")

    def motor_cb(self, msg):
        self.u = np.array(msg.motorThrusts)

    @staticmethod
    def get_t():
        return rospy.Time.now().to_sec()

    def prediction(self):
        # Get jacobian of function f
        F = self.calc_F(self.x, self.u)
        F = np.reshape(F, (self.n, self.n))
        Fx = F[0:self.n, 0:self.n]
        # print("Fx")
        # print(Fx)

        # Update x and uncertainty P
        self.x = self.f(self.x, self.u)
        inter = np.dot(Fx, self.P)
        self.P = np.dot(inter, np.transpose(Fx)) + self.Q

    def update(self):
        # Update state based on sensor measurements
        z = np.zeros((0, 0))  # measurements
        h = np.zeros((0, 0))  # predicted measurements

        H = np.zeros((0, 0))  # Jacobian of function h
        R = np.zeros((0, 0))  # Uncertainty matrix of measurement

        def add_block_diag(H, newH):
            if H.shape[0] == 0:
                ret = np.diag(newH)
                ret.shape = (newH.shape[0], newH.shape[0])
                return ret
            return scipy.linalg.block_diag(H, newH)

        def add_block_vertical(H, newH):
            if H.shape[0] == 0:
                return newH
            return np.vstack([H, newH])

        def read_listener(listener, z, h, H, R):
            if listener.is_valid():
                meas = listener.get_measurement_z()
                z = np.append(z, np.array([meas]))
                pred = listener.state_to_measurement_h(self.x, self.u)
                h = np.append(h, np.array([pred]))

                H = add_block_vertical(H, listener.get_H(self.x, self.u))
                R = add_block_diag(R, listener.get_R())

                return z, h, H, R

        for listener in [self.depth_sub,
                         self.imu_sub]:
            try:
                z, h, H, R = read_listener(listener, z, h, H, R)
            except TypeError as e:
                print(e)
                return

        if R.shape[0] == 0:
            return

        # Error in measurements vs predicted
        y = z - h
        y.shape = (y.shape[0], 1)

        # Calculate Kalman gain
        Ht = np.transpose(H)
        S = np.dot(np.dot(H, self.P), Ht) + R
        K = np.dot(np.dot(self.P, Ht), np.linalg.inv(S))

        KH = np.dot(K, H)
        I = np.eye(KH.shape[0])

        diff = np.dot(K, y)

        # Update state x and uncertainty P
        self.x = self.x + diff
        self.P = np.dot(I - KH, self.P)

    def f(self, x, u):
        # Calculate next state from current state x and inputs u
        # Must be autograd-able

        # Update time and calculate dt
        t = self.get_t()
        dt = t - self.last_prediction_t
        self.last_prediction_t = t

        accel = self.calculate_forces(x, u) / self.mass

        new_vel = np.array([x[IDx.Vx],
                            x[IDx.Vy],
                            x[IDx.Vz]]) + dt * accel

        R = Rotation.from_quat(self.x[IDx.Ow:IDx.Oz + 1]).as_matrix()

        dp_v = dt * np.dot(R, new_vel)
        dp_a = 0.5 * dt * dt * np.dot(R, accel)

        new_pos = np.array([x[IDx.Px],
                            x[IDx.Py],
                            x[IDx.Pz]]) + dp_v + dp_a

        new_or = self.update_orientation_quaternion(x, dt)

        ang_acc = np.dot(self.inertia_inv, self.calculate_torques(x, u))
        new_ang_vel = np.array([x[IDx.Ax],
                                x[IDx.Ay],
                                x[IDx.Az]])

        x_out = np.vstack([new_pos, new_vel, accel, new_or, new_ang_vel])

        return x_out

    def calculate_forces(self, x, u):
        motor_force = np.dot(self.B[:4, :], u)

        drag_forces_linear = np.array([-0.01 * x[IDx.Vx],
                                       -0.01 * x[IDx.Vy],
                                       -0.01 * x[IDx.Vz]])

        drag_forces_quad = np.array([-0.01 * x[IDx.Vx] * np.abs(x[IDx.Vx]),
                                     -0.01 * x[IDx.Vy] * np.abs(x[IDx.Vy]),
                                     -0.01 * x[IDx.Vz] * np.abs(x[IDx.Vz])])

        return motor_force + drag_forces_linear + drag_forces_quad

    def calculate_torques(self, x, u):
        motor_torque = np.dot(self.B[4:, :], u)

        # TODO: Add drag
        # TODO: add torque and forces from buoyancy

        return motor_torque

    def update_orientation_quaternion(self, x, dt):
        # https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion
        q_old = x[IDx.Ow:IDx.Oz + 1]

        w = x[IDx.Ax:IDx.Az + 1]

        new_or = q_old * (1 + 0.5 * dt * w)

        return new_or / np.linalg.norm(new_or)

    def output(self):
        var = np.array(np.diagonal(self.P))
        var.shape = (self.n, 1)

        # print("X, Var:")
        # print(np.hstack([self.x, var]))

    def fill_vector_value_variance(self, val, var, x, P, i0):
        val.x = x[i0 + 0]
        val.y = x[i0 + 1]
        val.z = x[i0 + 2]

        var.x = P[i0 + 0, i0 + 0]
        var.y = P[i0 + 1, i0 + 1]
        var.z = P[i0 + 2, i0 + 2]

    def update_state_msg(self):
        self.fill_vector_value_variance(self.sub_state_msg.position,
                                        self.sub_state_msg.pos_variance,
                                        self.x, self.P, IDx.Px)
        self.fill_vector_value_variance(self.sub_state_msg.velocity,
                                        self.sub_state_msg.vel_variance,
                                        self.x, self.P, IDx.Vx)

        quat = Quaternion()
        quat.x = self.x[IDx.Ox]
        quat.y = self.x[IDx.Oy]
        quat.z = self.x[IDx.Oz]
        quat.w = self.x[IDx.Ow]
        self.sub_state_msg.orientation_quat = quat
        self.sub_state_msg.orientation_quat_variance.w = self.P[IDx.Ow, IDx.Ow]
        self.sub_state_msg.orientation_quat_variance.x = self.P[IDx.Ox, IDx.Ox]
        self.sub_state_msg.orientation_quat_variance.y = self.P[IDx.Oy, IDx.Oy]
        self.sub_state_msg.orientation_quat_variance.z = self.P[IDx.Oz, IDx.Oz]

        self.sub_state_msg.orientation_RPY = OMath.msg_quaternion_to_euler(quat)

        # Get RPY Variance from quaternion variance
        # https://stats.stackexchange.com/questions/119780/what-does-the-covariance-of-a-quaternion-mean
        Cq = np.diag([self.sub_state_msg.orientation_quat_variance.w,
                      self.sub_state_msg.orientation_quat_variance.x,
                      self.sub_state_msg.orientation_quat_variance.y,
                      self.sub_state_msg.orientation_quat_variance.z])

        quat_vec = OMath.quat_msg_to_vec(self.sub_state_msg.orientation_quat)
        G = quat_to_euler_jacobian(quat_vec)
        G.shape = (3, 4)

        rpy_var_mat = np.dot(G, np.dot(Cq, np.transpose(G)))
        self.sub_state_msg.orientation_RPY_variance.x = rpy_var_mat[0, 0]
        self.sub_state_msg.orientation_RPY_variance.y = rpy_var_mat[1, 1]
        self.sub_state_msg.orientation_RPY_variance.z = rpy_var_mat[2, 2]

        ang_vel = Vector3()
        ang_vel.x = self.x[IDx.Ax]
        ang_vel.y = self.x[IDx.Ay]
        ang_vel.z = self.x[IDx.Az]
        self.sub_state_msg.ang_vel = ang_vel

        ang_vel_var = Vector3()
        ang_vel_var.x = self.P[IDx.Ax][IDx.Ax]
        ang_vel_var.y = self.P[IDx.Ay][IDx.Ay]
        ang_vel_var.z = self.P[IDx.Az][IDx.Az]
        self.sub_state_msg.ang_vel_variance = ang_vel_var

    def run(self):
        while not rospy.is_shutdown():
            self.rate_ctrl.sleep()

            self.output()

            self.prediction()
            self.update()

            self.update_state_msg()
            self.state_pub.publish(self.sub_state_msg)
