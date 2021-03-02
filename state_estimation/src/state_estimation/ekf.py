import rospy
import math
#import numpy as np
import scipy.linalg

import autograd.numpy as np  # Thinly-wrapped numpy
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, FluidPressure

from std_srvs.srv import Trigger, TriggerResponse

from aquadrone_msgs.msg import SubState, MotorControls

from state_estimation.ekf_indices import IDx as IDx
from state_estimation.ekf_sensors import IMUSensorListener, PressureSensorListener
import aquadrone_math_utils.orientation_math as OMath




quat_to_euler_jacobian = jacobian(OMath.quaternion_to_euler)



class EKF:
    def __init__(self, config):
        # https://en.wikipedia.org/wiki/Extended_Kalman_filter

        ''' Model Description
        x = state
        u = inputs (thruster forces, etc)
        z = ouputs (measurements)
        P = unvertainty/variance matrix of state x

        Dynamics: x[k+1] = f(x[k], u[k])
        Outputs:    z[k] = h(x[k], u[k])

        Linear Form:
          x[k+1] = A*x[k] + B*u[k]
            z[k] = C*x[k] + B*u[k]
        '''

        ''' Process Description

        Each loop will do a prediction step based on the motor thrusts,
        gravity, bouyancy, drag, and other forces to update the expected
        state and its variance.
        
        Then a measurement step, where it uses input from the pressure sensor
        and gyro (and sensors added in hte future) to refine its expected state
        and variance.
        
        Then there is a step where the new expected state/variance is converted
        to a SubState message which is then published.

        '''

        self.n = IDx.NUM # Number of state elements
        self.m = config.get_num_thrusters()  # Number of inputs

        self.x = None
        self.P = None
        self.initialize_state()

        self.u = np.zeros((self.m, 1))

        self.B = np.array(config.get_thrusts_to_wrench_matrix())

        
        self.Q = np.eye(self.n) * 0.01 # Uncertanty in dynamics model

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

        self.reset_service = rospy.Service('reset_sub_state_estimation', Trigger,self.initialize_state)

        self.last_prediction_t = self.get_t()

    def initialize_state(self, msg=None):
        self.x = np.zeros((self.n, 1))
        self.x[IDx.Ow] = 1
        # Shouldn't x and y variance be initialized to 0, since the sub is by definition starting at (0, 0)
        self.P = np.eye(self.n)
        return TriggerResponse(success=True, message="reset")

    def motor_cb(self, msg):
        self.u = np.array(msg.motorThrusts)

    def get_t(self):
        return rospy.Time.now().to_sec()

    def prediction(self):

        # Get jacobian of function f
        F = self.calc_F(self.x, self.u)
        F = np.reshape(F, (self.n,self.n))
        Fx = F[0:self.n, 0:self.n]
        #print("Fx")
        #print(Fx)

        # Update x and uncertainty P
        self.x = self.f(self.x, self.u)
        inter = np.dot(Fx, self.P)
        self.P = np.dot(  inter,  np.transpose(Fx)  ) + self.Q

        

    def update(self):
        # Update state based on sensor measurements
        z = np.zeros((0,0)) # measurements
        h = np.zeros((0,0)) # predicted measurements

        H = np.zeros((0,0)) # Jacobian of function h
        R = np.zeros((0,0)) # Uncertainty matrix of measurement

        def add_block_diag(H, newH):
            if H.shape[0] == 0:
                ret = np.diag(newH)
                ret.shape = (newH.shape[0],newH.shape[0])
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
        n = x.shape[0]

        # Update time and calculate dt
        t = self.get_t()
        dt = t - self.last_prediction_t
        self.last_prediction_t = t

        mass = 10.0
        accel = (self.calculate_linear_forces(x, u)) * 1.0 / mass

        new_vel = np.array([x[IDx.Vx],
                            x[IDx.Vy],
                            x[IDx.Vz] ]) + dt*accel


        quat = [self.x[IDx.Ow],
                self.x[IDx.Ox],
                self.x[IDx.Oy],
                self.x[IDx.Oz]]
        rpy = OMath.quaternion_to_euler(quat)

        R = OMath.RPY_Matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        R = np.asarray(R)

        dp_v = dt * np.dot(R, new_vel)
        dp_a = 0.5*dt*dt* np.dot(R, accel) 

        new_pos = np.array([x[IDx.Px],
                            x[IDx.Py],
                            x[IDx.Pz] ]) + dp_v + dp_a


        new_or = self.update_orientation_quaternion(x)

        # Update with inertia
        ang_acc = self.calculate_angular_forces(x, u) * 1.0/mass
        new_ang_vel = np.array([x[IDx.Ax],
                                x[IDx.Ay],
                                x[IDx.Az]])

        x_out = np.vstack([new_pos, new_vel, accel, new_or, new_ang_vel])


        #return np.dot(1.0*np.eye(n), x) + np.dot(2.0*np.eye(m), u)
        return x_out

    def calculate_linear_forces(self, x, u):
        mass = 10.0
        motor_wrench = np.dot(self.B, u)
        motor_forces = np.array(motor_wrench[0:3])
        motor_forces.shape = (3,1)

        drag_forces_linear = np.array([ -0.01 * x[IDx.Vx],
                                        -0.01 * x[IDx.Vy],
                                        -0.01 * x[IDx.Vz]])
        try:
            drag_forces_linear.shape = (3,1)
        except Exception as e:
            pass

        drag_forces_quad = np.array([ -0.01 * x[IDx.Vx]*np.abs(x[IDx.Vx]),
                                      -0.01 * x[IDx.Vy]*np.abs(x[IDx.Vy]),
                                      -0.01 * x[IDx.Vz]*np.abs(x[IDx.Vz])])
        try:
            drag_forces_quad.shape = (3,1)
        except Exception as e:
            pass

        return motor_forces + drag_forces_linear + drag_forces_quad

    def calculate_angular_forces(self, x, u):
        motor_wrench = np.dot(self.B, u)
        motor_forces = np.array(motor_wrench[0:3])
        motor_forces.shape = (3,1)

        # TODO: Add drag
        # TODO: add torque and forces from buoyancy

        return motor_forces
        

    def update_orientation_quaternion(self, x):
        dt = 1.0 / self.rate
        # https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion
        q_old = np.array([x[IDx.Ow],
                          x[IDx.Ox],
                          x[IDx.Oy],
                          x[IDx.Oz]])
        try:
            q_old.shape = (4,1)
        except Exception as e:
            pass
        
        w = np.array([0,
                      x[IDx.Ax],
                      x[IDx.Ay],
                      x[IDx.Az]])[np.newaxis]
        w = np.transpose(w)
        try:
            w.shape = (4,1)
        except Exception as e:
            pass

        new_or = q_old + 0.5 * dt * w
        
        try:
            new_or.shape = (4,1)
        except Exception as e:
            pass
        mag = np.linalg.norm(new_or)
        new_or = new_or / mag
        return new_or

    def output(self):
        var = np.array(np.diagonal(self.P))
        var.shape = (self.n, 1)
        
        #print("X, Var:")
        #print(np.hstack([self.x, var]))

    def fill_vector_value_variance(self, val, var, x, P, i0):
        val.x = x[i0 + 0]
        val.y = x[i0 + 1]
        val.z = x[i0 + 2]

        var.x = P[i0+0, i0+0]
        var.y = P[i0+1, i0+1]
        var.z = P[i0+2, i0+2]


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
        G.shape = (3,4)

        rpy_var_mat = np.dot(G, np.dot(Cq, np.transpose(G)))
        self.sub_state_msg.orientation_RPY_variance.x = rpy_var_mat[0,0]
        self.sub_state_msg.orientation_RPY_variance.y = rpy_var_mat[1,1]
        self.sub_state_msg.orientation_RPY_variance.z = rpy_var_mat[2,2]


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
            try:
                self.rate_ctrl.sleep()
            except ROSInterruptException:
                break

            self.output()

            self.prediction()
            self.update()

            self.update_state_msg()
            self.state_pub.publish(self.sub_state_msg)
            
