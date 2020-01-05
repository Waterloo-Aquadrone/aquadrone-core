#!/usr/bin/env python

import rospy
import math
import numpy as np
import scipy.linalg

import autograd.numpy as np  # Thinly-wrapped numpy
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, FluidPressure

from thruster_control.configurations.v28_configuration import V28Configuration
from aquadrone_msgs.msg import SubState, MotorControls

def quaternion_to_euler(orientation):
        # From wikipedia (https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

        angles = Vector3()

        # roll (x-axis rotation)
        sinr_cosp = +2.0 * (orientation.w * orientation.x + orientation.y * orientation.z)
        cosr_cosp = +1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y)
        angles.x = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = +2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
        if np.abs(sinp) >= 1:
            angles.y = np.sign(sinp) * math.pi # use 90 degrees if out of range
        else:
            angles.y = math.asin(sinp);

        # yaw (z-axis rotation)
        siny_cosp = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
        cosy_cosp = +1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);  
        angles.z = math.atan2(siny_cosp, cosy_cosp);

        return angles;

class IDx:
    # Position
    Px = 0
    Py = 1
    Pz = 2
    
    # Velocity
    Vx = 3
    Vy = 4
    Vz = 5

    # Orientation - Quaternion
    Ow = 6
    Ox = 7
    Oy = 8
    Oz = 9

    # Angular Velocity
    Ax = 10
    Ay = 11
    Az = 12

class PressureSensorListener:
    def __init__(self):
        self.last_time = rospy.Time.now().to_sec()
        self.depth_sub = rospy.Subscriber("aquadrone/out/pressure", FluidPressure, self.depth_cb)

        self.z = 0
        self.var = 1

        self.pressure_offset = 100.0
        self.g = 9.8

        self.calc_H = jacobian(self.state_to_measurement)


    def get_timeout_sec(self):
        return 0.1

    def get_p(self):
        return 1

    def is_valid(self):
        return rospy.Time.now().to_sec() - self.last_time < self.get_timeout_sec()

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
        H = np.reshape(H, (self.get_p(), x.shape[0]))
        return H

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


class IMUSensorListener:
    def __init__(self):
        self.last_time = rospy.Time.now().to_sec()
        self.imu_sub = rospy.Subscriber("aquadrone/out/imu", Imu, self.imu_cb)

        self.calc_H = jacobian(self.state_to_measurement)

        self.orientation = np.array([1, 0, 0, 0])
        self.orientation_var = np.array([1, 1, 1, 1])

        self.pressure_offset = 100.0
        self.g = 9.8

    def get_timeout_sec(self):
        return 0.1

    def get_p(self):
        return 4

    def is_valid(self):
        return rospy.Time.now().to_sec() - self.last_time < self.get_timeout_sec()

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

    def get_H(self, x, u):
        # Jacobian of measurement wrt state (as calculated by get_measurement)
        H = self.calc_H(x, u)
        H = np.reshape(H, (self.get_p(), x.shape[0]))
        return H

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





class EKF:
    def __init__(self, config):
        # https://en.wikipedia.org/wiki/Extended_Kalman_filter

        self.n = 13
        self.m = 8

        self.x = np.zeros((self.n, 1))
        self.x[IDx.Ow] = 1

        self.u = np.zeros((self.m,1))

        self.B = np.array(config.get_thrusts_to_wrench_matrix())

        self.P = np.eye(self.n)
        self.Q = np.eye(self.n) * 0.01

        self.depth_sub = PressureSensorListener()
        self.imu_sub = IMUSensorListener()

        self.rate = 20
        self.rate_ctrl = rospy.Rate(self.rate)

        self.calc_F = jacobian(self.f)

        self.sub_state_msg = SubState()
        self.state_pub = rospy.Publisher("state_estimation", SubState, queue_size=1)

        self.motor_sub = rospy.Subscriber("motor_command", MotorControls, self.motor_cb)

    def motor_cb(self, msg):
        self.u = np.array(msg.motorThrusts)

    def prediction(self):
        F = self.calc_F(self.x, self.u)
        F = np.reshape(F, (self.n,self.n))
        Fx = F[0:self.n, 0:self.n]
        #print("Fx")
        #print(Fx)

        self.x = self.f(self.x, self.u)
        inter = np.dot(Fx, self.P)
        self.P = np.dot(  inter,  np.transpose(Fx)  ) + self.Q

        

    def update(self):
        z = np.zeros((0,0))
        h = np.zeros((0,0))

        H = np.zeros((0,0))
        R = np.zeros((0,0))

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
                meas = listener.get_measurement()
                z = np.append(z, np.array([meas]))
                pred = listener.state_to_measurement(self.x, self.u)
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


    def f(self, x, u):
        n = x.shape[0]
        dt = 1.0 / self.rate

        motor_wrench = np.dot(self.B, self.u)

        new_pos = np.array([x[IDx.Px] + dt*x[IDx.Vx],
                            x[IDx.Py] + dt*x[IDx.Vy],
                            x[IDx.Pz] + dt*x[IDx.Vz]])

        mass = 10.0
        accel = (self.calculate_linear_forces(x, motor_wrench)) * 1.0 / mass

        new_vel = np.array([x[IDx.Vx],
                            x[IDx.Vy],
                            x[IDx.Vz] ]) + dt*accel


        new_or = self.update_orientation_quaternion(x)

        # Update with inertia
        ang_acc = self.calculate_angular_forces(x, motor_wrench) * 1.0/mass
        new_ang_vel = np.array([x[IDx.Ax],
                                x[IDx.Ay],
                                x[IDx.Az]])

        x_out = np.vstack([new_pos, new_vel, new_or, new_ang_vel])


        #return np.dot(1.0*np.eye(n), x) + np.dot(2.0*np.eye(m), u)
        return x_out

    def calculate_linear_forces(self, x, wrench):
        mass = 10.0
        motor_forces = np.array(wrench[0:3])
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

    def calculate_angular_forces(self, x, wrench):
        motor_forces = np.array(wrench[0:3])
        motor_forces.shape = (3,1)

        # TODO: Add drag

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
        
        print("X, Var:")
        print(np.hstack([self.x, var]))

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
        self.sub_state_msg.orientation_RPY = quaternion_to_euler(quat)


    def run(self):
        while not rospy.is_shutdown():
            self.rate_ctrl.sleep()

            self.output()
            self.update_state_msg()
            self.state_pub.publish(self.sub_state_msg)

            self.prediction()
            self.update()
            
            

if __name__ == "__main__":
    rospy.init_node("simple_state_estimation")

    config = V28Configuration()
    config.initialize()

    ekf = EKF(config)
    ekf.run()