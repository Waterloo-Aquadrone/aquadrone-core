import rospy
import math
import scipy.linalg

import autograd.numpy as np
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from aquadrone_msgs.msg import SubState

from worldP_indicies import IDx as IDx

def quat_msg_to_array(q):
    return np.array([q.w, q.x, q.y, q.z])

def vec_to_msg(vec):
    message = Vector3()
    message.x = vec[0]
    message.y = vec[1]
    message.z = vec[2]
    return message

def quat_to_euler(q):
    # From wikipedia (https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
    # Copied from state estimation
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]

    euler = np.array([0,0,0])

    # roll (x-axis rotation)
    sinr_cosp = +2.0 * (w * x + y * z)
    cosr_cosp = +1.0 - 2.0 * (x * x + y * y)
    #angles[0] = np.arctan2(sinr_cosp, cosr_cosp)
    

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (w * z + x * y)
    cosy_cosp = +1.0 - 2.0 * (y * y + z * z)
    #angles[2] = math.atan2(siny_cosp, cosy_cosp)

    # pitch (y-axis rotation)
    sinp = +2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        #angles[1] = np.sign(sinp) * math.pi # use 90 degrees if out of range
        return np.array([ np.arctan2(sinr_cosp, cosr_cosp),
                            np.sign(sinp) * math.pi,
                            np.arctan2(siny_cosp, cosy_cosp)
                            ])
    else:
        #angles[1] = math.asin(sinp)
        return np.array([ np.arctan2(sinr_cosp, cosr_cosp),
                            np.arcsin(sinp),
                            np.arctan2(siny_cosp, cosy_cosp)
                            ])

    return euler

def quat_msg_to_euler(q):
    q_array = quat_msg_to_array(q)
    return vec_to_msg(quat_to_euler(q_array))

def quat_to_euler_jacobian(q):
    return jacobian(quat_to_euler(q))

class EKF:
    def __init__(self):
        # https://en.wikipedia.org/wiki/Extended_Kalman_filter
        
        ''' Model Description
        x = state
        u = inputs (sub position and orientation, object position in camera)
        z = outputs (sub and object position)
        P = unvertainty/variance matrix of state x

        Dynamics: x[k+1] = f(x[k], u[k])
        Outputs:    z[k] = h(x[k], u[k])

        Linear Form:
          x[k+1] = A*x[k] + B*u[k]
            z[k] = C*x[k] + B*u[k]
        '''

        ''' Process Description

        Each loop will do a prediction step based on the sub state,
        to update the expected sub and world object positions and their variance.
        
        Then a measurement step, where it uses input from the zed camera's
        detected objects to refine the prediction and varience
        
        Then there is a step where the new expected sub state and world state is converted
        to a WorldState message which is then published.

        '''

        #first version: only 1 object in state

        self.n = 13+3 #substate + world x,y,z of object
        self.m = 13+3 #sub state + x,y,z of object in camera

        self.x = np.zeros((self.n,1))
        self.x[IDx.Ow] = 1

        self.u = np.zeros((self.m, 1))

        self.B = np.array() #input modification matrix

        self.P = np.eye(self.n)
        self.Q = np.eye(self.n) * 0.01 #uncert in dynamics model

        self.camera_info = NotImplemented #implement camera listening thing

        self.rate = 20
        self.rate_ctrl = rospy.Rate(self.rate)

        self.calc_F = jacobian(self.f)

        #self.world_pos_msg = NotImplemented #implement worldpositioning message
        self.world_pub = rospy.Publisher("world state", NotImplemented, queue_size = 1)
        self.sub_state_sub = rospy.Subscriber("state estimation", NotImplemented, NotImplemented)

        self.last_prediction_t = self.get_t()

    def get_t(self):
        return rospy.Time.now().to_sec()

    def prediction(self):
        #jacobian of F
        F = self.calc_F(self.x, self.u)
        F = np.reshape(F,(self.n ,self.n))
        Fx = F[0:self.n, 0:self.n]
        print("Fx", Fx)

        #update x and P
        self.x = self.f(self.x, self.u)  #where is this f coming from
        inter = np.dot(Fx, self.P)
        self.P = np.dot(inter, np.transpose(Fx)) + self.Q

    def update(self):
        #update world based on sub state
        z = np.zeros((0,0)) #measurements
        h = np.zeros((0,0)) #measurement prediction

        H = np.zeros((0,0)) #jacobian of h
        R = np.zeros((0,0)) #uncert of measurement

        def add_block_diag(H,newH):
            if H.shape[0] == 0:
                ans = np.diag(newH)
                ans.shape = (newH.shape[0],newH.shape[0])
                return ans
            return scipy.linalg.block_diag(H,newH)

        def add_block_vert(H,newH):
            if H.shape[0] == 0:
                return newH
            return np.vstack([H,newH])

        def read_listener(listener, z, h, H, R):
            NotImplemented

        

