import rospy 
import math
import numpy as np 
import scipy.linalg

import autograd.numpy as np
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from aquadrone_msgs.msg import SubState

from worldP_indicies import IDx as IDx

class BaseListener(object):
    #----------------
    #Common Functions
    #----------------
    def __init__(self):
        self.last_time = rospy.Time.now().to_sec()
        self.calc_H = jacobian(self.state_to_measurement_h)

    def is_valid(self):
        return rospy.Time.now().to_sec - self.last_time < self.get_timeout_sec()

    def get_H(self, x, u):
        H = self.calc_H(x,u)
        H = np.reshape(H, (self.get_p(), x.shape[0]))
        return H

    #---------------------------
    #Listener Specific Functions
    #---------------------------

    def get_timeout_sec(self):
        #amount of time before old readings are outdated
        raise NotImplementedError

    def get_p(self):
        #number of elements in input
        raise NotImplementedError

    def get_measurement_z(self):
        #input
        raise NotImplementedError

    def get_R(self):
        #uncertainty of measurements
        raise NotImplementedError

    def state_to_measurement_h(self):
        #calculate predicted sensor readings
        #use autograd
        raise NotImplementedError


class SubStateListener(BaseListener):
    def __init__(self):
        super(SubStateListener, self).__init__()
        self.state_sub = rospy.Subscriber("state_estimation",SubState, self.substate_translate)

        self.position = np.array([0,0,0])
        self.pos_var = np.array([0,0,0])

        self.velocity = np.array([0,0,0])
        self.vel_var = np.array([0,0,0])

        self.orientation = np.array([1,0,0,0])
        self.orient_var = np.array([1,1,1,1])

        self.angular_vel = np.array([0,0,0])
        self.angular_vel_var = np.array([0,0,0])

    def get_timeout_sec(self):
        return 1 #subject to change

    def get_p(self):
        return 13

    def get_measurement_z(self):
        vec = np.zeros((self.get_p,1))
        for i in range(0,3):
            vec[i] = self.position
        for i in range(0,3):
            vec[i+3] = self.velocity
        for i in range(0,4):
            vec[i+6] = self.orientation
        for i in range(0,3):
            vec[i+10] = self.angular_vel
        return vec

    def get_R(self):
        vec = np.zeros((self.get_p,1))
        for i in range(0,3):
            vec[i] = self.pos_var
        for i in range(0,3):
            vec[i+3] = self.vel_var
        for i in range(0,4):
            vec[i+6] = self.orient_var
        for i in range(0,3):
            vec[i+10] = self.angular_vel_var
        return vec

    