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

    self.numObjects = 10

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
            vec[i] = self.position[i]
        for i in range(0,3):
            vec[i+3] = self.velocity[i]
        for i in range(0,4):
            vec[i+6] = self.orientation[i]
        for i in range(0,3):
            vec[i+10] = self.angular_vel[i]
        return vec

    def get_R(self):
        vec = np.zeros((self.get_p,1))
        for i in range(0,3):
            vec[i,i] = self.pos_var[i]
        for i in range(0,3):
            vec[i+3,i+3] = self.vel_var[i]
        for i in range(0,4):
            vec[i+6,i+6] = self.orient_var[i]
        for i in range(0,3):
            vec[i+10,i+10] = self.angular_vel_var[i]
        return vec

    def subState_to_measurement_h(self,x,u): #do we need u?
        z = np.array(  [x[IDx.SPx],
                        x[IDx.SPy],
                        x[IDx.SPz],
                        x[IDx.SVx],
                        x[IDx.SVy],
                        x[IDx.SVz],
                        x[IDx.Ow],
                        x[IDx.Ox],
                        x[IDx.Oy],
                        x[IDx.Oz],
                        x[IDx.Ax],
                        x[IDx.Ay],
                        x[IDx.Az]])
        return z

    def subState_cb(self,msg):
        #input position
        self.position = np.array(  [msg.position.x,
                                    msg.position.y,
                                    msg.position.z])[np.newaxis]
        self.position.shape = (3,1)
        #input position variance
        self.pos_var = np.array(   [msg.pos_variance.x,
                                    msg.pos_variance.y,
                                    msg.pos_variance.z])
        self.pos_var.shape = (3,1)
        #input velocity
        self.velocity = np.array(  [msg.velocity.x,
                                    msg.velocity.y,
                                    msg.velocity.z])[np.newaxis]
        self.velocity.shape = (3,1)
        #input velocity varience
        self.vel_var = np.array(   [msg.vel_variance.x,
                                    msg.vel_variance.y,
                                    msg.vel_variance.z])
        self.vel_var.shape = (3,1)
        #input orientation
        self.orientation = np.array(   [msg.orientation_RPY.w, ##not sure whether rpy is the right one
                                        msg.orientation_RPY.x,
                                        msg.orientation_RPY.y,
                                        msg.orientation_RPY.z])[np.newaxis]
        self.orientation.shape = (4,1)
        #input orientation varience
        self.orient_var = np.array(    [msg.orientation_RPY_variance.w, ##not sure whether rpy is the right one
                                        msg.orientation_RPY_variance.x,
                                        msg.orientation_RPY_variance.y,
                                        msg.orientation_RPY_variance.z])
        self.orient_var.shape = (4,1)
        #input angular velocity
        self.angular_vel = np.array(   [msg.ang_vel.x,
                                        msg.ang_vel.y,
                                        msg.ang_vel.z])[np.newaxis]
        self.angular_vel.shape = (3,1)
        #input angular velocity varience
        self.angular_vel_var = np.array(   [msg.ang_vel_varience.x,
                                            msg.ang_vel_varience.y,
                                            msg.ang_vel_varience.z])
        self.angular_vel_var.shape = (3,1)

        self.last_time = rospy.Time.now().to_sec()

class ZedCamListener(BaseListener):

    def __init__(self):
        super(SubStateListener, self).__init__()
        #currently working for a single object
        self.cam_sub = rospy.Subscriber("zed_camera",Point,self.cam_cb)
        
        self.obj_pos = np.array.zeros(len = 3*self.numObjects)
        self.obj_pos_var = np.array.zeros(len = 3*self.numObjects)

    def get_timeout_sec(self):
        return 0.1
    
    def get_p(self):
        #single point, 3 elements
        return 3*self.numObjects

    def get_measurement_z(self):
        vec = np.zeros((self.get_p(),1))
        for i in range(0,3*self.numObjects):
            vec[i] = self.obj.pos[i]
        return vec
    
    def get_R(self):
        var = np.zeros((self.get_p(),1))
        for i in range(0,3*self.numObjects):
            var[i,i] = self.obj_pos_var[i]
        return var

    def state_to_measurement_h(self, x, u):
        ret = np.array.zeros((3*self.numObjects,1))
        for i in range(0,3*self.numObjects):
            ret[i] = x[13+i]
        return ret

    def cam_cb(self, msg):
        for i in range(0,self.numObjects*3):
            self.obj_pos = msg[i]
        self.obj_pos.shape = (3*self.numObjects,1)
        #self.obj_pos = np.array([msg.point.x, msg.point.y, msg.point.z])[np.newaxis]
        
        for i in range(0,self.numObjects*3):
            self.obj_pos_var = msg[i+30] #not sure about formatting
        self.obj_pos_var.shape = (3*self.numObjects,1)
        """
        self.obj_pos_var = np.array([msg.point_covariance[0],
                                    msg.point_covariance[1],
                                    msg.point_covariance[2]]) 
        """
        self.last_time = rospy.Time.now().to_sec()