#!/usr/bin/env python

import rospy
import math
import scipy.linalg

import autograd.numpy as np
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from aquadrone_msgs.msg import SubState, WorldState, Vision_Array, Vision

from worldP_indicies import IDx as IDx
from worldP_listener import SubStateListener, ZedCamListener
from aquadrone_math_utils.orientation_math import Yaw, Pitch, Roll, RPY_Matrix

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
        P = uncertainty/variance matrix of state x

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

        self.nState = 13 #number of elements in state
        self.numObjects = 10 #number objects being considered

        #n and m are only for 1 point
        self.n = self.nState+3*self.numObjects #substate + world x,y,z of object
        self.m = 7+3*self.numObjects #position and orientation + x,y,z of object in camera

        self.x = np.zeros((self.n,1))
        self.x[IDx.Ow] = 1

        self.u = np.zeros((self.m, 1))

        self.B = np.zeros((self.nState,1)) #input modification matrix

        self.P = np.eye(self.n)
        self.Q = np.eye(self.n) * 0.01 #uncert in dynamics model

        self.camera_info = ZedCamListener() 
        self.state_info = SubStateListener()

        self.state_msg = SubState()
        self.camera_msg = Vision_Array() 
        self.world_msg = WorldState()

        self.rate = 20
        self.rate_ctrl = rospy.Rate(self.rate)

        self.calc_F = jacobian(self.f)

        self.world_pub = rospy.Publisher("world_state", WorldState, queue_size = 1)

        self.last_prediction_t = self.get_t()

    def get_t(self):
        return rospy.Time.now().to_sec()

    def prediction(self):
        #jacobian of F
        F = self.calc_F(self.x, self.u)
        F = np.reshape(F,(self.n ,self.n))
        Fx = F[0:self.n, 0:self.n]
        #print(Fx)

        #update x and P
        self.x = self.f(self.x, self.u)

        self.P = np.linalg.multi_dot([Fx,self.P, np.transpose(Fx)]) + self.Q
        

    def update(self):
        #update world based on sub state
        z = np.zeros((0,0)) #measurements
        h = np.zeros((0,0)) #measurement prediction

        H = np.zeros((0,0)) #jacobian of h
        R = np.zeros((0,0)) #uncert of measurement

        def add_block_diag(H,newH):
            if H.shape[0] == 0:
                #ans = np.diag(newH)
                ans = newH
                #print(ans)
                ans.reshape((newH.shape[0],newH.shape[0]))
                return ans
            return scipy.linalg.block_diag(H,newH)

        def add_block_vert(H,newH):
            if H.shape[0] == 0:
                return newH
            return np.vstack([H,newH])

        def read_listener(listener, z, h, H, R):
            
            if listener.is_valid():
                meas = listener.get_measurement_z()
                z = np.append(z,np.array([meas]))
                pred = listener.state_to_measurement_h(self.x, self.u)
                h = np.append(h,np.array([pred]))
                
                #print(self.x)
                #print("test")
                H = add_block_vert(H, listener.get_H(self.x, self.u))
                #print(listener, listener.is_valid())
                R = add_block_diag(R, listener.get_R())
                
                return z,h,H,R
        """
        def read_listener_camera(listener, z,h,H,R):
            if listener.is_valid():
                meas = listener.get_measurement_z()

                rotation_matrix = RPY_Matrix([[0,0,0],
                                              [0,0,0],
                                              [0,0,0]])
                #returns np array
                for i in range(0,meas.shape[0],3): # reads in a point at a time
                    #get sub orientation in quat
                    sub_quat = np.array([z[6],z[7],z[8],z[9]])
                    #turn into euler
                    sub_euler = quat_to_euler(sub_quat)
                    #get object pos relative to camera
                    camP = np.array([meas[i],meas[i+1],meas[i+2]])
                    #get sub position
                    subP = np.array([z[0],z[1],z[2]])

                    rotation_matrix = RPY_Matrix(sub_euler[0], sub_euler[1], sub_euler[2])

                    final = camP*rotation_matrix+ subP


                    z = np.append(z,np.array([final]))
                pred = listener.state_to_measurement_h(self.x, self.u) #TODO might need to check x
                h=np.append(h,np.array([pred]))

                H = add_block_vert(H, listener.get_H(self.x, self.u))
                
                local_var = listener.get_R()
                
                #local_x = np.array([local_var[0],0,0]) #making local x var vector
                #local_y = np.array([0,local_var[1],0]) #making local y var vector
                #local_z = np.array([0,0,local_var[2]]) #making local z var vector

                world_var = local_var*rotation_matrix #converting local variance to world frame variance
                R = add_block_diag(R, world_var) #TODO need to implement localizing of uncert

                return z,h,H,R
        """

        for listener in (self.state_info, self.camera_info):  
            try:
                z,h,H,R = read_listener(listener,z,h,H,R)
            except TypeError as e:
                print(e)
                return

        if R.shape[0] == 0:
            return

        #error in measurments vs predicted
        y = z-h
        y.shape = (y.shape[0],1)

        #calc kalman gain
        Ht = np.transpose(H)
        S = np.dot(np.dot(H,self.P),Ht)+R
        #print(S)
        K = np.dot(np.dot(self.P,Ht),np.linalg.inv(S)) #TODO with "proper" rotation and stuff of the input data, we get a singular matrix, so its just using the raw data for now, which is technically correct for the fake omniscient node thing

        KH = np.dot(K,H)
        I = np.eye(KH.shape[0])

        diff = np.dot(K,y)

        #update state x and uncert P
        self.x = self.x+diff
        self.P = np.dot(I-KH,self.P)

    def f(self, x, u):
        #calculate next state from current state x and inputs u
        #must be autograd-able

        #update time and calc dt
        t = self.get_t()
        dt = t - self.last_prediction_t
        self.last_prediction_t = t
                
        #find new position
        new_pos = np.array([x[IDx.SPx] + dt*x[IDx.SVx],
                            x[IDx.SPy] + dt*x[IDx.SVy],
                            x[IDx.SPz] + dt*x[IDx.SVz]])

        #find new orientation in euler
        new_orient = self.update_orientation_quaternion(x)
        new_vel = np.array([x[IDx.SVx],x[IDx.SVy],x[IDx.SVz]])
        new_ang_vel = np.array([x[IDx.Ax],x[IDx.Ay],x[IDx.Az]])

        
        #find new object position (essentially the same spot since they dont move)
        new_obj_pos = np.array(x[self.nState:x.shape[0]])


        #new_obj_pos = np.zeros((x.shape[0]-self.nState,1))
        #for i in range(self.nState,x.shape[0]):
        #    new_obj_pos[i-self.nState] = x.tolist()

        return_val = np.vstack([new_pos,new_vel, new_orient, new_ang_vel, new_obj_pos])
        return return_val
        
    #copied from ekf.py
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
        var.shape = (self.n,1)

    def fill_vector_value_variance(self,val, var, x, P, i0):
        val.x = x[i0+0]
        val.y = x[i0+1]
        val.z = x[i0+2]

        var.x = P[i0+0, i0+0]
        var.y = P[i0+1, i0+1]
        var.z = P[i0+2, i0+2]

    def fill_vector_value_variance_objects(self,val, var, x, P, i0):
        
        #get sub orientation in quat
        sub_quat = np.array([x[IDx.Ow],x[IDx.Ox],x[IDx.Oy],x[IDx.Oz]])
        #turn into euler
        sub_euler = quat_to_euler(sub_quat)
        """
        #get object pos relative to camera
        camP = np.array([x[i0],x[i0+1],x[i0+2]])
        #get sub position
        subP = np.array([x[IDx.Px],x[IDx.Py],x[IDx.Pz]])
        """
        rotation_matrix = RPY_Matrix(sub_euler[0], sub_euler[1], sub_euler[2])

        #final = camP*rotation_matrix+ subP

        #val = Vector3()

        val.x = x[i0][0] #absolute position of objects
        val.y = x[i0 + 1][0]
        val.z = x[i0 + 2][0]
        
        #print("val",x[i0][0])

        # calculate varience of objects' position
        
        v = np.array([P[i0],P[i0+1],P[i0+2]])
        var_final = np.linalg.multi_dot([v.T,rotation_matrix])
        
        #var = Vector3()

        var.x = var_final[0][0]
        var.y = var_final[1][0]
        var.z = var_final[2][0]

        
    def update_world_msg(self):
        #position msg part
        self.world_msg = WorldState()
        self.fill_vector_value_variance(self.world_msg.position,
                                        self.world_msg.pos_variance,
                                        self.x, self.P, IDx.SPx)
        #orientation msg part
        quat = Quaternion()
        quat.x = self.x[IDx.Ox]
        quat.y = self.x[IDx.Oy]
        quat.z = self.x[IDx.Oz]
        quat.w = self.x[IDx.Ow]

        self.world_msg.orientation_quat = quat
        self.world_msg.orientation_quat_variance.w = self.P[IDx.Ow, IDx.Ow]
        self.world_msg.orientation_quat_variance.x = self.P[IDx.Ox, IDx.Ox]
        self.world_msg.orientation_quat_variance.y = self.P[IDx.Oy, IDx.Oy]
        self.world_msg.orientation_quat_variance.z = self.P[IDx.Oz, IDx.Oz]

        #obj position msg part
        #print(self.x[self.nState:self.nState+(self.numObjects-1)*3])
        self.world_msg.obj_positions = []
        self.world_msg.obj_pos_variances = []
        for i in range(0,self.numObjects):
            self.world_msg.obj_positions.append(Point())
            self.world_msg.obj_pos_variances.append(Vector3())
            self.fill_vector_value_variance_objects(self.world_msg.obj_positions[i],
                                            self.world_msg.obj_pos_variances[i],
                                            self.x, self.P, self.nState + i*3)

    def run(self):
        while not rospy.is_shutdown():
            self.rate_ctrl.sleep()

            self.output()

            self.prediction()
            self.update()
            self.update_world_msg()
            self.world_pub.publish(self.world_msg)
