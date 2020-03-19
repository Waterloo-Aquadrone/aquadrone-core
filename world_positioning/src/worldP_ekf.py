import rospy
import math
import scipy.linalg

import autograd.numpy as np
from autograd import grad, jacobian, elementwise_grad

from geometry_msgs.msg import Point, Vector3, Quaternion
from aquadrone_msgs.msg import SubState, WorldState

from worldP_indicies import IDx as IDx
from worldP_listener import SubStateListener, ZedCamListener

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

        self.nState = 13 #number of elements in state

        #n and m are only for 1 point
        self.n = self.nState+3 #substate + world x,y,z of object
        self.m = self.nState+3 #sub state + x,y,z of object in camera

        self.x = np.zeros((self.n,1))
        self.x[IDx.Ow] = 1

        self.u = np.zeros((self.m, 1))

        self.B = np.array() #input modification matrix

        self.P = np.eye(self.n)
        self.Q = np.eye(self.n) * 0.01 #uncert in dynamics model

        self.camera_info = ZedCamListener() 
        self.state_info = SubStateListener()

        self.state_msg = SubState()
        self.camera_msg = Camera() #need to implement this for camera
        self.world_msg = WorldState()

        self.rate = 20
        self.rate_ctrl = rospy.Rate(self.rate)

        self.calc_F = jacobian(self.f)

        #self.world_pos_msg = NotImplemented #implement worldpositioning message
        self.world_pub = rospy.Publisher("world state", NotImplemented, queue_size = 1)
        #should be in listener already
        #self.sub_state_sub = rospy.Subscriber("state estimation", NotImplemented, NotImplemented)

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

    def Yaw(self, y):
        return np.matrix([
                [math.cos(y), -math.sin(y), 0],
                [math.sin(y),  math.cos(y), 0],
                [          0,            0, 1]
                ])

    def Pitch(self, p):
        return np.matrix([
                [ math.cos(p), 0, math.sin(p)],
                [               0, 1,               0],
                [-math.sin(p), 0, math.cos(p)]
                ])

    def Roll(self, r):
        return np.matrix([
                [1,              0,               0],
                [0, math.cos(r), -math.sin(r)],
                [0, math.sin(r),  math.cos(r)]
                ])

    def RPY_Matrix(self, r, p, y):
        r = Roll(r)
        p = Pitch(p)
        y = Yaw(y)
        return np.linalg.multi_dot([y, p, r])
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

        def read_listener_state(listener, z, h, H, R):
            if listener.is_valid():
                meas = listener.get_measurement_z()
                z = np.append(z,np.array([meas]))
                pred = listener.state_to_measurement_h(self.x, self.u)
                h=np.append(h,np.array([pred]))

                H = add_block_vert(H, listener.get_H(self.x, self.u))
                R = add_block_diag(R, listener.get_R())

                return z,h,H,R

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
                """
                local_x = np.array([local_var[0],0,0]) #making local x var vector
                local_y = np.array([0,local_var[1],0]) #making local y var vector
                local_z = np.array([0,0,local_var[2]]) #making local z var vector
                """
                world_var = local_var*rotation_matrix #converting local variance to world frame variance
                R = add_block_diag(R, world_var) #TODO need to implement localizing of uncert

                return z,h,H,R
        #state
        try:
            z,h,H,R = read_listener_state(self.state_info,z,h,H,R)
        except TypeError as e:
            print(e)
            return
        #camera
        try: 
            z,h,H,R = read_listener_camera(self.camera_info,z,h,H,R)
        except TypeError as e:
            print(e)
            return

        if R.shape[0] == 0:
            return

        #error in measurments vs predicted
        y = z-h
        y.shape = (y.shape[0],1)

        #calc kalman gain
        Ht = np.transpose[H]
        S = np.dot(np.dot(H,self.P),Ht)+R
        K = np.dot(np.dot(self.P,Ht),np.linalg.inv(S))

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
        dt = t-self.last_prediction_t()
        self.last_prediction_t = t
                
        #find new position
        new_pos = np.array([x[IDx.SPx] + dt*x[IDx.SVx],
                            x[IDx.SPy] + dt*x[IDx.SVy],
                            x[IDx.SPz] + dt*x[IDx.SVz]])

        #find new orientation in euler
        new_orient = self.update_orientation_quaternion(x)
        new_vel = np.array([x[IDx.SVx],x[IDx.SVy],x[IDx.SVz]])
        new_ang_vel = np.array([x[IDx.Ax],x[IDx.Ay],x[IDx.Az]])

        new_obj_pos = np.array.zeros(x.shape[0]-self.nState)
        for i in range(self.nState,x.shape[0]):
            new_obj_pos[i-self.nState] = x[i]

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
        
    def update_world_msg(self):
        #position msg part
        self.fill_vector_value_variance(self.world_msg.position,
                                        self.world_msg.pos_variance,
                                        self.x, self.P, IDx.Px)
        #orientation msg part
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

        #obj position msg part
        self.fill_vector_value_variance(self.world_msg.position,
                                        self.world_msg.pos_variance,
                                        self.x, self.P, self.nState)

    def run(self):
        while not rospy.is_shutdown():
            self.rate_ctrl.sleep()

            self.output()

            self.prediction()
            self.update()
            self.update_world_msg()
            self.world_pub.publish(self.world_msg)