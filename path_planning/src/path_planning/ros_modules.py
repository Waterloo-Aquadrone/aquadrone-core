import rospy
import cv2
import numpy as np
import time

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Wrench
from sensor_msgs.msg import Image

import data_structures  as DS

from aquadrone_msgs.msg import SubState


class ROSControlsModule:
    def __init__(self):
        self.depth_pub = rospy.Publisher("/depth_control/goal_depth", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("orientation_target", Vector3, queue_size=1)
        self.planar_move_pub = rospy.Publisher("/movement_command", Wrench, queue_size=1)

    def set_depth_goal(self, d):
        self.depth_pub.publish(d)

    def set_yaw_goal(self, y):
        target = Vector3()
        target.x = 0
        target.y = 0
        target.z = y
        self.yaw_pub.publish(target)

    def planar_move_command(self, Fx=0, Fy=0, Tz=0):
        w = Wrench()
        w.force.x = Fx
        w.force.y = Fy
        w.force.z = 0
        w.torque.x = 0
        w.torque.y = 0
        w.torque.z = Tz
        self.planar_move_pub.publish(w)


class ROSStateEstimationModule:
    def __init__(self):
        self.sub_state_sub = rospy.Subscriber("/state_estimation", SubState, self.sub_state_callback)
        self.sub_state = SubState()

    def sub_state_callback(self, msg):
        self.sub_state = msg

    def get_submarinne_state(self):

        def make_vector(msg):
            out = DS.Vector(0, 0, 0)
            out.from_msg(msg)
            return out
        def make_quat(msg):
            out = DS.Quaternion()
            out.from_msg(msg)
            return out

        position = make_vector(self.sub_state.position)
        velocity = make_vector(self.sub_state.velocity)
        orientation_quat = make_quat(self.sub_state.orientation_quat)
        orientation_rpy = make_vector(self.sub_state.orientation_RPY)
        ang_vel = make_vector(self.sub_state.ang_vel)

        position_var = make_vector(self.sub_state.pos_variance)
        velocity_var = make_vector(self.sub_state.vel_variance)
        orientation_quat_var = make_quat(self.sub_state.orientation_quat_variance)
        orientation_rpy_var = make_vector(self.sub_state.orientation_RPY_variance)
        ang_vel_var = make_vector(self.sub_state.ang_vel_variance)

        out = DS.Submarine(position, velocity, orientation_quat, orientation_rpy, ang_vel)
        out.set_uncertainties(position_var, velocity_var, orientation_quat_var, orientation_rpy_var, ang_vel_var)

        return out


class ROSSensorDataModule:
    def __init__(self):
        self.main_cam_image_sub = rospy.Subscriber("/aquadrone/out/front_cam/image_raw", Image, self.image_callback)
        self.main_cam_image = None
        print("init'd")

    def image_callback(self, msg):
        self.main_cam_image = msg
    
    def get_main_cam_image(self):
        if self.main_cam_image is None:
            return None

        height = self.main_cam_image.height
        width = self.main_cam_image.width
        image = np.zeros((height, width, 3), np.uint8)

        for y in range(0, height):
            for x in range(0, width):
                for p in range(0, 3):
                    idx = y*width*3 + x*3 + p
                    #print(idx)
                    dat = ord(bytes(self.main_cam_image.data[idx]))
                    #print(dat)
                    image[y][x][2-p] = int(dat)

        return image


if __name__ == "__main__":

    rospy.init_node("test_sensors")
    rsdm = ROSSensorDataModule()

    t0 = time.time()

    while time.time() - t0 < 3:
        image = rsdm.get_main_cam_image()
        if image is not None:
            pass
            cv2.imshow('image',image)
            cv2.waitKey(0)
        time.sleep(0.25)
    exit()