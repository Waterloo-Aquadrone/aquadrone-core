import rospy
import cv2
import numpy as np
import time

from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Vector3, Wrench
from sensor_msgs.msg import Image

import data_structures as DS

from aquadrone_msgs.msg import SubState, MotorControls


class ROSControlsModule:
    def __init__(self):
        self.depth_pub = rospy.Publisher("/depth_control/goal_depth", Float64, queue_size=1)
        self.orientation_pub = rospy.Publisher("orientation_target", Vector3, queue_size=1)
        self.planar_move_pub = rospy.Publisher("/movement_command", Wrench, queue_size=1)
        self.motor_command_pub = rospy.Publisher("/motor_command", MotorControls, queue_size=0)

    def set_depth_goal(self, d):
        self.depth_pub.publish(d)

    @staticmethod
    def normalize_angle(angle):
        while angle > 2 * np.pi:
            angle -= 2 * np.pi
        while angle < 0:
            angle += 2 * np.pi
        return angle

    def set_orientation_goal(self, r=0, p=0, y=0):
        target = Vector3()
        target.x = ROSControlsModule.normalize_angle(r)
        target.y = ROSControlsModule.normalize_angle(p)
        target.z = ROSControlsModule.normalize_angle(y)
        self.orientation_pub.publish(target)

    def set_roll_goal(self, roll):
        """
        Commands sent using this method will be persistent. Even if the active state in the state machine changes, the
        command will continue to be in effect until another one of the same type overrides it.
        Implicitly sets pitch and yaw to 0.

        :param roll:
        """
        target = Vector3()
        target.x = ROSControlsModule.normalize_angle(roll)
        self.orientation_pub.publish(target)

    def set_yaw_goal(self, yaw):
        """
        Implicitly sets pitch and roll to 0.

        :param yaw:
        """
        target = Vector3()
        target.x = 0
        target.y = 0
        target.z = ROSControlsModule.normalize_angle(yaw)
        self.orientation_pub.publish(target)

    def planar_move_command(self, Fx=0, Fy=0, Tz=0):
        """
        Commands sent using this method will expire after a configurable amount of time. This should be repeated called
        in the process loop.

        :param Fx:
        :param Fy:
        :param Tz:
        """
        w = Wrench()
        w.force.x = Fx
        w.force.y = Fy
        w.force.z = 0
        w.torque.x = 0
        w.torque.y = 0
        w.torque.z = Tz
        self.planar_move_pub.publish(w)

    def send_direct_motor_thrusts(self, thrusts):
        """
        This command will only work if the thrust_computer node is not running.
        Otherwise, this command will be immediately be overwritten.

        :param thrusts: Array of 8 floats for the 8 motors.
        """
        msg = MotorControls()
        msg.motorThrusts = thrusts
        self.motor_command_pub.publish(msg)

    def halt_and_catch_fire(self):
        """
        Immediately stops all thruster outputs. The sub will naturally rise to the surface via buoyancy,
        and cannot be controlled again until everything is restarted.
        """
        # TODO: setup thrust_distributor.py to have a service that sets all thrusts to 0, then disables future requests.
        #  Alternatively, use a shutdown hook
        pass


class ROSStateEstimationModule:
    def __init__(self):
        self.sub_state_sub = rospy.Subscriber("/state_estimation", SubState, self.sub_state_callback)
        self.sub_state = SubState()

        rospy.wait_for_service('reset_sub_state_estimation')
        self.state_est_reset_service = rospy.ServiceProxy('reset_sub_state_estimation', Trigger)

    def sub_state_callback(self, msg):
        self.sub_state = msg

    def get_submarine_state(self):
        position = DS.Vector.from_msg(self.sub_state.position)
        velocity = DS.Vector.from_msg(self.sub_state.velocity)
        orientation_quat = DS.Quaternion.from_msg(self.sub_state.orientation_quat)
        orientation_rpy = DS.Vector.from_msg(self.sub_state.orientation_RPY)
        ang_vel = DS.Vector.from_msg(self.sub_state.ang_vel)

        orientation_rpy.x = ROSControlsModule.normalize_angle(orientation_rpy.x)
        orientation_rpy.y = ROSControlsModule.normalize_angle(orientation_rpy.y)
        orientation_rpy.z = ROSControlsModule.normalize_angle(orientation_rpy.z)

        position_var = DS.Vector.from_msg(self.sub_state.pos_variance)
        velocity_var = DS.Vector.from_msg(self.sub_state.vel_variance)
        orientation_quat_var = DS.Quaternion.from_msg(self.sub_state.orientation_quat_variance)
        orientation_rpy_var = DS.Vector.from_msg(self.sub_state.orientation_RPY_variance)
        ang_vel_var = DS.Vector.from_msg(self.sub_state.ang_vel_variance)

        out = DS.Submarine(position, velocity, orientation_quat, orientation_rpy, ang_vel)
        out.set_uncertainties(position_var, velocity_var, orientation_quat_var, orientation_rpy_var, ang_vel_var)

        return out

    def reset_state_estimation(self):
        req = TriggerRequest()
        self.state_est_reset_service(req)


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

        # TODO: replace with numpy operations (likely reshape)
        for y in range(0, height):
            for x in range(0, width):
                for p in range(0, 3):
                    idx = y*width*3 + x*3 + p
                    # print(idx)
                    dat = ord(bytes(self.main_cam_image.data[idx]))
                    # print(dat)
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
            cv2.imshow('image', image)
            cv2.waitKey(0)
        time.sleep(0.25)
    exit()
