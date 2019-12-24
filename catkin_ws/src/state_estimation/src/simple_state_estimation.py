#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, FluidPressure

from aquadrone_msgs.msg import SubState

class StateEstimator:

    def __init__(self):
        self.position = Point()
        self.pos_variance = Vector3(*(1, 1, 1))

        self.velocity = Vector3()
        self.vel_variance = Vector3(*(1, 1, 1))

        self.orientation = Quaternion()
    
        self.imu_sub = None
        self.depth_sub = None
        self.state_pub = None
        self.state_msg = SubState()

        self.pressure_offset = 100.0
        self.g = 9.8

        self.rate = rospy.Rate(10)

    def imu_cb(self, msg):
        self.orientation = msg.orientation

    def depth_cb(self, msg):
        press = msg.fluid_pressure
        var = msg.variance
        self.position.z = -self.pressure_to_depth(press)
        self.pos_variance.z = var / self.g

    def pressure_to_depth(self, press):
        return (press - self.pressure_offset) / self.g

    def initialize(self):
        self.imu_sub = rospy.Subscriber("aquadrone/out/imu", Imu, self.imu_cb)
        self.depth_sub = rospy.Subscriber("aquadrone/out/pressure", FluidPressure, self.depth_cb)
        self.state_pub = rospy.Publisher("state_estimation", SubState, queue_size=1)

    def orientation_to_euler(self):
        # From wikipedia (https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

        orientation = self.orientation
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


    def publish(self):
        msg = SubState()
        msg.position = self.position
        msg.pos_variance = self.pos_variance
        msg.velocity = self.velocity
        msg.vel_variance = self.vel_variance
        msg.orientation_quat = self.orientation
        msg.orientation_RPY = self.orientation_to_euler()
        print(msg)
        self.state_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():

            self.publish()
            self.rate.sleep()
            

if __name__ == "__main__":
    rospy.init_node("simple_state_estimation")

    se = StateEstimator()
    se.initialize()
    se.run()