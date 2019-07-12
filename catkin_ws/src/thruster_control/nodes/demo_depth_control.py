#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from sensor_msgs.msg import FluidPressure

class DemoDepthController:

    def __init__(self):
        rospy.init_node('depth_control')
        self.depth = 0
        self.goal = 5
        self.kP = 100.0

        # In future, take in state estimation, and not sensor reading
        self.depth_sub = rospy.Subscriber('aquadrone_v2/out/pressure', FluidPressure, self.depth_cb)

        self.pressure_offset = 100.0
        self.g = 9.8

        # In future, send to control accumulator node
        self.th_0_pup = rospy.Publisher('aquadrone_v2/thrusters/0/input', FloatStamped, queue_size=3)
        self.th_1_pup = rospy.Publisher('aquadrone_v2/thrusters/1/input', FloatStamped, queue_size=3)
        self.th_4_pup = rospy.Publisher('aquadrone_v2/thrusters/4/input', FloatStamped, queue_size=3)
        self.th_5_pup = rospy.Publisher('aquadrone_v2/thrusters/5/input', FloatStamped, queue_size=3)

        self.w_pub = rospy.Publisher('/depth_command', Wrench, queue_size=3)

        self.depth_sub = rospy.Subscriber("/depth_control/goal_depth", Float64, callback=self.goal_cb)

    def goal_cb(self, msg):
        self.goal = msg.data


    def depth_cb(self, msg):
        self.depth = self.pressure_to_m(msg.fluid_pressure)

    def pressure_to_m(self, p):
        return (p - self.pressure_offset) / self.g

    def run(self):

        while not rospy.is_shutdown():
            err = self.depth - self.goal
            u = self.kP * err

            self.publish_wrench(u)

            rospy.sleep(0.1)

    def publish_thrusts(self, u):
            self.send_command(self.th_0_pup, u)
            self.send_command(self.th_1_pup, u)

            self.send_command(self.th_4_pup, u)
            self.send_command(self.th_5_pup, u)

    def publish_wrench(self, u):
        msg = Wrench()
        msg.force.z = u
        self.w_pub.publish(msg)


    def send_command(self, pub, u):
        msg = FloatStamped()
        msg.data = u
        pub.publish(msg)



if __name__ == "__main__":
    ddc = DemoDepthController()
    ddc.run()