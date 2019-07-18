#!/usr/bin/env python
import rospy
from simple_pid import PID

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from sensor_msgs.msg import FluidPressure

class DemoDepthController:

    def __init__(self):
        rospy.init_node('depth_control')
        self.depth = 0

        self.Kp = 100
        self.Kd = 20
        self.Ki = 0.05

        self.loop_rate = 10.0
        self.rate = rospy.Rate(self.loop_rate)
        self.pid = PID(self.Kp, self.Ki, self.Kd, setpoint=5)
        self.pid.output_limits = (-50, 50)
        self.pid.sample_time = 1.0 / self.loop_rate 

        # In future, take in state estimation, and not sensor reading
        self.depth_sub = rospy.Subscriber('aquadrone_v2/out/pressure', FluidPressure, self.depth_cb)

        self.pressure_offset = 100.0
        self.g = 9.8

        self.w_pub = rospy.Publisher('/depth_command', Wrench, queue_size=3)
        self.depth_sub = rospy.Subscriber("/depth_control/goal_depth", Float64, callback=self.goal_cb)

    def goal_cb(self, msg):
        self.goal = msg.data
        self.pid.setpoint = self.goal


    def depth_cb(self, msg):
        self.depth = self.pressure_to_m(msg.fluid_pressure)

    def pressure_to_m(self, p):
        return (p - self.pressure_offset) / self.g

    def run(self):

        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    def control_loop(self):
        u = -self.pid(self.depth)
        self.publish_wrench(u)


    def publish_wrench(self, u):
        msg = Wrench()
        msg.force.z = u
        self.w_pub.publish(msg)



if __name__ == "__main__":
    ddc = DemoDepthController()
    ddc.run()