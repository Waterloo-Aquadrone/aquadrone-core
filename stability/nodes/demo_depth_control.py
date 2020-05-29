#!/usr/bin/env python
import rospy
from simple_pid import PID

from thruster_control.depth_pid_controller import DepthPIDController


if __name__ == "__main__":
    rospy.init_node('depth_control')

    Kp = rospy.get_param('/stability/depth/Kp')
    Kd = rospy.get_param('/stability/depth/Kd')
    Ki = rospy.get_param('/stability/depth/Ki')

    ddc = DepthPIDController(Kp=Kp, Ki=Ki, Kd=Kd)
    ddc.run()
