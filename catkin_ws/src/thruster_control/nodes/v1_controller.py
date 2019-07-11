#!/usr/bin/env python

import rospy
import numpy as np

from thruster_control.wrench_converter import WrenchConverter
from thruster_control.configurations import v1_configuration

from aquadrone_msgs.msg import MotorControls
from geometry_msgs.msg import Wrench

if __name__ == "__main__":
    rospy.init_node("sub_controller")
    sc = WrenchConverter(v1_configuration.get_wrench_to_thrusts_lb_in())

    sc.run()