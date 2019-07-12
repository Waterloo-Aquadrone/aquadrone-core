#!/usr/bin/env python

import rospy
import numpy as np

from thruster_control.movement_command_collector import MovementCommandCollector
from thruster_control.thrust_control_allocator import ThrustCommandAllocator
from thruster_control.configurations import v1_configuration

from aquadrone_msgs.msg import MotorControls
from geometry_msgs.msg import Wrench

if __name__ == "__main__":
    rospy.init_node("sub_controller")

    mcc = MovementCommandCollector()

    transform_mat = v1_configuration.get_wrench_to_thrusts_lb_in()
    tca = ThrustCommandAllocator(transform_mat, mcc)

    tca.run()