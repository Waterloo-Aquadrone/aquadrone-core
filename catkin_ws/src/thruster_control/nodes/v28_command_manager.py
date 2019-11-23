#!/usr/bin/env python

import rospy
import numpy as np

from thruster_control.movement_command_collector import MovementCommandCollector
from thruster_control.thrust_control_allocator import ThrustCommandAllocator
from thruster_control.configurations.v28_configuration import V28Configuration

from aquadrone_msgs.msg import MotorControls
from geometry_msgs.msg import Wrench

if __name__ == "__main__":
    rospy.init_node("sub_controller")

    mcc = MovementCommandCollector()

    config = V28Configuration()
    config.initialize()

    tca = ThrustCommandAllocator(config, mcc)
    tca.run()