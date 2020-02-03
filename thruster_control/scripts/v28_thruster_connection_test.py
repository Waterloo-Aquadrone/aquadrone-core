#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np

from thruster_control.thruster_interfaces.v28_interface import V28ThrusterInterface
from thruster_control.thruster_types import BlueRoboticsT100

import board
import busio
import adafruit_pca9685


if __name__ == "__main__":
    #rospy.init_node('sim_thruster_controller')

    interface = V28ThrusterInterface()
    interface.initialize()

    spec = BlueRoboticsT100(interface.get_frequency())
    spec.initialize()

    def send_command(i, T):
        sig = spec.thrust_to_signal(T)
        interface.command(i, sig)

    THRUST = 2

    for i in range(0, 6):
        if rospy.is_shutdown():
            break

        print("Testing thruster %d" % i)

        send_command(i, THRUST)
        rospy.sleep(1.0)

        send_command(i, -THRUST)
        rospy.sleep(1.0)
    
        send_command(i, 0)
        rospy.sleep(2.0)
