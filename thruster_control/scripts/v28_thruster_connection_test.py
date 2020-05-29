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

    spec = BlueRoboticsT100(interface.get_frequency())
    spec.initialize()
    specs = [spec, spec, spec, spec, spec, spec, spec, spec]

    interface.init_gpio()
    
    interface.init_thrusters(specs)

    

    def send_command(i, T):
        sig = spec.apply_thrust(T)
        interface.command(i, sig)

    THRUST = 0.5

    while not rospy.is_shutdown():
        for i in range(0, 1):
            if rospy.is_shutdown():
                break

            print("Testing thruster %d" % i)

            for th in range(0, 30):
                send_command(i, th/10.0)
                rospy.sleep(0.1)

            rospy.sleep(3.0)

            send_command(i, 0)
            rospy.sleep(3.0)


            for th in range(0, 30):
                send_command(i, -th/10.0)
                rospy.sleep(0.1)
        
            rospy.sleep(3.0)

            send_command(i, 0)
            rospy.sleep(3.0)
