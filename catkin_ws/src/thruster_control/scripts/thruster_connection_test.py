#!/usr/bin/env python2

import rospy
import rospkg
import numpy as np

from aquadrone_msgs.msg import MotorControls
from thruster_control.thruster_collection_manager import ThrusterCollectionManager
from thruster_control.thruster_interfaces import SimulatedThrusterInterface

import board
import busio
import adafruit_pca9685


if __name__ == "__main__":
    rospy.init_node('real_thruster_tester')
    pub = rospy.pub("motor_command", MotorControls, queue_size=1)


    def send_command(i, T):
        msg = MotorControls()
        msg.motorThrusts[i] = T
        pup.publish(msg)

    for i in range(0, 6):
        print("Testing thruster %d" % i)

        send_command(i, 1)
        rospy.sleep(1.0)

        send_command(i, -1)
        rospy.sleep(1.0)
    
        send_command(i, 0)
        rospy.sleep(3.0)
