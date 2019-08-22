#!/usr/bin/env python

import rospy
import numpy as np

from aquadrone_msgs.msg import MotorControls
from geometry_msgs.msg import Wrench


class ThrustCommandAllocator:
    # Uses the mcc to gather all commands to sub
    # Then uses configuration information to
    #  transform wrench commands into thruster thrusts
    
    def __init__(self, w_to_t, mcc):
        self.transform = w_to_t
        self.mcc = mcc

        self.publisher = rospy.Publisher("motor_command", MotorControls, queue_size=0)

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            rospy.sleep(0.1)

    def control_loop(self):
        w = self.mcc.get_recent_thrusts()
        print(w)
        self.publish_command(w)


    def publish_command(self, force):
        thrusts = np.dot(self.transform, force)
        th = np.ndarray.tolist(thrusts)[0]

        msg = MotorControls()
        msg.motorThrusts = list(th)
        self.publisher.publish(msg)


