#!/usr/bin/env python2

import rospy
import rospkg
import numpy as np

from thruster_control.thruster_types import BlueRoboticsT100




if __name__ == "__main__":
    spec = BlueRoboticsT100(400)
    spec.initialize()

    for i in range(-5, 5):
        spec.apply_thrust(i)


