#!/usr/bin/env python
import rospy
from simple_pid import PID

from thruster_control.depth_pid_controller import DepthPIDController


if __name__ == "__main__":
    ddc = DepthPIDController()
    ddc.run()