#!/usr/bin/env python3


import rospy
from stability.orientation_pid_controller import OrientationPIDController


if __name__ == "__main__":
    rospy.init_node('orientation_pid')

    pid = OrientationPIDController()
    pid.run()
