#!/usr/bin/env python3

import rospy
from stability.movement_pid_controller import MovementPIDController


if __name__ == "__main__":
    rospy.init_node('movement_pid')
    pid = MovementPIDController()
    pid.run()
