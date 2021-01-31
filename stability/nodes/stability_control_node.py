#!/usr/bin/env python3

import rospy

from stability.stability_pid_controller import StabilityPIDController


if __name__ == "__main__":
    rospy.init_node('stability_pid')
    pid = StabilityPIDController()
    pid.run()
