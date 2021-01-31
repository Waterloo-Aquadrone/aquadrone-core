#!/usr/bin/env python3
import rospy

from stability.angular_pid_controller import AngularPIDController


if __name__ == "__main__":
    rospy.init_node('orientation_pid')

    orientation_pid = AngularPIDController()
    orientation_pid.run()
