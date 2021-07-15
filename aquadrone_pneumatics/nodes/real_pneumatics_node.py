#!/usr/bin/env python3

import rospy
from aquadrone_pneumatics.pneumatics_controller import PneumaticsController


if __name__ == "main":
    rospy.init_node('pneumatics_controller', log_level=rospy.DEBUG)

    pneumatics_controller = PneumaticsController(real=True)
    pneumatics_controller.run()
