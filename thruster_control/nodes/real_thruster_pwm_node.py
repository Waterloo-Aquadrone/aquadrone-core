#!/usr/bin/env python

import rospy
import rospkg
import json

from thruster_control.thrust_computer.thruster_configurations import get_thruster_count, get_thruster_class


"""
This node is responsible for linking the topics where the thrusts for individual motors are published with the 
ThrusterType objects that will process those thrusts and send the corresponding PWM signals.
This node should only be run for the real thrusters, not in simulation!
It will read the model name and use the appropriate ThrusterConfiguration to determine the number and type of thrusters.
"""


if __name__ == "__main__":
    rospy.init_node('real_thruster_output')

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    num = get_thruster_count(model)

    rospack = rospkg.RosPack()
    with open(rospack.get_path('thruster_control') + "/config/thruster_gpio.json") as gpio_json:
        gpio_config = json.load(gpio_json)

    pwm_frequency = 400
    thrusters = []
    for i in range(num):
        ThrusterClass = get_thruster_class(i)
        thrusters.append(ThrusterClass(pwm_frequency, i, gpio_config[i]['gpio']))

    rospy.spin()
