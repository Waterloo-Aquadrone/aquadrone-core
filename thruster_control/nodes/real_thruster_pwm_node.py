#!/usr/bin/env python

import rospy
import rospkg
import json

from thruster_control.thruster_configurations import get_thruster_count
from thruster_control.T100Thruster import T100Thruster


if __name__ == "__main__":
    rospy.init_node('real_thruster_output')

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    num = get_thruster_count(model)

    rospack = rospkg.RosPack()
    with open(rospack.get_path('thruster_control') + "/config/thruster_gpio.json") as gpio_json:
        gpio_config = json.load(gpio_json)

    pwm_frequency = 400
    # TODO: add new class for different thruster type (if necessary)
    thrusters = [T100Thruster(pwm_frequency, i, gpio_config[i]['gpio']) for i in range(num)]

    rospy.spin()
