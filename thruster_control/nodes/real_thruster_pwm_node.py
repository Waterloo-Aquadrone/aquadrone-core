#!/usr/bin/env python3

import rospy
import rospkg
import json

# import RPi.GPIO as GPIO
import board
import busio
import adafruit_pca9685

from thruster_control.thrust_computer.thruster_configurations import ThrusterConfiguration


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
    config = ThrusterConfiguration(model)
    config.initialize()

    rospack = rospkg.RosPack()
    with open(rospack.get_path('thruster_control') + "/config/thrusters_" + model + ".json") as thruster_json:
        thruster_config = json.load(thruster_json)
    i2c_data = thruster_config['i2c_data']

    if len(i2c_data) != config.get_num_thrusters():
        raise Exception('Invalid config! Expected ' +
                        str(config.get_num_thrusters()) + ' entries but got ' + str(len(i2c_data)))

    pwm_frequency = 60

    i2c = busio.I2C(board.SCL, board.SDA)
    pca = adafruit_pca9685.PCA9685(i2c)
    pca.frequency = pwm_frequency

    thrusters = []
    for i in range(config.get_num_thrusters()):
        ThrusterClass = config.get_thruster_class(i)
        thrusters.append(ThrusterClass(pwm_frequency, i, pca.channels[i2c_data[i]['i2c_index']]))

    rospy.spin()
