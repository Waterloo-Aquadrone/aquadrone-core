#!/usr/bin/env python3.8

import rospy
import rospkg
import json

from thrust_computer.thruster_configurations import V28Configuration
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


if __name__ == "__main__":
    # rospy.init_node('sim_thruster_controller')

    config = V28Configuration()

    pwm_frequency = 400
    rospack = rospkg.RosPack()
    with open(rospack.get_path('thruster_control') + "/config/thruster_gpio.json") as gpio_json:
        gpio_config = json.load(gpio_json)

    thrusters = []
    for i in range(config.get_num_thrusters()):
        ThrusterClass = config.get_thruster_class(i)
        thrusters.append(ThrusterClass(pwm_frequency, i, gpio_config[i]['gpio']))

    def send_command(idx, T):
        thrust = FloatStamped()
        thrust.data = T
        thrusters[idx].apply_thrust(thrust)

    THRUST = 0.5

    while not rospy.is_shutdown():
        for i in range(config.get_num_thrusters()):
            if rospy.is_shutdown():
                break

            print("Testing thruster %d" % i)

            for th in range(0, 30):
                send_command(i, th / 10.0)
                rospy.sleep(0.1)

            rospy.sleep(3.0)

            send_command(i, 0)
            rospy.sleep(3.0)

            for th in range(0, 30):
                send_command(i, -th/10.0)
                rospy.sleep(0.1)
        
            rospy.sleep(3.0)

            send_command(i, 0)
            rospy.sleep(3.0)
