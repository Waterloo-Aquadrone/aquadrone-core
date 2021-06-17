#!/usr/bin/env python3.8

from real_thruster_types.thruster_types import T100Thruster
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


if __name__ == "__main__":
    pwm_frequency = 400
    gpio = 1
    thruster = T100Thruster(pwm_frequency, 0, gpio)

    for i in range(-5, 5):
        thrust = FloatStamped()
        thrust.data = i
        thruster.apply_thrust(thrust)
