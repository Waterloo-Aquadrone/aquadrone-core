#!/usr/bin/env python3.8
import rospy
from aquadrone_sensors.sim_pressure_sensor import SimPressureSensor


if __name__ == '__main__':
    rospy.init_node('sim_pressure_sensor')

    sensor = SimPressureSensor()
    sensor.run()
