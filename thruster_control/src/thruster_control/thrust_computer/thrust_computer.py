#!/usr/bin/env python

import rospy
from aquadrone_msgs.msg import MotorControls
from thruster_control.thrust_computer.movement_command_collector import MovementCommandCollector


class ThrustComputer:
    """
    Creates a MovementCommandCollector to listen to the relevant topics for thrust commands.
    Calculates the required individual motor thrusts to achieve the desired Wrench (using the ThrusterConfiguration
    supplied on construction), and publishes the resulting MotorControls message to the motor_command topic.
    This node may be bypassed (not turned on) if fine grained control of individual thrusters is desired, such as for a
    diagnostic test.
    """

    def __init__(self, config, rate=rospy.Rate(10)):
        self.config = config
        self.mcc = MovementCommandCollector()
        self.rate = rate

        # Will need motor commands published for state estimation
        self.publisher = rospy.Publisher("motor_command", MotorControls, queue_size=0)

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    def control_loop(self):
        wrench = self.mcc.get_recent_thrusts()
        thrusts = self.config.wrench_to_thrusts(wrench)
        self.publish_command(thrusts)

    def publish_command(self, thrusts):
        msg = MotorControls()
        msg.motorThrusts = [float(thrust) for thrust in thrusts]
        self.publisher.publish(msg)
