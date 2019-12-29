#!/usr/bin/env python

import rospy

from aquadrone_msgs.msg import MotorControls 

class SimThrusterController:
    def __init__(self, config, mcc, interface, thruster_specs):
        self.config = config
        self.mcc = mcc
        self.interface = interface
        self.thruster_specs = thruster_specs

        # Will need motor commands published for state estimation
        self.publisher = rospy.Publisher("motor_command", MotorControls, queue_size=0)


    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            rospy.sleep(0.1)

    def control_loop(self):
        w = self.mcc.get_recent_thrusts()
        thrusts = self.wrench_to_thrust_list(w)

        self.publish_command(thrusts)

        for i in range(0, self.config.get_num_thrusters()):
            thrust = thrusts[i]
            sig = self.thruster_specs[i].thrust_to_signal(thrust)
            self.interface.command(i, sig)

    def wrench_to_thrust_list(self, w):
        thrusts = self.config.wrench_to_thrusts(w)
        thrusts = thrusts.tolist()[0]
        return thrusts

    def publish_command(self, thrusts):
        msg = MotorControls()
        msg.motorThrusts = [float(th) for th in thrusts]
        self.publisher.publish(msg)
