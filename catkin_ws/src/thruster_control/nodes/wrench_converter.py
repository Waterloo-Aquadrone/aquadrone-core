#!/usr/bin/env python

import rospy
import numpy as np

from thruster_control.configurations.v2_configuration import get_wrench_to_thrusts_lb_in

from aquadrone_msgs.msg import MotorControls
from geometry_msgs.msg import Wrench


class CommandSubscriber:
    def __init__(self, topic):
        self.cmd = Wrench()
        self.time = rospy.Time()
        self.sub = rospy.Subscriber(topic, Wrench, self.callback)

    def callback(self, msg):
        self.cmd = msg
        self.time = rospy.Time()

    def get_cmd(self):
        return self.cmd, (rospy.Time() - self.time).to_sec()


class WrenchConverter:

    def __init__(self, w_to_t):
        self.transform = w_to_t

        self.sources = []
        self.sources.append(CommandSubscriber("movement_command"))
        self.sources.append(CommandSubscriber("depth_command"))

        self.publisher = rospy.Publisher("motor_command", MotorControls, queue_size=0)

    def run(self):
        while not rospy.is_shutdown():
            w = self.get_empty_thrusts()

            for source in self.sources:
                w = w + self.add_source_command(source)

            self.publish_command(w)

            rospy.sleep(0.1)

    def add_source_command(self, source):
        cmd, dt = source.get_cmd()
        if dt > 0.5:
            cmd = cmd * 0.0
        return self.wrench_to_np_array(cmd)


    def publish_command(self, force):
        thrusts = np.dot(self.transform, force)
        msg = MotorControls()
        msg.motorThrusts = thrusts
        self.publisher.publish(msg)

    def get_empty_thrusts(self):
        return self.wrench_to_np_array(Wrench())
    
    def wrench_to_np_array(self, wrench):
        return np.array([wrench.force.x, wrench.force.y, wrench.force.z, 
                         wrench.torque.x, wrench.torque.y, wrench.torque.z])

if __name__ == "__main__":
    rospy.init_node("wrench_transformer")
    sc = WrenchConverter(get_wrench_to_thrusts_lb_in())

    sc.run()