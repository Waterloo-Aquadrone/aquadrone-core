#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Wrench


class CommandSubscriber:
    # Should listen to a wrench command from some source topic
    # Track the latest command, as well as the last time it was received

    def __init__(self, topic):
        self.cmd = Wrench()
        self.time = rospy.Time()
        self.sub = rospy.Subscriber(topic, Wrench, self.callback)

        self.cmd_timeout = 0.5

    def callback(self, msg):
        self.cmd = msg
        self.time = rospy.Time()

    def get_cmd(self):
        return self.cmd, (rospy.Time() - self.time).to_sec()


class MovementCommandCollector:
    def __init__(self):
        # Set up command sources
        # First added is highest priority
        self.sources = []
        self.sources.append(CommandSubscriber("movement_command"))
        self.sources.append(CommandSubscriber("depth_command"))

    def get_recent_thrusts(self, drop=0):
        w = self.get_empty_wrench()

        # Add commands from each source
        for i in range(0, len(self.sources) - drop):
            w = w + self.add_source_command(self.sources[i])

        return w

    def add_source_command(self, source):
        # Return the latest command from the source
        # Set to 0 if not received for some time
        cmd, dt = source.get_cmd()
        cmd = self.wrench_to_np_array(cmd)
        if dt > self.cmd_timeout:
            cmd = cmd * 0.0
        return cmd

    def get_empty_wrench(self):
        return self.wrench_to_np_array(Wrench())
    
    def wrench_to_np_array(self, wrench):
        return np.array([wrench.force.x, wrench.force.y, wrench.force.z, 
                         wrench.torque.x, wrench.torque.y, wrench.torque.z])


