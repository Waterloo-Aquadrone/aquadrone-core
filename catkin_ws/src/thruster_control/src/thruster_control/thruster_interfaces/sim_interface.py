#import RPi.GPIO as GPIO

import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class SimulatedThrusterInterface:
    def __init__(self, ns, n):
        self.ns = ns
        self.n = n
        self.publishers = None
        self.msg = FloatStamped()

    def initialize(self):
        self.publishers = []
        for i in range(0, self.n):
            pub = rospy.Publisher("/%s/thrusters/%d/input" % (self.ns, i), FloatStamped, queue_size=1)
            self.publishers.append(pub)
        
    def command(self, idx, sig):
        self.msg.data = sig
        self.publishers[idx].publish(self.msg)
