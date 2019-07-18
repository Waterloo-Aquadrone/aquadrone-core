#import RPi.GPIO as GPIO

import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped




class SimulatedThrusterInterface:
    def __init__(self, ns, id):
        self.pub = rospy.Publisher("/%s/thrusters/%d/input" % (ns, id), FloatStamped, queue_size=2)
        self.msg = FloatStamped()

    def command(self, th):
        self.msg.data = th
        self.pub.publish(self.msg)

    def emergency():
        self.msg.data = 0
        self.pub.publish(self.msg)