#import RPi.GPIO as GPIO

import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped




class SimulatedThrusterInterface:
    def __init__(self, id):
        self.pub = rospy.Publisher("/aquadrone_v2/thrusters/%d/input" % id, FloatStamped, queue_size=2)
        self.msg = FloatStamped()

    def command(self, th):
        self.msg.data = th
        self.pub.publish(self.msg)