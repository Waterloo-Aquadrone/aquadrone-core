#!/usr/bin/env python

import rospy
from aquadrone_msgs.msg import SubState
from gazebo_msgs.msg import ModelStates
import aquadrone_math_utils.orientation_math as OMath


class OmniscientEKF:
    def __init__(self, sub_model_name='aquadrone'):
        self.sub_model_name = sub_model_name
        rospy.Subscriber("gazebo/model_states", ModelStates, self.get_obj_pos, queue_size=1)
        self.state_pub = rospy.Publisher("state_estimation", SubState, queue_size=1)

    def get_obj_pos(self, model_states):
        for name, pose, twist in zip(model_states.name, model_states.pose, model_states.twist):
            if name == self.sub_model_name:
                self.publish_state(pose, twist)
                break

    def publish_state(self, pose, twist):
        # Variances will all be set to 0 by default, which is the desired behaviour
        msg = SubState()
        msg.position = pose.position
        msg.velocity = twist.linear
        msg.orientation_quat = pose.orientation
        msg.orientation_RPY = OMath.msg_quaternion_to_euler(pose.orientation)
        msg.ang_vel = twist.angular

        self.state_pub.publish(msg)

    @staticmethod
    def run():
        rospy.spin()
