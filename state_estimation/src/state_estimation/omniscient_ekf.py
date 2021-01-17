#!/usr/bin/env python

import rospy
from aquadrone_msgs.msg import SubState, WorldObjectState, WorldState
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger, TriggerResponse
import aquadrone_math_utils.orientation_math as OMath


class OmniscientEKF:
    # the default names in gazebo for the objects to track
    DEFAULT_WORLD_MODEL_NAMES = ['pole']

    def __init__(self, sub_model_name='aquadrone', world_model_names=None):
        self.sub_model_name = sub_model_name
        self.world_model_names = world_model_names if world_model_names is not None \
            else OmniscientEKF.DEFAULT_WORLD_MODEL_NAMES

        rospy.Subscriber("gazebo/model_states", ModelStates, self.get_obj_pos, queue_size=1)

        self.sub_state_pub = rospy.Publisher("state_estimation", SubState, queue_size=1)
        self.world_state_pub = rospy.Publisher("world_state_estimation", WorldState, queue_size=1)

        rospy.Service('reset_sub_state_estimation', Trigger, self.reset_ekf)
        rospy.Service('reset_world_state_estimation', Trigger, self.reset_ekf)

    @staticmethod
    def reset_ekf(msg=None):
        return TriggerResponse(success=True, message="nothing to reset")

    def get_obj_pos(self, model_states):
        world_state = WorldState()

        for name, pose, twist in zip(model_states.name, model_states.pose, model_states.twist):
            if name == self.sub_model_name:
                self.publish_sub_state(pose, twist)
            elif name in self.world_model_names:
                world_state.data.append(self.create_object_state(name, pose))

        self.world_state_pub.publish(world_state)

    def publish_sub_state(self, pose, twist):
        # Variances will all be set to 0 by default, which is the desired behaviour
        msg = SubState()
        msg.position = pose.position
        msg.velocity = twist.linear
        msg.orientation_quat = pose.orientation
        msg.orientation_RPY = OMath.msg_quaternion_to_euler(pose.orientation)
        msg.ang_vel = twist.angular

        self.sub_state_pub.publish(msg)

    @staticmethod
    def create_object_state(name, pose):
        object_state = WorldObjectState()
        object_state.identifier = name
        object_state.pose_with_covariance.pose = pose
        return object_state

    @staticmethod
    def run():
        rospy.spin()
