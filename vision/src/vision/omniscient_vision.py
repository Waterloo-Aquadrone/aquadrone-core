import rospy

import numpy as np
from gazebo_msgs.msg import ModelStates
from aquadrone_msgs.msg import WorldObjectState, WorldState
from aquadrone_math_utils.quaternion import Quaternion
from aquadrone_math_utils.ros_utils import make_vector, vector_to_np, make_quaternion, quaternion_to_np


class OmniscientVision:
    def __init__(self):
        rospy.Subscriber("gazebo/model_states", ModelStates, self.get_gazebo_data)
        self.publisher = rospy.Publisher("aquadrone/vision_data", WorldState, queue_size=1)

    def get_gazebo_data(self, data):
        sub_pose = None
        world_object_names = []
        world_object_poses = []

        for name, pose in zip(data.name, data.pose):
            if name == "aquadrone":
                sub_pose = pose
            else:
                world_object_names.append(name)
                world_object_poses.append(pose)

        if sub_pose is None:
            print('Warning: cannot publish omniscient data because submarine was not found!')
            return

        msg = WorldState()
        msg.data = [self.get_world_object_state(sub_pose, object_name, object_pose)
                    for object_name, object_pose in zip(world_object_names, world_object_poses)]
        self.publisher.publish(msg)

    @staticmethod
    def get_world_object_state(sub_pose, object_name, object_pose):
        """
        Create a WorldObjectState message for the provided object.
        The pose of the object will be transformed to the submarine-relative reference frame
        using the provided sub_pose.
        """
        sub_position = vector_to_np(sub_pose.position)
        sub_orientation = quaternion_to_np(sub_pose.orientation)
        object_position = vector_to_np(object_pose.position)

        relative_object_position = sub_orientation.unrotate(object_position - sub_position)

        msg = WorldObjectState()
        msg.identifier = object_name
        msg.pose_with_covariance.pose.position = make_vector(relative_object_position)
        # TODO: add computation of object's orientation quaternion (instead of just using the identity)
        msg.pose_with_covariance.pose.orientation = make_quaternion(Quaternion.identity())
        # gazebo does not report covariances, so just provide reasonable values
        msg.pose_with_covariance.covariance = 0.01 * np.eye(6).flatten()
        return msg

    @staticmethod
    def run():
        rospy.spin()
