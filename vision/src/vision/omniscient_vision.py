import rospy

from gazebo_msgs.msg import ModelStates
from aquadrone_msgs.msg import Vision_Array, Vision
from aquadrone_math_utils.ros_utils import make_vector, vector_to_np


class OmniscientVision:
    def __init__(self):
        self.relative_pos_pub = rospy.Publisher("Vision_Data", Vision_Array, queue_size=1)
        self.object_pos_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.get_obj_pos)
        self.world_object_poses = []
        self.world_object_names = []
        self.sub_pos = [0, 0, 0]
        self.relative_pos = []
        self.rate = rospy.Rate(20)
        self.pub_msg = Vision_Array()
        self.testing = False

    def get_obj_pos(self, data):
        sub_pose = []
        world_object_names = []
        world_object_poses = []
        if not self.testing:
            for name, pose in zip(data.name, data.pose):
                if name != "aquadrone":
                    world_object_poses.append(vector_to_np(pose.position))
                    world_object_names.append(name)
                else:
                    sub_pose = vector_to_np(pose.position)
        else:
            world_object_poses = [[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]
            sub_pose = [0, 0, 0]
            world_object_names = ["a", "a", "a", "a"]
        self.world_object_poses = world_object_poses
        self.world_object_names = world_object_names
        self.sub_pos = sub_pose

    def calc_rel_pos(self):
        self.relative_pos = []
        if len(self.world_object_poses) != 0 and len(self.sub_pos) != 0:
            for i in range(len(self.world_object_poses)):
                self.relative_pos.append([self.world_object_poses[i][0] - self.sub_pos[0],
                                          self.world_object_poses[i][1] - self.sub_pos[1],
                                          self.world_object_poses[i][2] - self.sub_pos[2]])
        else:
            self.relative_pos = [[0, 0, 0]]

    def get_pub_msg(
            self):  # currently outputting the absolute vectors between objects, not in the perspective of the sub
        # print(self.object_pos)
        message = Vision_Array()
        # initializing message
        if len(self.relative_pos) != 0 and len(self.world_object_names) != 0:
            message.data = []
            for i in range(len(self.relative_pos)):
                message.data.append(Vision())
                message.data[i].obj_data = make_vector(self.relative_pos[i])
                message.data[i].identifier = self.world_object_names[i]

        self.pub_msg = message

    def run(self):
        while not rospy.is_shutdown():
            # self.set_rate()
            self.calc_rel_pos()
            self.get_pub_msg()
            self.relative_pos_pub.publish(self.pub_msg)
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break
