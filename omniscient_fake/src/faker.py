#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, Vector3, Pose, PoseArray
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from aquadrone_msgs.msg import SubState





class Faker:
    def __init__(self):
        self.relative_pos_pub = rospy.Publisher("omni_faker", Float64MultiArray, queue_size = 1)
        self.object_pos_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.get_obj_pos)
        self.object_pos = []
        self.sub_pos = [0,0,0]
        self.relative_pos = []
        self.rate = rospy.Rate(20)
        self.pub_msg = Float64MultiArray()

    def get_obj_pos(self, data):
		names = data.name
		model_pose = data.pose
		model_pos = []
		for pose in model_pose:
			model_pos.append(pose.position)
		self.object_pos = []
		if(len(names) != 0):
		    for i in range(len(names)):
				if(names[i] != "aquadrone"):
					self.object_pos.append([model_pos[i].x, model_pos[i].y, model_pos[i].z])
				else:
					self.sub_pos = [model_pos[i].x, model_pos[i].y, model_pos[i].z]
		else:
			self.sub_pos = []

    def calc_rel_pos(self):
        self.relative_pos = []
        if len(self.object_pos) != 0 and len(self.sub_pos) != 0:
            for i in range(len(self.object_pos)):
                self.relative_pos.append([  self.object_pos[i][0] - self.sub_pos[0],
                            				self.object_pos[i][1] - self.sub_pos[1],
                            				self.object_pos[i][2] - self.sub_pos[2],])
        else:
            self.relative_pos = [[0,0,0]]
    
    def get_pub_msg(self):
		message = Float64MultiArray()
		#initializing message
		
		message.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

		message.layout.dim[0].label = "point_array"
		message.layout.dim[0].size = len(self.relative_pos)
		message.layout.dim[0].stride = 3*len(self.relative_pos)

		message.layout.dim[1].label = "point"
		message.layout.dim[1].size = 3
		message.layout.dim[1].stride = 3

		message.layout.data_offset = 0

		message.data = []

		for i in range(len(self.relative_pos)):
			for j in range(3):
				message.data.append(self.relative_pos[i][j])
		
		self.pub_msg = message

    def run(self):
        while not rospy.is_shutdown():
            #self.set_rate()
            self.calc_rel_pos()
            self.get_pub_msg()
            self.relative_pos_pub.publish(self.pub_msg)
	    self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("omniscient_fake")

    faker = Faker()
    faker.run()

