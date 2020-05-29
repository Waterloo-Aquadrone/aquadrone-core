#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from aquadrone_msgs.msg import Vision_Array, Vision





class Omniscient_Vision:
	def __init__(self):
		self.relative_pos_pub = rospy.Publisher("Vision_Data", Vision_Array, queue_size = 1)
		self.object_pos_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.get_obj_pos)
		self.object_pos = []
		self.object_ident = []
		self.sub_pos = [0,0,0]
		self.relative_pos = []
		self.rate = rospy.Rate(20)
		self.pub_msg = Vision_Array()
		self.testing = False

	def get_obj_pos(self, data):
		names = data.name
		model_pos = [pose.position for pose in data.pose]
		temp_obj = []
		temp_sub = []
		temp_ident = []
		if(not self.testing):
			if(len(names) != 0):
				for i in range(len(names)):
					if(names[i] != "aquadrone"):
						temp_obj.append([model_pos[i].x, model_pos[i].y, model_pos[i].z])
						temp_ident.append(names[i])
					else:
						temp_sub = [model_pos[i].x, model_pos[i].y, model_pos[i].z]
		else:
			temp_obj = [[0,0,1],[0,0,1],[0,0,1],[0,0,1]]
			temp_sub = [0,0,0]
			temp_ident = ["a","a","a","a"]
		self.object_pos = temp_obj
		self.sub_pos = temp_sub
		self.object_ident = temp_ident


	def calc_rel_pos(self):
		self.relative_pos = []
		if len(self.object_pos) != 0 and len(self.sub_pos) != 0:
			for i in range(len(self.object_pos)):
				self.relative_pos.append([  self.object_pos[i][0] - self.sub_pos[0],
                            				self.object_pos[i][1] - self.sub_pos[1],
                            				self.object_pos[i][2] - self.sub_pos[2]])
		else:
			self.relative_pos = [[0,0,0]]
    
	def get_pub_msg(self): #currently outputting the absolute vectors between objects, not in the perspective of the sub 
		#print(self.object_pos)
		message = Vision_Array()
		#initializing message
		if(len(self.relative_pos) != 0 and len(self.object_ident) != 0):
			message.data = []		
			for i in range(len(self.relative_pos)):
				message.data.append(Vision())	
				new_data = Vector3()
				new_data.x = self.relative_pos[i][0]
				new_data.y = self.relative_pos[i][1]
				new_data.z = self.relative_pos[i][2]
				message.data[i].obj_data = new_data
				message.data[i].identifier = self.object_ident[i]
		
		self.pub_msg = message

	def run(self):
		while not rospy.is_shutdown():
            #self.set_rate()
			self.calc_rel_pos()
			self.get_pub_msg()
			self.relative_pos_pub.publish(self.pub_msg)
		self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("omniscient_vision_node")

    faker = Omniscient_Vision()
    faker.run()

