import numpy as np
import rospy
from data_structures import Vector, Submarine, Gate, Pole, Wall, CompetitionMap, PIDCommand


class PathPlanner:
	def __init__(self, node_name='path_planner', business_topic='business', localization_topic='localization', pid_topic='pid_command'):
		rospy.init_node(node_name)
		rospy.Subscriber(business_topic, String, process_business_command, queue_size=1)
		rospy.Subscriber(localization_topic, CompetitionMap, process_data, queue_size=1)
		self.pid_publisher = rospy.Publisher(pid_topic, PIDCommand, queue_size=1)

		self.state = 'GATE'
		self.searching_tolerance = 5  # the maximum standard deviation in position of an object before it is considered lost
		self.searched_areas = []


	def process_business_command(self, state):
		"""
		Called whenever there is an updated high level task from the business/competition logic
		:param state: The string specifying the new state
		"""
		self.state = state


	def process_data(self, competition_map):
		"""
		# TODO: add ROS code to subscribe to a topic and call this function
		Called whenever there is updated location data from localization and mapping.
		"""
		if self.state == 'GATE':
			if competition_map.gate is None or competition_map.gate.position_std.magnitude() > self.searching_tolerance:
				search_for_gate(competition_map)
			else:
				go_through_gate(sub, gate)
		else if self.state == 'POLE':
			if competition_map.pole is None or competition_map.pole.position_std.magnitude() > self.searching_tolerance:
				search_for_pole(competition_map)
			else:
				go_around_pole(sub, pole)
		else:
			raise Exception('Unknown state: ' + self.state)

	
	def search_for_gate(self, competition_map):
		# Example code:
		sub = competition_map.sub
		current_x = sub.position.x
		deisred_x = current_x + 5 ## meters
		send_pid_command('xyzrp', [deisred_x, sub.position.y, sub.position.z, 0, 0])


	def go_through_gate(self, sub, gate):
		"""
		Given sub's current 
		"""
		pass


	def search_for_pole(self, competition_map):
		pass


	def go_around_pole(self, sub, pole):
		pass


	def send_pid_command(self, mode, values):
		self.pid_publisher.publish(PIDCommand(mode, values))
