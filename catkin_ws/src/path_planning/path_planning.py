import numpy as np
import rospy
from data_structures import Vector, Submarine, Gate, Pole, Wall, CompetitionMap, PIDCommand, ThrusterCommand


class PID:
	def __init__(self, node_name='pid', pid_command_topic='pid_command', 
				 localization_topic='localization', thruster_command_topic='thruster_command'):
		rospy.init_node(node_name)
		rospy.Subscriber(pid_command_topic, PIDCommand, set_desired_state, queue_size=1)
		rospy.Subscriber(localization_topic, CompetitionMap, process_data, queue_size=1)
		thruster_publisher = rospy.Publisher(thruster_command_topic, ThrusterCommand, queue_size=1)

		self.desired_state = PIDCommand.OFF_COMMAND.copy()

		# TODO: tune PD constants, implement integration functionality
		self.linear_proportional = Vector(1, 1, 1)
		self.linear_derivative = Vector(0.1, 0.1, 0.1)
		self.angular_proportional = Vector(1, 1, 1)
		self.angular_derivative = Vector(0.1, 0.1, 0.1)


	def set_desired_state(self, pid_command):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is a new desired state for the PID to aim for
		self.desired_state = pid_command.desired_state


	def process_data(self, competition_map):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is updated location data from localization and mapping.
		sub = competition_map.sub
		
		acceleration = Vector(0, 0, 0)
		if self.desired_state['x'] is not None:
			acceleration.x = self.linear_proportional.x * (self.desired_state['x'] - sub.postition.x) - self.linear_derivative.x * sub.velocity.x
		if self.desired_state['y'] is not None:
			acceleration.y = self.linear_proportional.y * (self.desired_state['y'] - sub.postition.y) - self.linear_derivative.y * sub.velocity.y
		if self.desired_state['z'] is not None:
			acceleration.z = self.linear_proportional.z * (self.desired_state['z'] - sub.postition.z) - self.linear_derivative.z * sub.velocity.z
		
		angular_acceleration = Vector(0, 0, 0)
		if self.desired_state['r'] is not None:
			angular_acceleration.x = self.angular_proportional.x * (self.desired_state['r'] - sub.direction.x) - self.angular_derivative.x * sub.angular_velocity.x
		if self.desired_state['p'] is not None:
			angular_acceleration.y = self.angular_proportional.y * (self.desired_state['p'] - sub.direction.y) - self.angular_derivative.y * sub.angular_velocity.y
		if self.desired_state['w'] is not None:
			angular_acceleration.z = self.angular_proportional.z * (self.desired_state['w'] - sub.direction.z) - self.angular_derivative.z * sub.angular_velocity.z

		apply_thrust(acceleration, angular_acceleration)



	def apply_thrust(self, acceleration, angular_acceleration):
		# TODO: add ROS code to publish to a topic when calling this function
		thruster_publisher.publish(ThrusterCommand(acceleration, angular_acceleration))


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
