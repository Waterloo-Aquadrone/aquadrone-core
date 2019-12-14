import rospy
from data_structures import Vector, Submarine, CompetitionMap, PIDCommand, ThrusterCommand


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