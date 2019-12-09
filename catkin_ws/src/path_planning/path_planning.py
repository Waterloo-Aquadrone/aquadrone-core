import numpy as np

class Vector:
	def __init__(self, x, y, z):
		self.x = x
		self.y = y 
		self.z = z

	def mean(*vectors):
		x = sum(v.x for v in vectors) / len(vectors)
		y = sum(v.y for v in vectors) / len(vectors)
		z = sum(v.z for v in vectors) / len(vectors)
		return Vector(x, y, z)

	def std_mean(*vectors):
		"""
		:@return: the component-wise standard deviation of the mean of a set of vectors with the given standard deviations
		"""
		x = (sum(v.x ** 2 for v in vectors) ** 0.5) / len(vectors)
		y = (sum(v.y ** 2 for v in vectors) ** 0.5) / len(vectors)
		z = (sum(v.z ** 2 for v in vectors) ** 0.5) / len(vectors)
		return Vector(x, y, z)

class Submarine:
	def __init__(self, position, direction, velocity, angular_velocity):
		self.position = position
		self.direction = direction
		self.velocity = velocity
		self.angular_velocity = angular_velocity
		self.position_std = None
		self.direction_std = None
		self.velocity_std = None
		self.angular_velocity_std = None

	def set_uncertainties(self, position, direction, velocity, angular_velocity):
		self.position_std = position
		self.direction_std = direction
		self.velocity_std = velocity
		self.angular_velocity_std = angular_velocity

class Gate:
	def __init__(self, top_corners, bottom_corners):
		self.top_corners = top_corners
		self.bottom_corners = bottom_corners
		self.position = Vector.mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])

	def set_uncertainties(self, top_corners, bottom_corners):
		self.top_corners_std = top_corners
		self.bottom_corners_std = bottom_corners
		self.position_std = Vector.std_mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])

class CompetitionMap:
	def __init__(self, sub):
		self.sub = sub
		self.gate = None

	def set_gate(gate):
		self.gate = gate

class PID:
	off_state =  {'x' : None, 'y' : None, 'z' : None, 'r' : None, 'p' : None, 'y' : None}
	def __init__(self):
		self.desired_state = PID.off_state

	def set_desired_state(self, mode, values):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is a new desired state for the PID to aim for
		self.desired_state = PID.off_state
		for i in range(len(mode)):
			self.desired_state[mode[i]] = values[i]

	def process_data(sub):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is updated location data from localization and mapping.
		pass

	def apply_thrust():
		# TODO: add ROS code to publish to a topic when calling this function
		pass

class PathPlanner:
	def __init__(self):
		self.state = 'GATE'
		self.searching_tolerance = 5  # the maximum standard deviation in position of an object before it is considered lost
		self.searched_areas = []

	def process_business_command(self, state):
		"""
		TODO: add ROS code to subscribe to a topic and call this function
		Called whenever there is an updated high level task from the business/competition logic
		"""
		self.state = state


	def process_data(self, competition_map):
		"""
		# TODO: add ROS code to subscribe to a topic and call this function
		Called whenever there is updated location data from localization and mapping.
		"""
		if self.state == 'GATE':
			if competition_map.gate is None or competition_map.gate.position_std > self.searching_tolerance:
				search_for_gate(sub, gate)
			else:
				go_through_gate(sub, gate)

	def search_for_gate(sub, gate=None):
		# gate can be None
		pass

	def go_through_gate(sub, gate):
		pass

	def send_pid_command(mode, values):
		"""
		# TODO: add ROS code to publish to a topic when calling this function
		"""
		pass
